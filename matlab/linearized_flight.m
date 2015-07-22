%% import telemetry
delta_time = 0.025;
run('import_telemetry');
%debug_acc = csvread([ksp_plots_path, 'debug_predict.csv']);
%debug_acc = debug_acc(1:length(debug_acc)-1);

%% prepare data structures

% GLOBAL TRAINING ENVIRONMENT
global_inputs = [aoa; control];
norm_acc = acc .* 2e4 ./ p ./ airspd.^2;      % divide acceleration by dynamic pressure
%norm_smoothed_acc = smoothed_acc .* 1e4 ./ p ./ airspd.^2;
global_outputs = norm_acc;
input_count = int16(size(global_inputs,1));         % number of network inputs

% LINEAR MODEL STORAGE
lin_params = zeros(1, 1 + input_count);
input_changed = false(1, 2);

% TELEMETRY CACHE
% immediate buffer contains last state vectors
imm_buf_size = int16(10);
imm_buf_input = zeros(input_count, imm_buf_size);
imm_buf_output = zeros(1, imm_buf_size);
imm_buf_count = 0;
imm_buf_head = 1;
% generalization space is required for reliable ann convergence towards model
gen_buf_dims = int16([11, 11]);
gen_buf_upper_start = [0.1, 0.1];
gen_buf_upper = gen_buf_upper_start;
gen_buf_lower_start = [-0.1, -0.1];
gen_buf_lower = gen_buf_lower_start;
gen_bound_decay = 0.1;
gen_time_decay = 2.0;           % importance of old generalization data is decreased by this factor
gen_time_decay_linear = 2.0;
gen_time_decay_nonlinear = 50.0;
max_age = 30.0;
gen_importance = 0.1;           % general error weight of generalization space
gen_min_weight = 0.005;
temp = cumprod(gen_buf_dims);   % temp array for deriving a linear size of generalization buffer
gen_buf_size = temp(end);
gen_buf_input = zeros(input_count, gen_buf_size) + NaN;
gen_buf_output = zeros(1, gen_buf_size) + NaN;
gen_buf_birth = zeros(1, gen_buf_size) + NaN;
gen_linear_index = 0;
gen_count = 0;

% linearization chech settings
linear_err_criteria = 0.05;
max_global_output = 0.01;
max_global_output_decay = 0.2;

% MODEL OUTPUT VECTOR
lin_global_outputs = zeros(1, length(global_outputs));
linear_bool = zeros(1, length(global_outputs));
gen_count_stat = zeros(1, length(global_outputs));

%% commit simulation

% SIMULATION SETTINGS
cpu_ratio = 0.5;    % amount of training iterations per 1 game physixcs frame
cpu_time = 0;       % amount of availiable training iterations. When >= 1, we should train once

% MAIN CYCLE
for frame = 1:length(global_inputs)
    
    % update immediate buffer
    imm_buf_input(:,imm_buf_head) = global_inputs(:,frame);
    imm_buf_output(:,imm_buf_head) = global_outputs(:,frame);
    imm_buf_count = min(imm_buf_count + 1, imm_buf_size);
    imm_buf_head = imm_buf_head + 1;
    if imm_buf_head > imm_buf_size
        imm_buf_head = 1;
    end
    
    % adapt time_decay
    max_global_output = max_global_output * (1 - max_global_output_decay * delta_time);
    max_global_output = max([max_global_output, abs(global_outputs(:,frame)), 0.01]);
    
    % update generalization space
    % stretch gen space if needed
    gen_buf_upper = max(gen_buf_upper, global_inputs(:,frame).');
    gen_buf_lower = min(gen_buf_lower, global_inputs(:,frame).');
    % gen space decay over time
    gen_buf_upper = global_inputs(:,frame).' +...
        max((1.0 - delta_time * gen_bound_decay) .* (gen_buf_upper - global_inputs(:,frame).'),...
        (gen_buf_upper_start - gen_buf_lower_start) ./ 2.0);
    gen_buf_lower = global_inputs(:,frame).' +...
        min((1.0 - delta_time * gen_bound_decay) .* (gen_buf_lower - global_inputs(:,frame).'),...
        (gen_buf_upper_start - gen_buf_lower_start) ./ -2.0);
    % insert data
    [gen_coord, cell_center] = maptocell(global_inputs(:,frame).',...
        gen_buf_lower, gen_buf_upper, gen_buf_dims);
    gen_linear_index = coord2index(gen_coord, gen_buf_dims);
    old_coord = gen_buf_input(:,gen_linear_index);
    need2write = false;
    if isnan(old_coord(1))
        need2write = true;
    else
        if (frame * delta_time - gen_buf_birth(1,gen_linear_index) > 1.0)
            need2write = true;
        else
            gen_buf_norm = gen_buf_upper - gen_buf_lower;
            if norm((global_inputs(:,frame).' - cell_center) ./ gen_buf_norm) <...
                    norm((old_coord.' - cell_center) ./ gen_buf_norm)
                need2write = true;
            end
        end
    end
    if need2write
        gen_buf_input(:,gen_linear_index) = global_inputs(:,frame);
        gen_buf_output(:,gen_linear_index) = global_outputs(:,frame);
        gen_buf_birth(1,gen_linear_index) = frame * delta_time;   % remember the birth time
    end
    
    % check linearization
    imm_lin_outputs = [zeros(imm_buf_count,1) + 1.0, imm_buf_input(:,1:imm_buf_count).'] * lin_params.';
    new_sqr_err = meansqr((imm_lin_outputs.' - imm_buf_output(1:imm_buf_count)) ./...
        max_global_output);
    new_max_abs_error = max(abs((imm_lin_outputs.' - imm_buf_output(1:imm_buf_count)) ./...
        max_global_output));
    if new_max_abs_error < linear_err_criteria
        gen_time_decay = gen_time_decay_linear;
        linear_bool(frame) = 0.1;
    else
        gen_time_decay = gen_time_decay_nonlinear;
        linear_bool(frame) = 0.0;
    end

    % try to perform training iterations
    if (cpu_time >= 1)
        % we'll iterate this frame so we need to prepare training data
        cur_time = frame * delta_time;
        
        % get all not NaN generalization outputs
        gen_inputs = zeros(input_count, gen_buf_size);
        gen_outputs = zeros(1, gen_buf_size);
        gen_births = zeros(1, gen_buf_size);
        gen_decayed_weights = zeros(1, gen_buf_size);
        gen_count = 0;
        for i=1:gen_buf_size
            if ~isnan(gen_buf_output(1,i))
                decayed_weight = 1.0 / (gen_time_decay * min(cur_time - gen_buf_birth(i), max_age) + 1.0);
                if decayed_weight > gen_min_weight
                    gen_count = gen_count+1;
                    gen_decayed_weights(gen_count) = decayed_weight;
                    gen_inputs(:,gen_count) = gen_buf_input(:,i);
                    gen_outputs(:,gen_count) = gen_buf_output(:,i);
                    gen_births(gen_count) = gen_buf_birth(i);
                else
                    % delete data from gen buffer because it's too old
                    gen_buf_input(:,i) = NaN(input_count,1);
                    gen_buf_output(:,i) = NaN;
                    gen_buf_birth(:,i) = NaN;
                end                
            end
        end

        % concatenate to unite training vectors
        training_input = [imm_buf_input(:,1:imm_buf_count), gen_inputs(:,1:gen_count)];
        training_output = [imm_buf_output(:,1:imm_buf_count), gen_outputs(:,1:gen_count)];

        % prepare input weights vector
        % decay immediate buffer weights
        decayed_imm_weight = zeros(1, imm_buf_count);
        j = imm_buf_head - 1;
        if j < 1
            j = imm_buf_size;
        end
        for i=1:imm_buf_count
            decayed_imm_weight(j) = 1.0 / (gen_time_decay * (double(i)-1.0) * delta_time + 1.0);
            j = j - 1;
            if j < 1
                j = imm_buf_size;
            end
        end
        if gen_count > 0
            gen_imm_ratio = gen_count / double(imm_buf_count);
            %gen_imm_ratio = 1.0;
            % base undecayed importance of generalization data point:
            base_gen_weight = gen_importance / gen_imm_ratio;
            input_weights  = [decayed_imm_weight, gen_decayed_weights(1:gen_count) .* base_gen_weight];
        else
            input_weights  = decayed_imm_weight;
        end
    end
    
    % update changed flags
    if frame > 1
        temp_changed = (global_inputs(:,frame) ~= global_inputs(:,frame-1)).';
        input_changed = input_changed | temp_changed;
    end
    
    if cpu_time >= 1
        training_basis = size(training_input, 2);
        % prepare matrixes
        X = zeros(training_basis, 1) + 1.0;
        Y = training_output.';
        for i = 1:input_count
            if input_changed(i)
                X = [X, training_input(i,:).'];
            %else
            %    Y = Y - lin_params(i+1) .* training_input(i,:).';
            end
        end 
        % solve linear weighted least squares
        x_w = X.' * diag(input_weights);
        new_lin_params = (x_w * X) \ (x_w * Y);
        % update params
        lin_params(1) = new_lin_params(1);
        j = 2;
        for i = 1:input_count
            if input_changed(i)
                lin_params(i+1) = new_lin_params(j);
                j = j + 1;
            else
                lin_params(i+1) = 0.0;
            end
        end
        cpu_time = cpu_time - 1.0;
    end
    lin_global_outputs(frame) = sum(lin_params .* [1.0, global_inputs(:,frame).']);
    gen_count_stat(frame) = double(gen_count) / double(gen_buf_size);  
    
    cpu_time = cpu_time + cpu_ratio;
end
%% plot graphics
scrsz = get(0,'ScreenSize');
figure('Name','linear online regression',...
    'Position',[100 50 scrsz(3)*0.9 scrsz(4)*0.8])
plot(time, norm_acc, 'r');
hold on
%plot(time, norm_smoothed_acc, 'r:');
plot(time, lin_global_outputs, 'b');
%plot(time, debug_acc, 'b-.');
plot(time, aoa, 'g');
%plot(time, aoa, 'g');
plot(time, control, 'c');
plot(time, linear_bool, 'k:');
plot(time, gen_count_stat, 'k-');
%plot(time, mu_values, 'k-.');
%plot(time, model_error,'k-.');
%plot(time, cumul_error,'m-.');
%plot(time, ann_sqr_errors, 'k-.');
hold off;
xlabel('time')
legend('acc', 'lin model acc', 'aoa', 'control', 'is linear', 'gen count');
h = gca;
set(h, 'Position', [0.035 0.06 0.96 0.92]);