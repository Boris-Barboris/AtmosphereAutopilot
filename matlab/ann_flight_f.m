function err = ann_flight_f(gen_time_decay, gen_importance)
    %% import telemetry
    run('import_telemetry');

    %% prepare data structures

    % GLOBAL TRAINING ENVIRONMENT
    global_inputs = [aoa; control];
    global_outputs = acc .* 1e4 ./ p ./ airspd.^2;      % divide acceleration by dynamic pressure

    % ANN SECTION
    hidden_count = int16(6);                            % hidden layer neuron count
    input_count = int16(size(global_inputs,1));         % number of network inputs
    mu = 1e-3;
    mu_min = 1e-7;
    mu_max = 1e6;
    tau = 100;
    grad_min = 1e-7;                % target gradient of mean square error
    grad_min_iter_limit = 4;        % exit training after this amount of minimal gradient
    grad_min_iter = 0;
    % create randomized weights and biases
    weights = 2 .* (rand(1, hidden_count*(input_count + 1)) - 0.5);
    biases = 2 .* (rand(1, hidden_count + 1) - 0.5);

    % TELEMETRY CACHE
    % immediate buffer contains last state vectors
    imm_buf_size = int16(19);
    imm_buf_input = zeros(input_count, imm_buf_size);
    imm_buf_output = zeros(1, imm_buf_size);
    imm_buf_count = 0;
    imm_buf_head = 1;
    % stohastic buffer contains random state vectors, wich left immediate buffer
    stoh_buf_size = int16(1);
    stoh_prob = 1.1;        % probability of new state vector to be included in stohastic buffer
    stoh_buf_input = zeros(input_count, stoh_buf_size);
    stoh_buf_output = zeros(1, stoh_buf_size);
    stoh_buf_count = 0;
    stoh_buf_head = 1;
    % generalization space is required for reliable ann convergence towards model
    gen_buf_dims = int16([13, 13]);
    gen_buf_upper = [0.25, 1.1];
    gen_buf_lower = [-0.25, -1.1];
    gen_buf_avg_factor = 4;         % continuous inputs to one cell will be smoothed by this factor
    %gen_time_decay = 1;             % importance of old generalization data is decreased by this factor
    %gen_importance = 1;             % general error weight of generalization space
    temp = cumprod(gen_buf_dims);   % temp array for deriving a linear size of generalization buffer
    gen_buf_size = temp(end);
    gen_buf_input = zeros(input_count, gen_buf_size) + NaN;
    gen_buf_output = zeros(1, gen_buf_size) + NaN;
    gen_buf_birth = zeros(1, gen_buf_size) + NaN;
    gen_linear_index = 0;

    % ANN OUTPUT VECTOR
    ann_global_outputs = zeros(1, length(global_outputs));

    %% commit simulation

    % SIMULATION SETTINGS
    cpu_ratio = 0.33;      % amount of ann training iterations per 1 game physics frame
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

        % update generalization space
        gen_index = maptocell(global_inputs(:,frame).',...
            gen_buf_lower, gen_buf_upper, gen_buf_dims);
        if gen_linear_index == coord2index(gen_index, gen_buf_dims)
            % we're writing into the same cell as in the last frame
            gen_buf_input(:,gen_linear_index) =...
                (global_inputs(:,frame) + (gen_buf_avg_factor-1) .* gen_buf_input(:,gen_linear_index))...
                ./ gen_buf_avg_factor;
            gen_buf_output(:,gen_linear_index) =...
                (global_outputs(:,frame) + (gen_buf_avg_factor-1) .* gen_buf_output(:,gen_linear_index))...
                ./ gen_buf_avg_factor;
        else
            gen_linear_index = coord2index(gen_index, gen_buf_dims);
            gen_buf_input(:,gen_linear_index) = global_inputs(:,frame);
            gen_buf_output(:,gen_linear_index) = global_outputs(:,frame);
        end
        gen_buf_birth(1,gen_linear_index) = frame * 0.02;   % remember the birth time
        % try to apply symmetry assumption
        gen_index_symm = gen_buf_dims - gen_index + 1;
        gen_linear_index_symm = coord2index(gen_index_symm, gen_buf_dims);
        % 
        if (gen_linear_index_symm ~= gen_linear_index && isnan(gen_buf_output(:,gen_linear_index_symm)))
            gen_buf_input(:,gen_linear_index_symm) = -gen_buf_input(:,gen_linear_index);
            gen_buf_output(:,gen_linear_index_symm) = -gen_buf_output(:,gen_linear_index);
            gen_buf_birth(1,gen_linear_index_symm) = frame * 0.02;
        end

        % if immediate buffer is full
        if imm_buf_count >= imm_buf_size
            % and we have a point in telemetry behind immediate buffer's reach
            if frame - imm_buf_size > 0
                % update stohastic buffer
                dice = rand(1);
                if dice <= stoh_prob
                    % this state vector must be saved in stohastic buffer
                    stoh_buf_input(:,stoh_buf_head) = global_inputs(:,frame-imm_buf_size);
                    stoh_buf_output(:,stoh_buf_head) = global_outputs(:,frame-imm_buf_size);
                    stoh_buf_count = min(stoh_buf_count + 1, stoh_buf_size);
                    stoh_buf_head = stoh_buf_head + 1;
                    if stoh_buf_head > stoh_buf_size
                        stoh_buf_head = 1;
                    end
                end
            end
        end

        % try to perform training iterations
        cpu_time = cpu_time + cpu_ratio;    
        if (cpu_time >= 1)
            % we'll iterate this frame so we need to prepare ANN training set
            % prepare set of excluded ring
            % excluded_ring = zeros(1, (gen_spacing * 2 + 1)^2);

            % get all not NaN generalization outputs
            gen_inputs = zeros(input_count, gen_buf_size);
            gen_outputs = zeros(1, gen_buf_size);
            gen_births = zeros(1, gen_buf_size);
            j = 0;
            for i=1:gen_buf_size
                if ~isnan(gen_buf_output(:,i)) && (i ~= gen_linear_index)
                    j = j+1;
                    gen_inputs(:,j) = gen_buf_input(:,i);
                    gen_outputs(:,j) = gen_buf_output(:,i);
                    gen_births(j) = gen_buf_birth(i);
                end
            end

            % concatenate to unite training vectors
            training_input = [imm_buf_input(:,1:imm_buf_count), stoh_buf_input(:,1:stoh_buf_count),...
                gen_inputs(:,1:j)];
            training_output = [imm_buf_output(:,1:imm_buf_count), stoh_buf_output(:,1:stoh_buf_count),...
                gen_outputs(:,1:j)];
            %training_input = [imm_buf_input(:,1:imm_buf_count), stoh_buf_input(:,1:stoh_buf_count)];
            %training_output = [imm_buf_output(:,1:imm_buf_count), stoh_buf_output(:,1:stoh_buf_count)];

            % prepare input weights vector
            if j > 0
                cur_time = frame * 0.02;
                gen_imm_ratio = j / double(imm_buf_count + stoh_buf_count);
                % base undecayed importance of generalization data point:
                base_gen_weight = gen_importance / gen_imm_ratio;
                decayed_gen_weight = zeros(1, j);
                for i=1:j
                    decayed_gen_weight(i) = base_gen_weight ./...
                        (gen_time_decay * (cur_time - gen_births(i)) + 1);
                end
                input_weights  = [zeros(1, imm_buf_count + stoh_buf_count) + 1, decayed_gen_weight];
            else
                input_weights  = zeros(1, imm_buf_count + stoh_buf_count) + 1;
            end
        end
        while cpu_time >= 1
            % iterate here
            [new_weights, new_biases, old_outputs] =...
                anntrain_lm(training_input, input_weights, training_output,...
                weights, biases, mu, input_count, hidden_count);
            ann_outputs = anneval_large(training_input, new_weights, new_biases, input_count, hidden_count);
            old_sqr_err = meansqr(old_outputs - training_output);
            new_sqr_err = meansqr(ann_outputs - training_output);
            if (new_sqr_err < old_sqr_err)
                weights = new_weights;
                biases = new_biases;
                if mu/tau >= mu_min
                    mu = mu / tau;
                end
                grad = old_sqr_err - new_sqr_err;
                if grad < grad_min
                    grad_min_iter = grad_min_iter + 1;
                    if grad_min_iter >= grad_min_iter_limit
                        cpu_time = 0;
                        break
                    end
                end
                if new_sqr_err == 0
                    cpu_time = 0;
                    break
                end
                %plot(inputs, ann_outputs);
            else
                if mu*tau <= mu_max
                    mu = mu * tau;
                end
            end
            cpu_time = cpu_time - 1;
        end

        % current model acceleration
        ann_global_outputs(1,frame) = anneval_large(global_inputs(:,frame), weights,...
            biases, input_count, hidden_count);
    end

    %% regressor performace analysis
    model_error = ann_global_outputs - smoothed_acc .* 1e4 ./ p ./ airspd.^2;
    cut_model_error = model_error(100:end);
    err = meansqr(cut_model_error);
end
