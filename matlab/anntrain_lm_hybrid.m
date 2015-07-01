function [new_weights, new_biases, old_meansqr] =...
    anntrain_lm_hybrid(inputs, input_weights, outputs, batch_size, batch_weight,...
    weights, biases, mu, input_count, hidden_count)
%ANNTRAIN_LM Iterate once with Levenberg-Marquardt training algorithm over 2-layer approximator ANN
    res_size = size(inputs,2) + 1;
    raw_old_outputs = zeros(1, size(inputs,2));
    param_count = length(weights) + length(biases);
    n1 = zeros(size(inputs,2), hidden_count);   % net outputs of hidden layer
    a1 = zeros(size(inputs,2), hidden_count);   % outputs of hidden layer
    n2 = zeros(1, size(inputs,2));              % net outputs of output layer
    for i = 1:size(inputs,2)
        [raw_old_outputs(i), n1(i,:), n2(i), a1(i,:)] =...
            anneval(inputs(:,i).', weights, biases, input_count, hidden_count);
    end
    errors = raw_old_outputs - outputs;
    jacob = zeros(size(inputs,2) + 1, param_count);
    % let's fill jacobian matrix
    % cycle over inputs
    for i = 1:size(inputs,2)
        % find Marquardt sensitivities using back propagation
        s2 = 1;
        s1 = tanh_deriv(n1(i,:)) .*...
            weights(hidden_count*input_count+1 : hidden_count*(input_count+1));
        % update jacobian for weights and biases of hidden layer for each neuron
        for neuron = 1:hidden_count
            jacob(i, 1 + (neuron-1)*input_count : input_count + (neuron-1)*input_count) =...
                s1(neuron) .* inputs(:,i);
            jacob(i, length(weights)+neuron) = s1(neuron);
        end
        % update jacobian for weights and biases of output layer
        jacob(i, hidden_count*input_count+1 : hidden_count*(input_count+1)) =...
            s2 .* a1(i,:);
        jacob(i, param_count) = s2;
    end
    
    % collapse batch
    jacob(res_size,:) = sum(jacob(1:batch_size, :));
    errors = [errors, sum(errors(1:batch_size))];
    
    % create weight matrix
    weight_mat = diag([input_weights, batch_weight]);

    old_meansqr = meansqr(errors .* [input_weights, batch_weight]);
    
    % descend
    jtwj = jacob.' * weight_mat * jacob;
    A = jtwj + mu .* eye(param_count); %diag(diag(jtwj));
    b = jacob.' * weight_mat * errors.';
    delta_params = A \ b;
    new_params = [weights, biases] - delta_params.';
    [new_weights, new_biases] = arr2weights(new_params, length(weights), length(biases));
end

function df = tanh_deriv(x)
    df = 1 - tanh(x).^2;
end

function [weights, biases] = arr2weights(arr, weights_count, biases_count)
    weights = arr(1:weights_count);
    biases = arr(weights_count+1:weights_count+biases_count);
end
