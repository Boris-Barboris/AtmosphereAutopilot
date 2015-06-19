function [new_weights, new_biases, old_meansqr] =...
    anntrain_lm(inputs, input_weights, outputs,...
    weights, biases, mu, input_count, hidden_count)
%ANNTRAIN_LM Iterate once with Levenberg-Marquardt training algorithm over 2-layer approximator ANN
    old_outputs = zeros(1, size(inputs,2));
    param_count = length(weights) + length(biases);
    n1 = zeros(size(inputs,2), hidden_count);   % net outputs of hidden layer
    a1 = zeros(size(inputs,2), hidden_count);   % outputs of hidden layer
    n2 = zeros(1, size(inputs,2));              % net outputs of output layer
    for i = 1:size(inputs,2)
        [old_outputs(i), n1(i,:), n2(i), a1(i,:)] =...
            anneval(inputs(:,i).', weights, biases, input_count, hidden_count);
    end
    errors = old_outputs - outputs;
    weighted_errors = errors .* input_weights;
    old_meansqr = meansqr(weighted_errors);
    jacob = zeros(size(inputs,2), param_count);
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
    
    % descend
    new_params = [weights, biases] -...
        (inv(mtimes(jacob.',jacob) + mu .* eye(param_count)) * jacob.' * weighted_errors.').';
    [new_weights, new_biases] = arr2weights(new_params, length(weights), length(biases));
end

function df = tanh_deriv(x)
    df = 1 - tanh(x).^2;
end

function [weights, biases] = arr2weights(arr, weights_count, biases_count)
    weights = arr(1:weights_count);
    biases = arr(weights_count+1:weights_count+biases_count);
end
