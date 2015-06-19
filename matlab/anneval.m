function [value, n1, n2, a1] = anneval(input, weights, biases, input_count, hidden_count)
    n1 = zeros(1, hidden_count);
    for neuron = 1:hidden_count
        w = zeros(1, input_count);
        for i = 1:input_count
            w(i) = weights(i + (neuron-1)*input_count);
        end
        b = biases(neuron);
        n1(neuron) = sum(input .* w) + b;
    end
    a1 = tanh(n1);
    w = weights(hidden_count*input_count+1 : hidden_count*(input_count+1));
    b = biases(hidden_count+1);
    n2 = sum(a1 .* w) + b;
    value = tanh(n2);
end

