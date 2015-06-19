function ann_outputs = anneval_large(inputs, weights, biases, input_count, hidden_count)
    ann_outputs = zeros(1, size(inputs,2));
    for i = 1:size(inputs,2)
        [ann_outputs(i), ~, ~, ~] =...
            anneval(inputs(:,i).', weights, biases, input_count, hidden_count);
    end
end

