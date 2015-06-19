
%% sinus approximation

inputs = linspace(-1, 1, 50);
outputs = 0.5 .* (sin(inputs) + 0.2 .* cos(7 .* inputs));
hidden_count = int16(10);
input_count = int16(1);
mu = 1e-3;
mu_min = 1e-8;
mu_max = 1e11;
grad_min = 1e-9;
grad_min_iter_limit = 6;
grad_min_iter = 0;
tau = 100;
weights = 2 .* (rand(1, hidden_count*(input_count + 1)) - 0.5);
biases = 2 .* (rand(1, hidden_count + 1) - 0.5);

ann_outputs = anneval_large(inputs, weights, biases, input_count, hidden_count);
old_sqr_err = meansqr(ann_outputs - outputs);

plot(inputs, outputs, 'r')
hold on
for i = 1:500
    [new_weights, new_biases] =...
        anntrain_lm(inputs, outputs, weights, biases, mu, input_count, hidden_count);
    ann_outputs = anneval_large(inputs, new_weights, new_biases, input_count, hidden_count);
    new_sqr_err = meansqr(ann_outputs - outputs);
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
                break
            end
        end
        if new_sqr_err == 0 
            break
        end
        old_sqr_err = new_sqr_err;
        %plot(inputs, ann_outputs);
    else
        if mu*tau <= mu_max
            mu = mu * tau;
        end
    end
end
ann_outputs = anneval_large(inputs, weights, biases, input_count, hidden_count);
plot(inputs, ann_outputs);
hold off;

