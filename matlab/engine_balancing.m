A = [-6, -6, 5, 5; -2, 3, -3, 3];
x = [1.0; 1.0; 1.0; 1.0];
c = [1.0; 1.0; 1.0; 1.0];

length = 20;
sizex = size(x, 1);
x_history = zeros(sizex, length);
x_history(:, 1) = x;
terr_history = zeros(size(A, 1), length);

step = 0.005;
thrust_mult = 0.001;

for i=1:length
    terr = A * x;
    grad_torque = zeros(sizex, 1);
    for j=1:size(terr)
        grad_torque = grad_torque + 2.0 * terr(j) * A(j,:)';
    end
    for j=1:size(grad_torque)
        if (x(j, 1) == 1.0) && (grad_torque(j, 1) < 0.0)
            grad_torque(j, 1) = 0.0;
        end
    end
    grad_thrust = - thrust_mult * c;
    for j=1:size(grad_thrust)
        if (x(j, 1) == 1.0) && (grad_thrust(j, 1) < 0.0)
            grad_thrust(j, 1) = 0.0;
        end
    end
    d = dot(grad_thrust, grad_torque);
    if (d < 0.0)
        grad_thrust = grad_thrust - grad_torque * d / dot(grad_torque, grad_torque);
    end;
    grad = step * grad_torque + grad_thrust;
    xold = x;
    x = x - grad;
    proportion = 1.0;
    for j=1:sizex
        if (x(j, 1) > 1.0)
            proportion = min(proportion, (1.0 - xold(j, 1)) / (x(j, 1) - xold(j, 1)));
        end
    end
    if (proportion < 1.0)
        x = xold + proportion * (x - xold);
    end
    x = max(0.0, min(x, 1.0));
    x_history(:, i+1) = x;
    terr_history(:, i) = terr;
end

subplot(2, 2, 1);
plot(x_history(1,:), x_history(2,:), '*-');
subplot(2, 2, 2);
plot(terr_history(1,:));
subplot(2, 2, 3);
plot(x_history(3,:), x_history(4,:), '*-');
subplot(2, 2, 4);
plot(terr_history(2,:));