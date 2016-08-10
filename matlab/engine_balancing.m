A = [4.44, 4.44, -4.24; 7.15, -7.15, -5.43; 2.39, -2.39, 2.28];
x = [1.0; 1.0; 1.0];
c = [1.0; 1.0; 1.0];

length = 50;
sizex = size(x, 1);
sizea = size(A, 1);
x_history = zeros(sizex, length);
x_history(:, 1) = x;
terr_history = zeros(sizea, length);
ideal_thrusts = c .* (ones(sizex, 1) + 9.0);

step = 1e-3;
unbalance_factor = 0.01;
adapt = 10.0;
switched = 0;

step_history = zeros(1, length);

for i=1:length
    if ((i > 10) || (step == 1e-20)) && (switched == 0)
        unbalance_factor = 100;
        switched = 1;
        step = 1e-3;
    end
        
    terr = A * x;
    grad_torque = zeros(sizex, 1);
    for j=1:sizea
        grad_torque = grad_torque + 2.0 * terr(j) * A(j,:)';
    end
    
    sqrerr = unbalance_factor * dot(terr, terr);
    grad_torque = grad_torque * unbalance_factor;
    
    for j=1:sizex
        if (x(j, 1) == 1.0) && (grad_torque(j, 1) < 0.0)
            grad_torque(j, 1) = 0.0;
        end
        if (x(j, 1) == 0.0) && (grad_torque(j, 1) > 0.0)
            grad_torque(j, 1) = 0.0;
        end
    end
    
    trerr = (c .* x - ideal_thrusts) / sizex;
    sqrerr = sqrerr + dot(trerr, trerr);
    
    grad_thrust = 2.0 * c .* trerr / sizex;
    
    for j=1:sizex
        if (x(j, 1) == 1.0) && (grad_thrust(j, 1) < 0.0)
            grad_thrust(j, 1) = 0.0;
        end
        if (x(j, 1) == 0.0) && (grad_thrust(j, 1) > 0.0)
            grad_thrust(j, 1) = 0.0;
        end
    end
    
    d = dot(grad_thrust, grad_torque);
    if (d < 0.0)
        grad_thrust = grad_thrust - grad_torque * d / dot(grad_torque, grad_torque);
    end;
    %grad = step * grad_torque + grad_thrust;
    
    grad = grad_torque + grad_thrust;
    
    for j=1:sizex
        if (x(j, 1) == 1.0) && (grad(j, 1) < 0.0)
            grad(j, 1) = 0.0;
        end
        if (x(j, 1) == 0.0) && (grad(j, 1) > 0.0)
            grad(j, 1) = 0.0;
        end
    end
    
    xold = x;    
    subiter = 0;
    
    while (subiter < 30)
        x = xold - grad * step;
        
        terr = A * x;
        sqrnew = unbalance_factor * dot(terr, terr);
        trerr = (c .* x - ideal_thrusts) / sizex;
        sqrnew = sqrnew + dot(trerr, trerr);
        
        if (sqrnew < sqrerr)
            step = step * (1.0 + 0.5 * (adapt - 1));
            break;
        end
        step = step / adapt;
        if (step < 1e-20)
            step = 1e-20;
            break;
        end
        
        subiter = subiter + 1;
    end
    step_history(1, i) = log10(step);
    
    
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

subplot(3, 2, 1);
plot(x_history(1,:), x_history(2,:), '*-');
subplot(3, 2, 2);
plot(terr_history(1,:));
subplot(3, 2, 3);
plot(x_history(3,:), '*-');
subplot(3, 2, 4);
plot(terr_history(2,:));
subplot(3, 2, 5);
plot(step_history);
subplot(3, 2, 6);
plot(x_history(1,:), 'r');
hold on
plot(x_history(2,:), 'r');
plot(x_history(3,:), 'b');
%plot(x_history(4,:), 'b');
hold off