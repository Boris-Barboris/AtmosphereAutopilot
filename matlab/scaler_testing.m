pressure = 1.0;
velocity = 100.0;
dyn_pressure = pressure * velocity ^ 2;
acc = -10.0:0.1:10.0;
div_acc = acc ./ dyn_pressure;
normalized_acc = div_acc .* 1e3;
plot(acc, normalized_acc)