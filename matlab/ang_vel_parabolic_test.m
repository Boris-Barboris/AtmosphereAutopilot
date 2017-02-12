x = linspace(-0.1, 1.0, 500);
k = -1.0;
d = -0.01;
dt = 0.025;
v_error = -0.1;

casat = d .* x;

b_s = 2.0 * k * dt + d;
a_s = k;
c_s = k * dt * dt + d * d / 4.0 / k - v_error;
D_s = b_s * b_s - 4.0 * a_s * c_s;
s1 = (-b_s + sqrt(D_s)) / 2.0 / a_s;
s2 = (-b_s - sqrt(D_s)) / 2.0 / a_s;
s = max(s1, s2);
b = d * s + d * d / 4.0 /  k;
parab = k .* (x - s) .^ 2 + b;

plot(x, casat);
hold on
plot(x, parab);

((k * s * s + b) - v_error) / dt
k * (-dt - s)^2 + b