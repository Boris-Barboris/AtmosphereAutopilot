%%
x = 0:0.25:30.0;
y = 1.0 ./ (x * 0.5 + 1.0);
plot(x, y)
%%
w_min = 0.02;
ans = (1.0 - w_min) / (w_min * 5.0)
%%
x = 0:0.1:30.0;
y = 1.0 ./ exp(x * 0.2);
plot(x, y)
%%
x = -5:0.05:5;
y = tansig(x);
plot(x, y);
%%
x = 0:0.01:0.5;
y = 0.2 * exp(x * -8.0);
z = zeros(1, length(x));
z(1) = 0.2;
for j = 2:length(x)
    z(j) = (1 - 0.2 / dt * 0.01) * z(j-1);
end
plot(x, y)
hold on
plot(x, z, 'r')