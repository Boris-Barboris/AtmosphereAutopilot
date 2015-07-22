%%
x = 0:0.25:30.0;
y = 1.0 ./ (x * 25.0 + 1.0);
plot(x, y)
%%
w_min = 0.005;
ans = (1.0 - w_min) / (w_min * 50.0)
%%
x = 0:0.1:30.0;
y = 1.0 ./ exp(x * 0.05);
plot(x, y)
%%
x = -5:0.05:5;
y = tansig(x);
plot(x, y);