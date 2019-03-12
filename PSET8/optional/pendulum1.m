function dxdt = pendulum1(t,x,u)
g = 9.81;
b = 0.1;
dxdt = [x(2); u-g*sin(x(1))-b*x(2)];
end