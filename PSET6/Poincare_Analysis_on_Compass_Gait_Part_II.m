clear all, close all, clc
p = CompassGaitPlant();
x0=Inf;
xf=[0;0;2;-0.4];
etol=10^-4;
it=0;
while(1)
    it=it+1;
    x0 = xf; xf = strideFunction(p, x0);
    if norm(xf-x0)<etol
        break;
    end
end
format long
disp(xf)