clc, clear all, close all
xi = randn(2,10);
x0 = randn(2,10);

K = convhull(xi(1,:),xi(2,:));
in = inpolygon(x0(1,:),x0(2,:),xi(1,K),xi(2,K));

collisionFree = 1-in;

plot(xi(1,:),xi(2,:),'xr'); hold on
plot(xi(1,K),xi(2,K),'r')
plot(x0(1,find(in==1)),x0(2,find(in==1)),'xb')
plot(x0(1,find(in==0)),x0(2,find(in==0)),'xg')
legend('Initial Points','Convex Hull Points','Interior Test Points','External Test Points')