function new_vert = extendLQR(closest_vert,xy,K)
    dt = 0.1;
    x=closest_vert;
    u=-K*(x-xy);%u0=0
    umax=5;
    if u>umax
        u=umax;
    elseif u<-umax
        u=-umax;
    end
    
    [t,x] = ode45(@(t,x) pendulum1(t,x,u),[0 dt], closest_vert);
    new_vert = x(end,:)';
    new_vert(1) = mod(x(end,1)+pi/2,2*pi)-pi/2;
    
end