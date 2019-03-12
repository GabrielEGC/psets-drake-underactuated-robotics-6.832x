function new_vert = extendEuclidean(x0,xy)
    u = linspace(-5,5,20);
    g = 9.81;
    b = 0.1;
    dt = 0.1;
    dxy0dt = [x0(2); -g*sin(x0(1))-b*x0(2)]*ones(1,length(u))+[0;1]*u;
    xy0u = x0*ones(1,length(u))+dxy0dt*dt;
    
    xy02=xy0u;
    if xy0u(1)>pi/2
        xy02(1)=xy0u(1)-2*pi;
    else
        xy02(1)=xy0u(1)+2*pi;
    end
    
    [~,I] = min(sum([(xy*ones(1,length(u))-xy0u).^2 (xy*ones(1,length(u))-xy02).^2]));
    
    try
        u = u(I(1));
    catch
        u = u(I(1)-size(u,2));
    end
    %disp(u)
    [t,x] = ode45(@(t,x) pendulum1(t,x,u),[0 dt], x0);
    new_vert = x(end,:)';
    new_vert(1) = mod(x(end,1)+pi/2,2*pi)-pi/2;
end