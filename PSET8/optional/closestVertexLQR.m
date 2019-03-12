function [closest_vert,K,I] = closestVertexLQR(rrt_verts,xy)
    xy2=xy;
    if xy(1)>pi/2
        xy2(1)=xy(1)-2*pi;
    else
        xy2(1)=xy(1)+2*pi;
    end
    
    x0=xy;
    Q=eye(2);R=0.1;
    A=[0 1; -9.81*cos(x0(1)) -0.1];B=[0;1];
    [K,S]=lqr(A,B,Q,R);
    
    diffe=[rrt_verts-xy*ones(1,size(rrt_verts,2)) rrt_verts-xy2*ones(1,size(rrt_verts,2))];
    [~,I]=min(sum((diffe'*S)'.*diffe));
    try
        closest_vert = rrt_verts(:,I(1));
        I=I(1);
    catch
        closest_vert = rrt_verts(:,I(1)-size(rrt_verts,2));
        I=I(1)-size(rrt_verts,2);
    end    
end