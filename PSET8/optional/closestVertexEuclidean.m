function closest_vert = closestVertexEuclidean(rrt_verts,xy)
    xy2=xy;
    if xy(1)>pi/2
        xy2(1)=xy(1)-2*pi;
    else
        xy2(1)=xy(1)+2*pi;
    end
    
    [~,I] = min([sum((rrt_verts-xy*ones(1,size(rrt_verts,2))).^2) sum((rrt_verts-xy2*ones(1,size(rrt_verts,2))).^2)]);
    try
        closest_vert = rrt_verts(:,I(1));
    catch
        closest_vert = rrt_verts(:,I(1)-size(rrt_verts,2));
    end
    
end
