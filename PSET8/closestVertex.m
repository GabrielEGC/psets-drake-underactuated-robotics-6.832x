function closest_vert = closestVertex(rrt_verts,xy)
    [~,I] = min(sum((rrt_verts-xy*ones(1,size(rrt_verts,2))).^2));
    closest_vert = rrt_verts(:,I);
end