function collFree = isCollisionFree(Obs,xy)
    n = size(Obs,2);
    for i = 1:n
        xi = Obs{i};
        x0 = xy;
        K = convhull(xi(1,:),xi(2,:));
        in = inpolygon(x0(1,:),x0(2,:),xi(1,K),xi(2,K));
        if in == 1
            break
        end
    end
    collFree = 1-in;
end