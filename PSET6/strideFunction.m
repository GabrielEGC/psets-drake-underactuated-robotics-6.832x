function xf = strideFunction(p, x0)
    traj = simulate(p,[0 1], [1; x0]);
    xf = traj.traj{2}.eval(traj.traj{2}.tspan(1));
end

