% Visualize path

dt = 0.1; % Fill this in. This is the time-step you used for the extend operator in the RRT (e.g., 0.1 seconds)

% Visualize path
v = PendulumVisualizer();
xp1 = unwrap(xpath(:,1)+2*pi);
xtraj = PPTrajectory(spline(0:dt:dt*(length(xpath)-1),[xp1,xpath(:,2)]'));
xtraj = xtraj.setOutputFrame(v.getInputFrame);
v.playback(xtraj);