megaclear
width = .25; % half-width of footsteps
length = .5; % distance between each footstep
speed = .5; % m/s average speed
N = 5;       % number of step pairs (2N is number os teps)

% Generate a step sequence
steps = zeros(2,0);
for i=1:N,
  steps = [steps [0; 2*length*(i-1)]];
  steps = [steps [width; 2*length*(i-1) + length/2]];
  steps = [steps [0; 2*length*(i-1) + length]];
  steps = [steps [-width; 2*length*(i-1) + 3*length/2]]; 
end
steps = [steps [0; 2*length*N]];

% Generate initial trajectory, adding static time at the beginning and end
t_beginning = 2;
t_end = 2;
T = length*2*N/speed+t_beginning+t_end; % use average speed while walking
t = linspace(t_beginning,T-t_end,size(steps,2));

if t_beginning > 0
  t = [0 t];
  steps = [[0;0] steps];
end

if t_end > 0
  t = [t T];
  steps = [steps steps(:,end)];
end

g = 9.81;
z_cm = 1.1;

% THE DESIRED ZMP TRAJECTORY
zmp_traj = PPTrajectory(foh(t,steps));


%% YOUR CODE HERE, CREATE A,B,C,D
A = [zeros(2) eye(2); zeros(2,4)];
B = [zeros(2);eye(2)];
C = [eye(2) zeros(2)];
D = -eye(2)*z_cm/g;

p = LinearSystem(A,B,[],[],eye(4),[]);

x0traj = setOutputFrame(ConstantTrajectory(zeros(4,1)),p.getStateFrame);
u0traj = setOutputFrame(ConstantTrajectory(zeros(2,1)),p.getInputFrame);
options.sqrtmethod = false;
options.tspan = [0 T];

Q_zmp = eye(2);

%% YOUR CODE HERE,
% This key shows how the terms in the input to tvlqr align with the problem
% statement
% Qf{1} = Qf
% Qf{2} = qf
% Qf{3} = 0; (a constant term)
%
% Q{1} = Q
% Q{2} = q
% Q{3} = 0; (a constant term)
%
% R{1} = R
% R{2} = r
% R{3} = 0; (a constant term)
%
% options.Ny = N;
%
% HINT!!! You can multiple PPTrajectories, such as zmp_traj, by constants and matrices
% So if you wanted a some F(t) = [2 1;0 1]*x_zmp(t), just write 
% F = [2 1;0 1]*zmp_traj

S =[0.6697         0    0.2243         0
         0    0.6697         0    0.2243
    0.2243         0    0.0751         0
         0    0.2243         0    0.0751];

xfz=[zmp_traj.eval(zmp_traj.tspan(2));0;0];
Qf = cell(3,1);
Qf{1} = S;
Qf{2} = (-2*xfz'*S)';
Qf{3} = 0;


Q{1} = C'*Q_zmp*C;
Q{2} = (-2*zmp_traj'*Q_zmp*C)'
Q{3} = 0;


R{1} = D'*Q_zmp*D;
R{2} = (-2*zmp_traj'*Q_zmp*D)'
R{3} = 0;

options.Ny = C'*Q_zmp*D;

%% Simulate

controller = tvlqr(p,x0traj,u0traj,Q,R,Qf,options);

controller = controller.setOutputFrame(p.getInputFrame);
controller = controller.setInputFrame(p.getOutputFrame);
sys_cl = feedback(p,controller);
%% Plotting
x0 = zeros(4,1);
traj = sys_cl.simulate([0 T],x0);
t = linspace(0,T,1000);
xcm = traj.eval(t);
xzmp_des = zmp_traj.eval(t);

% reconstruct the actual 
zmp_act_traj=traj(1:2) - (controller.D*traj + controller.y0)*(z_cm/g);
zmp_act = zmp_act_traj.eval(t);

plot(xcm(1,:),xcm(2,:),xzmp_des(1,:),xzmp_des(2,:),zmp_act(1,:),zmp_act(2,:))
axis equal
legend('COM','Desired ZMP','ZMP Actual')

% the controller at time 1
controller.output(1,[],traj.eval(1))
