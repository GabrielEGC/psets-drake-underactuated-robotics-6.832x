function [p,xtraj,utraj,v,x0] = pset5_catch
p = PlanarRigidBodyManipulator('Acrobot.urdf');
p = p.addRobotFromURDF('../../systems/plants/test/ball.urdf',zeros(3,1),zeros(3,1),struct('floating',true));
p = p.setInputLimits(-40,40);

N = 31;
T = 3;

x0 = [0;0;-10;3*5-2-4.5*9.81;0;0;0;3;3*9.81-5;0];

t_init = linspace(0,T,N);

% ********YOUR CODE HERE ********
% Set the initial guess for x and u, should be dim(x) by N and dim(u) by N
% respectively
x_init_vec = x0*ones(1,N);
u_init_vec = rand(1,N);
% *******************************

traj_init.x = PPTrajectory(foh(t_init,x_init_vec));
traj_init.u = PPTrajectory(foh(t_init,u_init_vec));
traj_init.x = traj_init.x.setOutputFrame(p.getStateFrame);
traj_init.u = traj_init.u.setOutputFrame(p.getInputFrame);

traj_opt =  DircolTrajectoryOptimization(p,N,[T/2 T]);

% FOR THE SECOND PART, YOU MIGHT WANT THIS LINE

traj_opt = traj_opt.addFinalCost(@(tt,x) final_state_obj(p,tt,x));
traj_opt = traj_opt.addRunningCost(@running_cost_fun);
traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0),1);
traj_opt = traj_opt.setSolver('fmincon');
traj_opt = traj_opt.setSolverOptions('fmincon','Algorithm','sqp');

catchConstraint = FunctionHandleConstraint([0;0],[0;0],10,@(x) final_state_con(p,x),1);

traj_opt = traj_opt.addStateConstraint(catchConstraint,N);

tic
[xtraj,utraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);
toc

v = p.constructVisualizer;
v.axis = [-5 5 -5 5];
v.playback(xtraj)
v.drawWrapper(3,xtraj.eval(3))

xf = xtraj.eval(3);
[~,dcon] = final_state_con(p,xf);
end

function [f,df] = running_cost_fun(h,x,u)
f = u^2;
df = [0 zeros(1,10) 2*u];
end

function [f,df] = final_state_con(obj,x)
  q = x(1:5);
  qd = x(6:10);
  kinsol = obj.doKinematics(q);
  
  % body index, so p.body(3) is the lower link
  hand_body = 3;
  
  % position of the "hand" on the lower link, 2.1m is the length
  pos_on_hand_body = [0;-2.1];
  
  % Calculate position of the hand in world coordinates
  % the gradient, dHand_pos, is the derivative w.r.t. q
  [hand_pos,dHand_pos] = obj.forwardKin(kinsol,hand_body,pos_on_hand_body);
  
  % ********YOUR CODE HERE ********
  % Calculate f and the gradient df/dx
  % f should be [0;0] if and only if the hand_pos calculated above equals
  % the current position of the ball
  % DO NOT simply pre-calculate the position of the ball at t=3
  % the final time of the trajectory might not be 3!
  f = [q(3)-hand_pos(1);q(4)-hand_pos(2)];
  df = [[[0 0 1 0 0]-dHand_pos(1,1:5)], zeros(1,5); [[0 0 0 1 0]-dHand_pos(2,1:5)], zeros(1,5)];
  %[qd(3)-dHand_pos(1);qd(4)-dHand_pos(2)];
  % *******************************
end


function [f,df] = final_state_obj(obj,T,x)
 % ********YOUR CODE HERE ********
 % For the second part, calculate a cost that rewards a catch point higher
 % in the air. There are lots of ways to do this, for this problem.
 % the derivative df = [df/dT df/dx]
 % where T is the final time.
 % Some solutions will use T, others may not.
  q = x(1:5);
  k = 500;
  f = -k*q(4);
  df = [zeros(1,4) -k zeros(1,6)];
 % *******************************
end