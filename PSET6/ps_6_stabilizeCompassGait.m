% Compass Gait plant
p = CompassGaitPlant();

%% Get limit cycle initial condition (FILL ME IN) %%%%%%%%%%%%%%%%%%%%%%%%%
x0 =  [  -0.323388544149323
   0.218668789029664
  -0.377182130914653
  -1.091826912735189]; % Your x0 from part (b) of the problem "Poincare analysis on compass gait"
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Simulate forwards from x0
xf = strideFunction(p,x0); % Your strideFunction from the previous problem

% Check that the resulting trajectory is periodic
if max(abs(xf-x0)) > 1e-4
    error('Simulating x0 does not result in a limit cycle');
end
%% 
% Simulate forwards from x0 to get the trajectory
xtraj = simulate(p,[0 1], [1; x0]);
xtraj = xtraj.traj{1}; % Extract first part of the trajectory (before collision)
ts = xtraj.getBreaks(); % Get time samples of trajectory
utraj = PPTrajectory(spline(ts,zeros(1,length(ts)))); % Define nominal u(t),
                                                      % which is all zeros since our
                                                      % system was passive                                                      
                                                      
% Set frames
xtraj = xtraj.setOutputFrame(p.modes{1}.getStateFrame);
utraj = utraj.setOutputFrame(p.modes{1}.getInputFrame);


%% Stabilize using tvlqr (Fill in jump equation) %%%%%%%%%%%%%%%%%%%%%%%%%%
% Define Q, Qf, R
Q = diag([10 10 1 1]);
R = 1;
Qf = Q;

options = struct();

converged = false;

%%
a=0
while ~converged
    a=a+1
% tvlqr for continuous phase
[tv,V] = tvlqr(p.modes{1},xtraj,utraj,Q,R,Qf,options);
QfV = Qf;
% Jump equation (FILL ME IN) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[xp,mode,status,dxp] = collisionDynamics(p,1,xtraj.tspan(2),xtraj.eval(xtraj.tspan(2)),utraj);
Ad=dxp(:,3:6);

S_t_plus = V.S.eval(0);
S_t_minus = Ad'*S_t_plus*Ad;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set Qf = to S_t_minus
Qf = S_t_minus;
% Check for convergence (FILL ME IN)
if max(max(abs(Qf-QfV)))<1e-4
    converged = true;
end
end

Qf_converged = Qf;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Setup tvlqr controller and simulate from x0 (Don't modify this block of code)

% Extend nominal limit cycle a little bit (see footnote at the bottom for
% an explanation of this - feel free to ignore this if you find it confusing
% This is a practical implementation issue.) 
tExtended = 1.0;                                                  
xtrajExtended = p.modes{1}.simulate([0 tExtended],x0); 
utrajExtended = PPTrajectory(spline(linspace(0,tExtended,100),zeros(1,100)));
xtrajExtended = xtrajExtended.setOutputFrame(p.modes{1}.getStateFrame);
utrajExtended = utrajExtended.setOutputFrame(p.modes{1}.getInputFrame);

[tv,V] = tvlqr(p.modes{1},xtrajExtended,utrajExtended,Q,R,Qf,options);

% Set frames of tvlqr controller
tv = tv.inOutputFrame(p.getInputFrame);
tv = tv.inInputFrame(p.getOutputFrame);

pmodel = SimulinkModel(p.getModel());

tv = tv.setInputFrame(pmodel.getOutputFrame);
tv = tv.setOutputFrame(pmodel.getInputFrame);

% Closed loop system (i.e., feedback system with TVLQR controller)
sysClosedLoop = feedback(pmodel,tv);

% Visualizer
v = CompassGaitVisualizer(p);

%% Simulate from x0 (syntax is useful for part (b)) %%%%%%%%%%%%%%%%%%%%%%%
xtrajSim = sysClosedLoop.simulate([0 1.0], [1;xtraj.eval(0)]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
% Play trajectory back
xtrajSim = xtrajSim.setOutputFrame(p.getOutputFrame);
v.playback(xtrajSim);
%% Controlling the Compass Gait (Part II)

M=zeros(4);
N=zeros(4);
itn=0;
xm=eye(4);
for it=1:4
    x01 = x0+0.0001*xm(:,it);
    xf = strideFunction(sysClosedLoop, x01);
    itn=itn+1;
    N(:,itn)=x01-x0;
    M(:,itn)=xf-x0;
end
A=M*N^-1
eig(A)


%% Footnote (feel free to ignore) %%
% Since the continuous portion of our nominal trajectory is only defined for a finite amount of time
% [0,ts(end)], we need to deal with cases where the compass gait doesn't
% make contact with the ground before ts(end). Our solution here is to
% extend the nominal trajectory (xtrajExtended) a little beyond ts(end) (by just simulating
% the passive system forwards for longer). This is a reasonable thing to do,
% but still somewhat of a hack. However, this is an important
% implementation issue and worth thinking about in practice.
%%