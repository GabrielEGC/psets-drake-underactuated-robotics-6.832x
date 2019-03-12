classdef AcrobotController < DrakeSystem
  properties
    p
    K
    S
  end
  methods
    function obj = AcrobotController(plant)
      obj = obj@DrakeSystem(0,0,4,1,true,true);
      obj.p = plant;
      obj = obj.setInputFrame(plant.getStateFrame);
      obj = obj.setOutputFrame(plant.getInputFrame);
    end
    
    function u = output(obj,t,~,x)
      q = x(1:2);
      qd = x(3:4);

      
      % unwrap angles q(1) to [0,2pi] and q(2) to [-pi,pi]
      q(1) = q(1) - 2*pi*floor(q(1)/(2*pi));
      q(2) = q(2) - 2*pi*floor((q(2) + pi)/(2*pi));

      %%%% put your controler here %%%%
      % You might find some of the following functions useful
      % [H,C,B] = obj.p.manipulatorDynamics(q,qd);
      % com_position = obj.p.getCOM(q);
      % mass = obj.p.getMass();
      % gravity = obj.p.gravity;
      % Recall that the kinetic energy for a manpulator given by .5*qd'*H*qd
       k1=8; kp=8 ;kd=6; th=100;
      
      x_0=[pi;0;0;0];
      [H,C,B] = obj.p.manipulatorDynamics(q,qd);
      com_position = obj.p.getCOM(q);
      com_posd = obj.p.getCOM(x_0);
      mass = obj.p.getMass();
      gravity = obj.p.gravity;
      KE=0.5*qd'*H*qd;
      PE=-mass*gravity(3)*com_position(2);
      E=KE+PE;
      Ed=-mass*gravity(3)*com_posd(2);
      
      ue=k1*(Ed-E)*qd(2);
      
      y=-kp*q(2)-kd*qd(2);%PD Controller
      Hi=inv(H);
      ud=(Hi(2,1)*C(1)+y)/Hi(2,2)+C(2); %PFL
      
      if(([q(1);q(2);qd]-x_0)'*obj.S*([q(1);q(2);qd]-x_0)>th)
        u=ud+ue;
        %disp('E+PD');
      else
        u = -obj.K*([q(1);q(2);qd]-x_0);
        %disp('LQR');
      end  
      % leave this line below, it limits the control input to [-20,20]
      u = max(min(u,20),-20);
      % This is the end of the function
    end
  end
  
  methods (Static)
    function [t,x,x_grade]=run()
      plant = PlanarRigidBodyManipulator('Acrobot.urdf');
      
      [f,df] = plant.dynamics(0,[pi;0;0;0],0);
      A = df(:,2:5);
      B = df(:,6);
      Q = diag([10 10 1 1]);
      R = 0.1;
      [Ku,Su] = lqr(A,B,Q,R);
      
      controller = AcrobotController(plant);

      controller.K = Ku;
      controller.S = Su;
      
      v = plant.constructVisualizer;
      sys_closedloop = feedback(plant,controller);
      
      x0 = [.1*(rand(4,1) - 1)]; % start near the downward position
%      x0 = [pi - .1*randn;0;0;0];  % start near the upright position
      xtraj=simulate(sys_closedloop,[0 10],x0);
      v.axis = [-4 4 -4 4];
      playback(v,xtraj);
      t = xtraj.pp.breaks;
      x = xtraj.eval(t);
      t_grade = linspace(3,4,98);
      x_grade = [xtraj.eval(0) xtraj.eval(t_grade) xtraj.eval(10)];
      x_grade = x_grade';
    end
  end
end
