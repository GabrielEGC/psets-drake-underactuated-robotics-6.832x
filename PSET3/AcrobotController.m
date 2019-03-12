classdef AcrobotController < DrakeSystem
  properties
    p
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
      u = 0;
      %%%% end of your controller %%%%
      
      % leave this line below, it limits the control input to [-20,20]
      u = max(min(u,20),-20);
      % This is the end of the function
    end
  end
  
  methods (Static)
    function [t,x,x_grade]=run()
      plant = PlanarRigidBodyManipulator('Acrobot.urdf');
      controller = AcrobotController(plant);
      v = plant.constructVisualizer;
      sys_closedloop = feedback(plant,controller);
      
%      x0 = [.1*(rand(4,1) - 1)]; % start near the downward position
      x0 = [pi - .1*randn;0;0;0];  % start near the upright position
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
