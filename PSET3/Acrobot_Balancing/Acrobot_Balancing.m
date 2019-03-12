p = PlanarRigidBodyManipulator('Acrobot.urdf');
[f,df] = p.dynamics(0,[pi 0 0 0]',0);
A=df(:,2:5);B=df(:,6);
Q=eye(4);R=1;
[K,S]=lqr(A,B,Q,R);

AcrobotController.run