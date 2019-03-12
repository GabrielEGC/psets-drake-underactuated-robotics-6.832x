g = 9.81;   %gravity
z_cm = 1.1; %height of the center of mass
A = [zeros(2) eye(2); zeros(2,4)];
B = [zeros(2);eye(2)];
C = [eye(2) zeros(2)];
D = -eye(2)*z_cm/g;
Qzmp = eye(2);

Q = C'*Qzmp*C;
R = D'*Qzmp*D;
N = C'*Qzmp*D;

[K,S] = lqr(A,B,Q,R,N);
      