% PROBLEM SETUP, DO NOT CHANGE
N = 4;
T = 5;
y_0 = randn(2,1);
yd_0 = randn(2,1);
ydd_0 = randn(2,1);
y_g = 2 + randn(2,1);

syms t
m=transpose(t.^[0:N]);
mtr=transpose(m);
v=diff(m,3);
Hp=kron(eye(2),v*transpose(v));

%
% QP SETUP HERE
H = 2*int(Hp,t,0,T);
H = eval(H);
f = zeros(2*(N+1),1);
A = zeros(0,2*(N+1));
b = zeros(0);
B = [kron(eye(2),subs(mtr,t,0));
    kron(eye(2),subs(diff(mtr),t,0));
    kron(eye(2),subs(diff(mtr,2),t,0));
    kron(eye(2),subs(mtr,t,T));];
B = eval(B);
c = [y_0;yd_0;ydd_0;y_g];

% SOLVE QP, "a_coeffs" and "b_coeffs" below will be graded!
[z,fval,exitflag] = quadprog(H,f,A,b,B,c);
exitflag
a_coeffs = z(1:N+1)
b_coeffs = z(N+2:end)
