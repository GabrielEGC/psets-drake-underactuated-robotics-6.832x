% PROBLEM SETUP, DO NOT EDIT
% Construct vectors r1 and r2
r1=sym('r1',[2 1]);
sym(r1,'real');

r2=sym('r2',[2 1]);
sym(r2,'real');

n1=sym('n1',[2 1]);
sym(n1,'real');

t1=sym('t1',[2 1]);
sym(t1,'real');

n2=sym('n2',[2 1]);
sym(n2,'real');

t2=sym('t2',[2 1]);
sym(t2,'real');

mu = .8;
%%
% YOUR ANSWER HERE
% in terms of r1,n1,t1,r2,n2,t2, and mu
rn = {r1;r2};
tn = {t1;t2};
nn = {n1;n2};
n = 2;
c = [1;zeros(2*n,1)];

A=[zeros(3,1)];
for i=1:n
    A=[A [cross([rn{i};0],[tn{i};0])+[tn{i};0], cross([rn{i};0],[nn{i};0])+[nn{i};0]]];
end

b = zeros(3,1);
C = [-ones(2*n,1) [kron(eye(n),[1 -mu]);kron(eye(n),[-1 -mu])]];
C = [C; [-1, zeros(1,2*n)]]
d = [zeros(2*n,1);10];
%%
c
A
b
C
d