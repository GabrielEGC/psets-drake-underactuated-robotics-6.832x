% Initialize SOS program
checkDependency('spotless');
prog = spotsosprog;

ok_mosek = checkDependency('mosek');
ok_sedumi = checkDependency('sedumi');

if ~ok_sedumi && ~ok_mosek
        error('You need either MOSEK or SeDuMi installed to use this function.');
end   

% Choose sdp solver
if ok_mosek
    solver = @spot_mosek;
    solver_name = 'mosek';
else
    solver = @spot_sedumi;
    solver_name = 'sedumi';
end

% State (x = [x1;x2])
x1 = msspoly('x');
x2 = msspoly('y');
x = [x1;x2];
prog = prog.withIndeterminate(x);

% Dynamics
f = [x2; -x1 + x1^3 - x2];

% Initialize barrier function as a free quartic polynomial
d = 6;
[prog,B] = prog.newFreePoly(monomials(x,0:d));

% Compute time derivative of B
Bdot = diff(B,x)*f;


%%FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description of initial condition set (copy and paste your answer from
% part (a) of the problem)
g_X0 = 2.1^2 - (x1 - 1.5)^2 - x2^2; % Should be in terms of x1 and x2

% Description of unsafe set (copy and paste your answer from
% part (b) of the problem)
g_Xu = 0.4^2 - (x1+1)^2 - (x2+1)^2; % Should be in terms of x1 and x2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% SOS constraints
% C1: B(x) < 0 inside initial condition set
[prog,L1] = prog.newSOSPoly(monomials(x,0:d-2)); % Quadratic SOS multiplier polynomial
prog = prog.withSOS(-B - L1*g_X0 - 1e-4); % The 1e-4 is there to make sure the ...
                                           %inequality is strict (but you don't have to...
                                           %worry about this for the other constraints)
                                           
                                           
% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% C2: B(x) >= 0 inside unsafe set
[prog,L2] = prog.newSOSPoly(monomials(x,0:d-2)); % Quadratic SOS multiplier polynomial
prog = prog.withSOS(B - L2*g_Xu);
% C3: Bdot <= 0 everywhere
prog = prog.withSOS(-Bdot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% DO NOT MODIFY THIS BLOCK OF CODE
% Solve SOS program
options = spot_sdp_default_options();
options.verbose = 1;
sol = prog.minimize(0,solver,options);

% Check if SOS program ran correctly
if ~sol.isPrimalFeasible
    error('The SOS problem is not feasible');
end

% One more check for SeDuMi
if strcmp(solver_name,'sedumi')
    if sol.info.solverInfo.feasratio < 0
        error('The SOS problem is not feasible');
    end
end
        

% Print out B after zeroing out very small coefficient
xprint = msspoly('x',2); % print in terms of x1 and x2
disp(' ');
disp('Barrier function:');
B_sol = clean(subs(sol.eval(B),x,xprint),1e-5)

% Plot stuff
figure
[X,Y] = meshgrid(linspace(-3,3,100),linspace(-3,3,100));
% initial condition set
gs_X0 = msubs(g_X0,x,[X(:),Y(:)]');
contour(X,Y,reshape(double(gs_X0),100,100),[0 0],'LineWidth',3); % initial condition set
hold on
% unsafe set
gs_Xu = msubs(g_Xu,x,[X(:),Y(:)]');
contour(X,Y,reshape(double(gs_Xu),100,100),[0 0],'r','LineWidth',3) % unsafe set
% 0-level set of B
gs_B = msubs(sol.eval(B),x,[X(:),Y(:)]');
contour(X,Y,reshape(double(gs_B),100,100),[0 0],'b','LineWidth',3) % unsafe set
%contour(X,Y,reshape(double(gs_B+0.7),100,100),[0 0],'r','LineWidth',3) % unsafe set
%contour(X,Y,reshape(double(gs_B+0.5),100,100),[0 0],'g','LineWidth',3) % unsafe set

% (scaled) vector field
[X,Y] = meshgrid(linspace(-3,3,50),linspace(-3,3,50));
x1dots = reshape(double(msubs(f(1),x,[X(:),Y(:)]')),50,50);
x2dots = reshape(double(msubs(f(2),x,[X(:),Y(:)]')),50,50);
x1dots = 0.1*x1dots./(sqrt(x1dots.^2 + x2dots.^2));
x2dots = 0.1*x2dots./(sqrt(x1dots.^2 + x2dots.^2));
quiver(X,Y,x1dots,x2dots,'AutoScale','off','Color','k');

% Title
title('Barrier functions')
xlabel('x_1');
ylabel('x_2');

legend('X_0','X_u','0 level-set of B(x)');






