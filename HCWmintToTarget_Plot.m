%------------------------------%
% Extract Solution from Output %
%------------------------------%
close all;
solution = output.result.solution;
time = solution.phase(1).time/60; % in MINUTES

w = auxdata.w;

x     = solution.phase(1).state(:,1)./1000; % scaled to km
y     = solution.phase(1).state(:,2)./1000; % scaled to km

xdot  = solution.phase(1).state(:,3); % m/s
ydot  = solution.phase(1).state(:,4); % m/s


u1  = solution.phase(1).control(:,1); % xhat direction of thrust
u2  = solution.phase(1).control(:,2); % yhat direction of thrust

state = [x, y, xdot, ydot, ]; 
control = [u1, u2];


%---------------%
% Plot Solution %
%---------------%
figure(3)
pp = plot(time,state,'-o');
xl = xlabel('$t$ (min)','Interpreter','LaTeX');
% yl = ylabel('$(r(t),\theta(t),v_r(t),v_\theta(t))$','Interpreter','LaTeX');
yl = ylabel('$x,y,\dot{x},\dot{y}$ in $km$ and $\frac{m}{s}$','Interpreter','LaTeX');
% ll = legend('$r(t)$','$\theta(t)$','$v_r(t)$','$v_\theta(t)$','Location','Northwest');
ll = legend('$x$','$y$','$\dot{x}$','$\dot{y}$','Location','Northwest');
tt = title('States vs. Time (GPOPS)');
set(tt,'FontSize',18);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
set(ll,'FontSize',18,'Interpreter','LaTeX');
set(gca,'FontSize',16,'FontName','Times');
set(pp,'LineWidth',1.25);
grid on
print -depsc2 HCWmintToTarget_State.eps;
print -dpng HCWmintToTarget_State.png;

figure(4)
pp = plot(time,control,'-o');
xl = xlabel('$t$ (min)','Interpreter','LaTeX');
yl = ylabel('Control Directions $u_1,u_2$','Interpreter','LaTeX');
tt = title('Control in $\hat{x}, \hat{y}$ Directions (GPOPS)','Interpreter','LaTeX');
ll = legend('$u_1$','$u_2$','Norm');
set(ll,'FontSize',18,'Interpreter','LaTeX');
set(tt,'FontSize',18);
set(xl,'FontSize',18);
set(yl,'FontSize',18);
set(gca,'FontSize',16,'FontName','Times');
set(gca,'FontSize',16);
set(pp,'LineWidth',1.25);
grid on
print -depsc2 HCWmintToTarget_Control.eps;
print -dpng HCWmintToTarget_Control.png;


