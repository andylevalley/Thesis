function phaseout = HCWmintToTarget_Continuous(input)

a0 = input.auxdata.a0;
c = input.auxdata.c;
w = input.auxdata.w;

t      = input.phase.time;

x     = input.phase.state(:,1);
y     = input.phase.state(:,2);

xdot  = input.phase.state(:,3);
ydot  = input.phase.state(:,4);

ux = input.phase.control(:,1);
uy = input.phase.control(:,2);



% Now perform calculations with inputs 
% a       = T./(m0 - dm.*t); % acceleration
a = a0./(1-t.*(a0./c));

% HERE'S THE DYNAMICS %
dx = xdot;
dy = ydot;

dxdot = 2*w*ydot+3*w^2*x+a.*ux;
dydot = -2*w*xdot + a.*uy;


phaseout.dynamics  = [dx,dy,dxdot,dydot];
phaseout.integrand = 0.5*(ux.^2+uy.^2);

