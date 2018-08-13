function output = RendezvousGPOPS(x_init, x_target, tt)


%--------------------------------------------------------------------------%
%---------------- Set Up Auxiliary Data for Problem -----------------------%
%--------------------------------------------------------------------------%
% auxdata.T  = 0.1405; % thrust
auxdata.a0 = .0686; % N/kg - initial acceleration
% auxdata.m0 = 1; % initial mass
auxdata.c = 3000; % m/s
% auxdata.dm = 0.0749; % mass flow rate
auxdata.w = sqrt(398600.5/(6378.137+35786)^3);
% auxdata.mu = 1; % gravitational parameter (these are all normalized)

%--------------------------------------------------------------------------%
%--------------- Set Up Bounds on State, Control, and Time ----------------%
%--------------------------------------------------------------------------%

% Time
t0 = 0;
tfmin = tt; % 1 min (in seconds)
tfmax = tt; % safety bound ?? 3 hours?  working in SECONDS

% Fixed initial conditions of states
x0 = x_init(1); % m % these distances need to be in bounds to make HCW applicable
y0 = x_init(2); % m

xdot0 = x_init(4); % m/s
ydot0 = x_init(5); % m/s


% Fixed final condition of state (Rendezvous)
xf = x_target(1);
yf = x_target(2);

xdotf = x_target(4);
ydotf = x_target(5);

% Hard and/or safety bounds
xmin = -200; % these should be based off of natural circumnavigation results
xmax = 200; % these are all in (m)
ymin = -200;
ymax = 200;
xdotmin = -1; % m/s
xdotmax = 1; % m/s
ydotmin = -1; % m/s
ydotmax = 1; % m/s

% Controls
uxmin = -1; % should never go above 1, right?
uxmax = 1;
uymin = -1;
uymax = 1;

% Put pre-defined values into GPOPS bounds structure
bounds.phase.initialtime.lower = t0; % fixed
bounds.phase.initialtime.upper = t0; % fixed
bounds.phase.finaltime.lower = tfmin; % lower bound
bounds.phase.finaltime.upper = tfmax; % upper bound

bounds.phase.initialstate.lower = [x0,y0,xdot0,ydot0]; % fixed
bounds.phase.initialstate.upper = [x0,y0,xdot0,ydot0]; % fixed
bounds.phase.state.lower = [xmin,ymin,xdotmin,ydotmin]; % lower bound
bounds.phase.state.upper = [xmax,ymax,xdotmax,ydotmax]; % upper bound
bounds.phase.finalstate.lower = [xf,yf,xdotf,ydotf]; % fixed position and velocity
bounds.phase.finalstate.upper = [xf,yf,xdotf,ydotf]; % fixed position and velocity

bounds.phase.control.lower = [uxmin,uymin]; % lower bound
bounds.phase.control.upper = [uxmax,uymax]; % upper bound

bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 1000;

%--------------------------------------------------------------------------%
%------------------------- Set Up Initial Guess ---------------------------%
%--------------------------------------------------------------------------%
tGuess = [t0; tfmax]; % fixed initial time, guessing 2 hour (in seconds) for final time

xGuess = [x0;xf]; % initial condition and terminal constraint (guess doesn't HAVE to be this though - but recommended?)
yGuess = [y0;yf];

xdotGuess = [xdot0;xdotf];
ydotGuess = [ydot0;ydotf];

uxGuess = [0; 0];
uyGuess = [0; 0];

guess.phase.time    = [tGuess];

guess.phase.state   = [xGuess,yGuess,xdotGuess,ydotGuess];

guess.phase.control = [uxGuess,uyGuess];

guess.phase.integral = 1;


%--------------------------------------------------------------------------%
%------------------------- Set Up Initial Mesh ----------------------------%
%--------------------------------------------------------------------------%
N = 10;
meshphase.colpoints = 4*ones(1,N);
meshphase.fraction   = ones(1,N)/N;

%--------------------------------------------------------------------------%
%-------------------------- Set Up for Solver -----------------------------%
%--------------------------------------------------------------------------%
setup.name = 'HCW-Min-Time-to-Target';
setup.functions.continuous = @HCWmintToTarget_Continuous; % the dynamics (and path constraints?)
setup.functions.endpoint = @HCWmintToTarget_Endpoint;

setup.displaylevel = 2; % ?
setup.nlp.solver = 'ipopt';
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;

setup.derivatives.supplier = 'sparseCD';
setup.derivatives.derivativelevel = 'second';

setup.mesh.method = 'hp-PattersonRao';
setup.mesh.tolerance = 1e-4;
setup.mesh.phase = meshphase;
setup.scales.method = 'automatic-hybrid';

%--------------------------------------------------------------------------%
%-------------------- Solve Problem and Extract Solution ------------------%
%--------------------------------------------------------------------------%
output = gpops2(setup);


end
