% Version 1.0 % 
clc
clear all

%% Tunable Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Scenario Parameters
t_total = 60*60*24*3;
num_objects = 2;
omega = 7.291e-5; % mean motion (rad/sec)
sat_state = [0 0 0 0 0 0]; % starting state of active sat
load('obj_traj_NMC');
load('SunVec_LVLH');

% Optimization Parameters
lb_init = 0;
lb_loiter = 60*60*24;
lb_transfer = 60*30;

ub_init = 60*60;
ub_loiter = t_total/3;
ub_transfer = t_total/3;

Popsize = 100;
EliteCount = ceil(0.1*Popsize);
MaxGenerations = 500;

UseParallel = 0;

%% Derived Parameters (DO NOT EDIT) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
numvars = num_objects + num_objects*2+2;

for i = 1:Popsize
    IntPop(i,:) = randperm(num_objects);
end

% IntPop = randi([1,4],Popsize,num_objects);
IntPop = [IntPop, 1+(60*60-1).*rand(Popsize,num_objects*2+2)]; 

IntCon = [1:1:num_objects];

lb = [repmat(1,1,num_objects), lb_init, lb_transfer, repmat([lb_loiter,lb_transfer],1,(numvars-num_objects)/2-1)];
ub = [repmat(num_objects,1,num_objects), ub_init, ub_transfer, repmat([ub_loiter,ub_transfer],1,(numvars-num_objects)/2-1)];

%% GA optimization (DO NOT EDIT) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Optimization Options, edited above

opts = optimoptions('ga','CrossoverFraction',0.9,'PopulationSize',Popsize,...
    'EliteCount',EliteCount,...
    'MaxGenerations',MaxGenerations,'PlotFcn',@gaplotbestf,'UseParallel',false);

% Fitness and Penalty Function definition    
FitnessFunction = @(dvar) ParameterizedFitness_HCW_rendezvous(dvar,obj_traj,omega,num_objects);
ConstraintFunction = @(dvar) constraint_Sun(dvar,t_total,num_objects,SunVec_LVLH,obj_traj);

% GA optimization
[dvar,fval,exitflag,output] = ga(FitnessFunction,numvars,[],[],[],[],...
    lb,ub,ConstraintFunction,IntCon,opts); 

