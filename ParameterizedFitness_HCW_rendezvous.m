function [DeltaV_tot] = ParameterizedFitness_HCW_rendezvous(dvar,obj_traj,omega,num_objects)
%MYFUNINT1 integration function for piecewise constant control


order = dvar(1:num_objects);
t_times = dvar(num_objects+1:end);
sat_state = [0 0 0 0 0 0];
DeltaV_tot = 0;
n = 1;
clock = 0;


for i = 1:length(order)
    
    
    % drift sat for first time index at current state
    [x_drift,v_drift] = CWHPropagator(sat_state(1:3)',sat_state(4:6)',omega,t_times(n));
    sat_state = [x_drift',v_drift'];
    
    % target obj we are travling to already drifted
    x_target = obj_traj{order(i)}(1:6,floor(clock+t_times(n+1)+t_times(n)))';
    % current location of the sat, not drifted
    x_init = sat_state(1,:);
    
    DeltaV = HCW_DeltaV(x_init(1:3)',x_target(1:3)',x_init(4:end)',x_target(4:end)',t_times(n+1),omega);
    
    DeltaV_tot = DeltaV_tot + norm(DeltaV);

    sat_state(1,:) = x_target;
    
    clock = clock + t_times(n) + t_times(n+1);
    n = n + 2;
    
end

% drift sat for first time index at current state
[x_drift,v_drift] = CWHPropagator(sat_state(1:3)',sat_state(4:6)',omega,t_times(n));
sat_state = [x_drift',v_drift'];

% target obj we are travling to already drifted
x_target = [0 0 0 0 0 0];
% current location of the sat, not drifted
x_init = sat_state(1,:);

DeltaV = HCW_DeltaV(x_init(1:3)',x_target(1:3)',x_init(4:end)',x_target(4:end)',t_times(n+1),omega);

DeltaV_tot = DeltaV_tot + norm(DeltaV);


