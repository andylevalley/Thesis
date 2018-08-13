function [u_tot] = ParameterizedFitness_BVPrsm_rendezvous(dvar,obj_traj,omega,num_objects)
%MYFUNINT1 integration function for piecewise constant control

order = dvar(1:num_objects);
t_times = dvar(num_objects+1:end);
sat_state = [0 0 0 0 0 0];
u_tot = 0;
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
    
    [u] = polyExpression(x_init(1), x_init(2), x_target(1), x_target(2), t_times(n+1));

    sat_state(1,:) = x_target;
    
    u_tot = u_tot + abs(u);
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

 [u] = polyExpression(x_init(1), x_init(2), x_target(1), x_target(2), t_times(n+1));

u_tot = u_tot + abs(u);

end


