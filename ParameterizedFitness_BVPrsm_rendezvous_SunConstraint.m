function [u_tot] = ParameterizedFitness_BVPrsm_rendezvous_SunConstraint(dvar,obj_traj,omega,num_objects,SunVector)
%MYFUNINT1 integration function for piecewise constant control


order = dvar(1:num_objects);
t_times = dvar(num_objects+1:end);
sat_state = [0 0 0 0 0 0];
u_tot = 0;
n = 1;
theta_mean = 0;
clock = 0;
p = 3;

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
    
    % Sun penalty calc
    N = floor(t_times(p));
    cl = floor(sum(t_times(1:p-1)));

    x_Sun = SunVector(1,cl:1:floor(cl+N))';
    y_Sun = SunVector(1,cl:1:floor(cl+N))';
    z_Sun = SunVector(1,cl:1:floor(cl+N))';

    state_Sun(:,1:3) = [x_Sun, y_Sun, z_Sun];
    sat_Sun = ones(length(state_Sun),3).*sat_state(1:3);

    theta = acos(sum(sat_Sun.*state_Sun,2)./(sqrt(sum(sat_Sun.^2,2)).*sqrt(sum(state_Sun.^2,2))));
    theta_mean = theta_mean + mean(theta);
    

    state_Sun = [];

    u_tot = u_tot + abs(u);
    clock = clock + t_times(n) + t_times(n+1);
    n = n + 2;
    p = p + 2;

    
end

% drift sat for first time index at current state
[x_drift,v_drift] = CWHPropagator(sat_state(1:3)',sat_state(4:6)',omega,t_times(n));
sat_state = [x_drift',v_drift'];

% target obj we are travling to already drifted
x_target = [0 0 0 0 0 0];
% current location of the sat, not drifted
x_init = sat_state(1,:);

[u] = polyExpression(x_init(1), x_init(2), x_target(1), x_target(2), t_times(n+1));


reward = (theta_mean/5-30*pi/180);

u_tot = u_tot + abs(u) + reward;



end


