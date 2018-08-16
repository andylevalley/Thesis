function [c ceq] = constraint_Sun(dvar,t_total,num_objects,SunVec,marker_state)

order = dvar(1:num_objects);
test = 1:1:num_objects;
A = sum(order(:) == test,1);
B = ones(1,num_objects);
t_times = dvar(num_objects+1:end);


n = 3;
g = 1;

for i = 1:length(order)
    
    N = floor(t_times(n));
    cl = floor(sum(t_times(1:n-1)));
    
    % target obj we are travling to already drifted
    sat_state = marker_state{order(i)}(:,cl:1:cl+N)';

    x_Sun = SunVec(1,cl:1:cl+N)';
    y_Sun = SunVec(2,cl:1:cl+N)';
    z_Sun = SunVec(3,cl:1:cl+N)';

    state_Sun(:,1:3) = [x_Sun(1:100:end), y_Sun(1:100:end), z_Sun(1:100:end)];
    sat_Sun = sat_state(1:100:end,1:3);
    
    
   
    theta = acos(sum(sat_Sun.*state_Sun,2)./(sqrt(sum(sat_Sun.^2,2)).*sqrt(sum(state_Sun.^2,2))));
    mean_theta(g) = mean(theta);
    g = g+1;
    
    state_Sun = [];
    theta = [];

    n = n+2;
    
    
end
    


% c = [sum(dvar(num_objects+1:end))- (t_total); % change t_total and num_objects 
%     -(sum(order(:)==1)*sum(order(:)==2)*sum(order(:)==3)*sum(order(:)==4)*sum(order(:)==5))+1]; % this needs to change depending on number of objects 
% ceq = [];

% c = [sum(dvar(num_objects+1:end))- (t_total);
%     -(isequal(A,B))+1;
%     mean_theta(1) - 90*pi/180;
%     mean_theta(2) - 180*pi/180;
%     mean_theta(3) - 180*pi/180;
%     mean_theta(4) - 180*pi/180];
c = [sum(dvar(num_objects+1:end))- (t_total);
    -(isequal(A,B))+1;
    mean_theta(1) + 15*pi/180;
    mean_theta(2) + 15*pi/180];
ceq = [];
end