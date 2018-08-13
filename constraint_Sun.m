function [c ceq] = constraint_Sun(dvar,t_total,num_objects,SunVector)

order = dvar(1:num_objects);
test = 1:1:num_objects;
A = sum(order(:) == test,1);
B = ones(1,num_objects);
t_times = dvar(num_objects+1:end);


n = 3;
g = 1;

for i = 1:length(order)
    
    % target obj we are travling to already drifted
    obj_state =[30 0 0 0 0 0;
            -30 0 0 0 0 0 ;
            0 30 0 0 0 0 ;
            0 -30 0 0 0 0];
    sat_state(1,:) = obj_state(order(i),:);
    
    % Straight line between current and next visit location
    
    N = floor(t_times(n));
    cl = floor(sum(t_times(1:n-1)));
    x_Sun = SunVector(1,cl:1:cl+N)';
    y_Sun = SunVector(2,cl:1:cl+N)';
    z_Sun = SunVector(3,cl:1:cl+N)';

    state_Sun(:,1:3) = [x_Sun, y_Sun, z_Sun];
    sat_Sun = ones(length(state_Sun),3).*sat_state(1:3);
    
    
   
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

c = [sum(dvar(num_objects+1:end))- (t_total);
    -(isequal(A,B))+1;
    mean_theta(1)-15*pi/180;
    mean_theta(2)-15*pi/180;
    mean_theta(3)-15*pi/180;
    mean_theta(4)-15*pi/180];
ceq = [];
end