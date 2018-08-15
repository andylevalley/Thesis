clc
clear all

omega = 7.291e-5;
% 
num = 4;
ae = [20,20,20,20];
xd = [0,0,0,0];
yd = [0,0,0,0];
beta = [0,pi/2,pi,-pi/2];

for n = 1:num
    [R,V] = roe2hcw(omega,ae(n),xd(n),yd(n),beta(n));
    obj_state(n,1:6) = [R,V];
end

num = 4;
n = 1;


obj_state =[30 0 0 0 0 0;
            -30 0 0 0 0 0 ;
            0 30 0 0 0 0 ;
            0 -30 0 0 0 0];
                

        
        

for i = 1:num
    [x_drift,v_drift] = CWHPropagator(obj_state(i,1:3)',obj_state(i,4:6)',omega,1:1:60*60*24*10);
    obj_traj{n} = [x_drift(1:3,:);v_drift(1:3,:)];
    n = n+1;
end


for i = 1:num
    for n = 1:60*60*24*5
    obj_traj{i}(:,n) = obj_state(i,:);
    end
end


%% creat ncm fixed point trajectories

% for i = 1:4
%     for n = 1:60*60*24*2
%         obj_traj_nmc{i}(:,n) = obj_traj{1}(:,i*60*60*6);
%     end
% end
% 
% save('obj_traj.mat','obj_traj_nmc')