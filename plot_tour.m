clc
omega = 7.291e-5; % mean motion (rad/sec)

%% Initial State cases

% inital starting state
order = dvar(1:num_objects);
t_times = dvar(num_objects+1:end);
sat_state = [0 0 0 0 0 0];
u_tot = 0;
u_RSM_tot = 0;
DeltaV_tot = 0;
n = 1;
clock = 0;
tt = [];

for i = 1:length(order)

    % drift sat for first time index at current state
    [x_drift,v_drift] = CWHPropagator(sat_state(1:3)',sat_state(4:6)',omega,0:t_times(n));
    sat_state = [x_drift(1:3,end)',v_drift(1:3,end)'];

    sat_traj{n} = [x_drift(1:2,:);v_drift(1:2,:)];

    % target obj we are travling to already drifted
    x_target = obj_traj{order(i)}(1:6,floor(clock+t_times(n+1)+t_times(n)))';
    % current location of the sat, not drifted
    x_init = sat_state(1,:);
    
    [u_RSM] = polyExpression(x_init(1), x_init(2), x_target(1), x_target(2), t_times(n+1));
    output = RendezvousGPOPS(x_init, x_target, t_times(n+1));
     
    solution = output.result.solution;
    
    time = solution.phase(1).time; % in MINUTES

    x     = solution.phase(1).state(:,1); % scaled to km
    y     = solution.phase(1).state(:,2); % scaled to km

    xdot  = solution.phase(1).state(:,3); % m/s
    ydot  = solution.phase(1).state(:,4); % m/s


    u1  = solution.phase(1).control(:,1); % xhat direction of thrust
    u2  = solution.phase(1).control(:,2); % yhat direction of thrust

    state = [x, y, xdot, ydot, ]; 
    control = [u1, u2];

    sat_state(1,:) = [x(end) y(end) 0 xdot(end) ydot(end) 0];
    sat_traj{n+1} = [x(:) y(:) xdot(:) ydot(:) time(:)]';
    sat_u{n} = [zeros(2,floor(t_times(n)))];
    sat_u{n+1} = [u1' u2'];
    
    
    uu = solution.phase(1).integral;
    u_tot = uu + u_tot;
    u_RSM_tot = u_RSM_tot+abs(u_RSM);

    clock = clock + t_times(n) + t_times(n+1);
    n = n + 2;

end

% drift sat for first time index at current state
[x_drift,v_drift] = CWHPropagator(sat_state(1:3)',sat_state(4:6)',omega,0:t_times(n));
sat_state = [x_drift(1:3,end)',v_drift(1:3,end)'];
sat_traj{n} = [x_drift(1:2,:);v_drift(1:2,:)];

% target obj we are travling to already drifted
x_target = [0 0 0 0 0 0];
% current location of the sat, not drifted
x_init = sat_state(1,:);

[u_RSM] = polyExpression(x_init(1), x_init(2), x_target(1), x_target(2), t_times(n+1));
output = RendezvousGPOPS(x_init, x_target, t_times(n+1));

solution = output.result.solution;

time = solution.phase(1).time; % in MINUTES

x     = solution.phase(1).state(:,1); % scaled to km
y     = solution.phase(1).state(:,2); % scaled to km

xdot  = solution.phase(1).state(:,3); % m/s
ydot  = solution.phase(1).state(:,4); % m/s


u1  = solution.phase(1).control(:,1); % xhat direction of thrust
u2  = solution.phase(1).control(:,2); % yhat direction of thrust

state = [x, y, xdot, ydot, ]; 
control = [u1, u2];

sat_state(1,:) = [x(end) y(end) 0 xdot(end) ydot(end) 0];
sat_traj{n+1} = [x(:) y(:) xdot(:) ydot(:) time(:)]';
sat_u{n} = [zeros(2,floor(t_times(n)))];
sat_u{n+1} = [u1' u2'];


uu = solution.phase(1).integral;
u_tot = uu + u_tot;
u_RSM_tot = u_RSM_tot+abs(u_RSM);

clock = clock + t_times(n) + t_times(n+1);

%% Static Plot

% for i = 1:5
%     plot(obj_traj{i}(2,:),obj_traj{i}(1,:),'Color','blue');
%     hold on
% end
% 
% for i = 1:length(t_times)
% %     h = quiver(sat_traj{i}(2,:),sat_traj{i}(1,:),sat_traj{i}(4,:),sat_traj{i}(3,:),0.5,'Color','k');
% %     h.Head.LineStyle = 'solid';
% %     hold on
%     plot(sat_traj{i}(2,:),sat_traj{i}(1,:),'Color','r');
%     hold on
% end
% 
% 
% 
% xlabel('y (in-track) km')
% ylabel('x (radial) km')
% title('6 Object Rendezvous')

%% Animated Plot
% 
n = 1;
plot_traj = [];

for i = 2:2:10
        x = interp1(sat_traj{i}(5,:),sat_traj{i}(1,:),1:1:t_times(i));
        y = interp1(sat_traj{i}(5,:),sat_traj{i}(2,:),1:1:t_times(i));
        xdot = interp1(sat_traj{i}(5,:),sat_traj{i}(3,:),1:1:t_times(i));
        ydot = interp1(sat_traj{i}(5,:),sat_traj{i}(4,:),1:1:t_times(i));
        
        plot_traj = horzcat(plot_traj,horzcat([sat_traj{i-1}(1,:);sat_traj{i-1}(2,:);sat_traj{i-1}(3,:);sat_traj{i-1}(4,:)],[x;y;xdot;ydot]));
        
        n = n+1;
end
        

vidObj1 = VideoWriter('Plot_Trajectory2.mp4','MPEG-4');
vidObj1.FrameRate = 50;  % Default 30
vidObj1.Quality = 100;    % Default 75
open(vidObj1);

for i = 1:100:length(plot_traj)
    figure(1);
        for k = 1:num_objects
            scatter(obj_traj{k}(2,i),obj_traj{k}(1,i),'MarkerFaceColor','b','MarkerEdgeColor','k');
            hold on
        end
        
    scatter(plot_traj(2,i),plot_traj(1,i),'MarkerFaceColor','r','MarkerEdgeColor','k');
    hold on
    % plotv([SunVec_LVLH(2,i),SunVec_LVLH(1,i)]')
    q = quiver(0,0,SunVec_LVLH(2,i),SunVec_LVLH(1,i),2);
    q.Color = 'r';
        
    

    hold off;
    grid on;
    xl = xlabel('y (km)');
    yl = ylabel('x (km)');
    tt = title('4 Waypoint Visit with Sunlight Constraint');
    axis([-75 75 -75 75])
    set(gcf,'color','w')
    
    drawnow
    Frame = getframe(gcf);
    writeVideo(vidObj1,Frame);
end
        
