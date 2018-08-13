syms omega t

STM = [4-3*cos(omega*t) 0 0 sin(omega*t)/omega -2*(cos(omega*t)-1)/omega 0;
       6*sin(omega*t)-6*omega*t 1 0 2*(cos(omega*t)-1)/omega (4*sin(omega*t)-3*omega*t)/omega 0;
       0 0 cos(omega*t) 0 0 sin(omega*t)/omega;
       3*omega*sin(omega*t) 0 0 cos(omega*t) 2*sin(omega*t) 0;
       6*omega*cos(omega*t)-6*omega 0 0 -2*sin(omega*t) 4*cos(omega*t)-3 0;
       0 0 -omega*sin(omega*t) 0 0 cos(omega*t)]


%% Phi matricies 

phi_11 = [4-3*cos(omega*t) 0 0;
          6*sin(omega*t)-6*omega*t 1 0;
          0 0 cos(omega*t)];
      
phi_12 = [sin(omega*t)/omega -(2*cos(omega*t)-2)/omega 0;
       (2*cos(omega*t)-2)/omega (4*sin(omega*t)-3*omega*t)/omega 0;
       0 0 sin(omega*t)/omega];
   
phi_21 = [3*omega*sin(omega*t) 0 0;
          6*omega*cos(omega*t)-6*omega 0 0;
          0 0 -omega*sin(omega*t)];
      
phi_22 = [cos(omega*t) 2*sin(omega*t) 0;
          -2*sin(omega*t) 4*cos(omega*t)-3 0;
          0 0 cos(omega*t)];

phi_11_inv = [-1/(3*cos(omega*t) - 4), 0, 0;
            (6*(sin(omega*t) - omega*t))/(3*cos(omega*t) - 4), 1, 0;
            0, 0, 1/cos(omega*t)];

phi_12_inv = [ (omega*(4*sin(omega*t) - 3*omega*t))/(4*cos(omega*t)^2 - 8*cos(omega*t) + 4*sin(omega*t)^2 - 3*omega*t*sin(omega*t) + 4), (2*omega*(cos(omega*t) - 1))/(4*cos(omega*t)^2 - 8*cos(omega*t) + 4*sin(omega*t)^2 - 3*omega*t*sin(omega*t) + 4), 0;
 -(2*omega*(cos(omega*t) - 1))/(4*cos(omega*t)^2 - 8*cos(omega*t) + 4*sin(omega*t)^2 - 3*omega*t*sin(omega*t) + 4), (omega*sin(omega*t))/(4*cos(omega*t)^2 - 8*cos(omega*t) + 4*sin(omega*t)^2 - 3*omega*t*sin(omega*t) + 4), 0;
0, 0, omega/sin(omega*t)];
 
