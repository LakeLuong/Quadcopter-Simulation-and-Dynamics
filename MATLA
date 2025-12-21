%% Physics Constants
g       = 9.81;        % Gravity (m/s^2)
rho_air = 1.21;        % Air density (kg/m^3)

%% Mass Properties
m  = 0.75;   % Mass (kg)
W  = m * g;                % Weight (N)

% Thrust Requirements
TWR = 4;                                
n_motors = 4;
T_total_max = TWR * W;                  
T_motor_max = T_total_max / n_motors;  

%% Moments of Inertia (kg*m^2)

Ix = 0.006;  
Iy = 0.006;  
Iz = 0.010;

%% Inertial Tensor 
Ixx = Ix;
Iyy = Iy;
Izz = Iz;
Ixy = 0;
Ixz = 0;
Iyz = 0;

I_tensor = [ Ixx  -Ixy  -Ixz;
             -Ixy  Iyy  -Iyz;
             -Ixz -Iyz   Izz ];

%% Arm Length
L = 0.12;   % Arm length (m)

%% Motor Direction
motor_dir = [ +1, -1, +1, -1 ];   % +1 = CCW, -1 = CW

%% Motor Mixing Matrix

k_tau = 0.01;
Lxy = L / sqrt(2);

M = [ 1      1       1       1;          % total thrust
      0    -Lxy    0    Lxy;        % roll   (tau_x)
     Lxy    0    -Lxy    0;        % pitch  (tau_y)
      k_tau -k_tau   k_tau  -k_tau ];    % yaw    (tau_z)


%% Initial Orientation

phi0   = 0.785398;     % initial roll
theta0 = 0.7853985;     % initial pitch
psi0   = 0.785398;     % initial yaw

%% Initial Euler Angle Rates
p0 = 0;   % initial roll rate
q0 = 0;   % initial pitch rate
r0 = 0;   % initial yaw rate

