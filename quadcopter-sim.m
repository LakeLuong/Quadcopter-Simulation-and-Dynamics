%% Physics Constants
g       = 9.81;        % Gravity (m/s^2)
rho_air = 1.21;        % Air density (kg/m^3)

%% Mass Properties
m  = 3.0 * 0.45359237;   % Mass (kg)
W  = m * g;                % Weight (N)

% Thrust Requirements
TWR = 2;                                
n_motors = 4;
T_total_max = TWR * W;                  
T_motor_max = T_total_max / n_motors;  

%% Moments of Inertia (kg*m^2)

Ix = 0.05;
Iy = 0.05;
Iz = 0.05;

%% Inertial Tensor 
Ixx = 0.05;
Iyy = 0.05;
Izz = 0.05;
Ixy = 0.05;
Ixz = 0.05;
Iyz = 0.05;

I_tensor = [ Ixx  -Ixy  -Ixz;
             -Ixy  Iyy  -Iyz;
             -Ixz -Iyz   Izz ];

%% Arm Length
L = 0.254;   % Arm length (m)

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

