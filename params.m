%% Physics Constants
g       = 9.81;        % Gravity (m/s^2)
rho_air = 1.21;        % Air density (kg/m^3)

%% Mass Properties
m  = 35.09 * 0.45359237;   % Mass (kg)
W  = m * g;                % Weight (N)

% Thrust Requirements
TWR = 2;                                
n_motors = 4;
T_total_max = TWR * W;                  
T_motor_max = T_total_max / n_motors;  

%% Moments of Inertia (kg*m^2)

Ix = 802.36   * 0.00029263965;
Iy = 3525.65  * 0.00029263965;
Iz = 4089.23  * 0.00029263965;

%% Inertial Tensor 
Ixx = 15634.61 * 0.00029263965;
Iyy = 16129.59 * 0.00029263965;
Izz = 9980.22  * 0.00029263965;
Ixy = 4587.10  * 0.00029263965;
Ixz = -6031.16 * 0.00029263965;
Iyz = -5963.06 * 0.00029263965;

I_tensor = [ Ixx  -Ixy  -Ixz;
             -Ixy  Iyy  -Iyz;
             -Ixz -Iyz   Izz ];

%% Arm Length
L = 0.254;   % Arm length (m)

%% Motor Directions
motor_dir = [ +1, -1, +1, -1 ];   % +1 = CCW, -1 = CW

%% Initial Orientation

phi0   = 0;     % initial roll
theta0 = 0;     % initial pitch
psi0   = 0;     % initial yaw

%% Initial Euler Angle Rates
p0 = 0;   % initial roll rate
q0 = 0;   % initial pitch rate
r0 = 0;   % initial yaw rate

