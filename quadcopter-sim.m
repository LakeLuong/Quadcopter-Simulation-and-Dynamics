clc; clear; close all;

%% ============================
% 1. Load Parameters
% ============================
params

%% ============================
% 2. Initial Conditions
% ============================
dt = 0.01; 
steps = 5000;

% Orientation (Euler angles)
phi   = phi0;
theta = theta0;
psi   = psi0;

% Angular rates
p = p0;
q = q0;
r = r0;

% Commanded Angles
phi_cmd = 0;
theta_cmd = 0;
psi_cmd = 0;

p_cmd = 0;
q_cmd = 0;
r_cmd = 0;

z_cmd  = 0;
vz_cmd = 0;

T_cmd = m*g; % Commanded/Desired Thrust Value

% Attitude Controller
x_cmd = 0;
y_cmd = 0;

vx_cmd = 0;
vy_cmd = 0;

% Translational states
x  = 0;   y  = 0;   z = 0;
vx = 0;   vy = 0;   vz = 0;

% PD Controllers
Kp_phi   = 1.2; Kd_phi   = 0.5;
Kp_theta = 1.2; Kd_theta = 0.5;
Kp_psi   = 0.2; Kd_psi   = 0.05;

% Altitude PD gains
Kp_z = 0.5;
Kd_z = 1;

% Position-to-angle PD gains
Kp_xy = 0.03;
Kd_xy = 0.05;

%% ============================
% 3. Preallocate History
% ============================
phi_hist   = zeros(1, steps);
theta_hist = zeros(1, steps);
psi_hist   = zeros(1, steps);

p_hist = zeros(1, steps);
q_hist = zeros(1, steps);
r_hist = zeros(1, steps);

x_hist = zeros(1, steps);
y_hist = zeros(1, steps);
z_hist = zeros(1, steps);

power_hist = zeros(1, steps);

%% NEW: Motor thrust histories
T1_hist = zeros(1, steps);
T2_hist = zeros(1, steps);
T3_hist = zeros(1, steps);
T4_hist = zeros(1, steps);

%% Initialize Motor Thrusts
T1 = T_cmd/4;
T2 = T_cmd/4;
T3 = T_cmd/4;
T4 = T_cmd/4;

%% ============================
% 4. Simulation Loop
% ============================
for step = 1:steps

    %% Outer Loop: Position → Angle Commands
    e_x  = x_cmd - x;
    e_y  = y_cmd - y;
    e_vx = vx_cmd - vx;
    e_vy = vy_cmd - vy;

    theta_cmd = -(Kp_xy*e_x + Kd_xy*e_vx);
    phi_cmd   =  (Kp_xy*e_y + Kd_xy*e_vy);

    max_tilt = deg2rad(6);
    phi_cmd   = max(min(phi_cmd,   max_tilt), -max_tilt);
    theta_cmd = max(min(theta_cmd, max_tilt), -max_tilt);

    %% Angle & Rate Errors
    e_phi   = phi_cmd   - phi;
    e_theta = theta_cmd - theta;
    e_psi   = psi_cmd   - psi;

    e_p = p_cmd - p;
    e_q = q_cmd - q;
    e_r = r_cmd - r;

    %% PD Controller → Desired Torques
    tau_phi   = Kp_phi   * e_phi   + Kd_phi   * e_p;
    tau_theta = Kp_theta * e_theta + Kd_theta * e_q;
    tau_psi   = Kp_psi   * e_psi   + Kd_psi   * e_r;

    %% Altitude Controller
    e_z  = z_cmd  - z;
    e_vz = vz_cmd - vz;
    T_cmd = m*(g + Kp_z*e_z + Kd_z*e_vz);

    %% Mixer
    u = [T_cmd; tau_phi; tau_theta; tau_psi];
    T_vec = M \ u;

    T1 = min(max(T_vec(1), 0), T_motor_max);
    T2 = min(max(T_vec(2), 0), T_motor_max);
    T3 = min(max(T_vec(3), 0), T_motor_max);
    T4 = min(max(T_vec(4), 0), T_motor_max);

    %% Store individual thrusts
    T1_hist(step) = T1;
    T2_hist(step) = T2;
    T3_hist(step) = T3;
    T4_hist(step) = T4;

    %% Total thrust
    T_total = T1 + T2 + T3 + T4;
    power_hist(step) = T_total;

    %% Torques from motors
    tau_x = -Lxy*T2 + Lxy*T4;
    tau_y =  Lxy*T1 - Lxy*T3;
    tau_z =  k_tau*T1 - k_tau*T2 + k_tau*T3 - k_tau*T4;

    %% Rotational Accelerations
    alpha_x = (tau_x - (Iy - Iz)*q*r) / Ix;
    alpha_y = (tau_y - (Iz - Ix)*p*r) / Iy;
    alpha_z = (tau_z - (Ix - Iy)*p*q) / Iz;

    %% Euler Angle Rates
    phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta); 
    theta_dot = q*cos(phi) - r*sin(phi);
    psi_dot   = (q*sin(phi)/cos(theta)) + (r*cos(phi)/cos(theta));

    %% Rotation Matrix
    R = [
        cos(theta)*cos(psi),  cos(theta)*sin(psi), -sin(theta);
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
        sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
        sin(phi)*cos(theta);
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), ...
        cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), ...
        cos(phi)*cos(theta)
    ];

    %% Drag
    c1 = 0.2;
    c2 = 0.05;

    F_world = R*[0;0;T_total] - [0;0;W];

    F_drag = [
        -c1*vx - c2*vx*abs(vx);
        -c1*vy - c2*vy*abs(vy);
        -c1*vz - c2*vz*abs(vz)
    ];

    F_total = F_world + F_drag;

    ax = F_total(1)/m;
    ay = F_total(2)/m;
    az = F_total(3)/m;

    %% Integrate States
    p = p + alpha_x * dt;
    q = q + alpha_y * dt;
    r = r + alpha_z * dt;

    phi   = phi   + phi_dot   * dt;
    theta = theta + theta_dot * dt;
    psi   = psi   + psi_dot   * dt;

    vx = vx + ax * dt;
    vy = vy + ay * dt;
    vz = vz + az * dt;

    x = x + vx * dt;
    y = y + vy * dt;
    z = z + vz * dt;

    %% Store for plots
    phi_hist(step)   = phi;
    theta_hist(step) = theta;
    psi_hist(step)   = psi;

    p_hist(step) = p;
    q_hist(step) = q;
    r_hist(step) = r;

    x_hist(step) = x;
    y_hist(step) = y;
    z_hist(step) = z;

end

%% ============================
% Plot Rotation Results
% ============================
figure("Name", "Rotation");
plot(phi_hist, 'LineWidth', 1.5); hold on;
plot(theta_hist, 'LineWidth', 1.5);
plot(psi_hist, 'LineWidth', 1.5);
legend('Roll (phi)', 'Pitch (theta)', 'Yaw (psi)');
xlabel('Time Step');
ylabel('Angle (rad)');
grid on;
title('Orientation Over Time');

%% ============================
% Plot Position Results
% ============================
figure("Name", "Position");
plot(x_hist, 'LineWidth', 1.5); hold on;
plot(y_hist, 'LineWidth', 1.5);
plot(z_hist, 'LineWidth', 1.5);
legend('x','y','z');
xlabel('Time Step');
ylabel('Position (m)');
grid on;
title('Position Over Time');

%% ============================
% Plot Total Thrust (Power)
% ============================
figure("Name","Total Thrust");
plot(power_hist,'LineWidth',1.5);
xlabel('Time Step');
ylabel('Total Thrust (N)');
grid on;
title('Total Thrust Over Time');

%% ============================
% Plot Individual Motor Thrusts
% ============================
figure("Name","Motor Thrusts");
hold on; grid on;
plot(T1_hist,'LineWidth',1.5);
plot(T2_hist,'LineWidth',1.5);
plot(T3_hist,'LineWidth',1.5);
plot(T4_hist,'LineWidth',1.5);

max(T2_hist)
min(T2_hist)
T_motor_max


yline(T_motor_max,'--r','Max Thrust');
yline(0,'--k');

xlabel('Time Step');
ylabel('Motor Thrust (N)');
legend('T1','T2','T3','T4');
title('Individual Motor Thrusts Over Time');
