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

% Translational states
x  = 0;   y  = 0;   z  = 0;
vx = 0;   vy = 0;   vz = 0;

% PD Controllers
Kp_phi   = 0.5; Kd_phi   = 0.1;
Kp_theta = 0.5; Kd_theta = 0.1;
Kp_psi   = 0.2; Kd_psi   = 0.05;

% Altitude PD gains
Kp_z = 0.5;
Kd_z = 1;

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

%% Initialize Motor Thrusts
T1 = T_cmd/4;
T2 = T_cmd/4;
T3 = T_cmd/4;
T4 = T_cmd/4;

%% ============================
% 4. Simulation Loop
% ============================
for step = 1:steps

    %% 4. Angle & Rate Errors
    e_phi   = phi_cmd   - phi;
    e_theta = theta_cmd - theta;
    e_psi   = psi_cmd   - psi;

    e_p = p_cmd - p;
    e_q = q_cmd - q;
    e_r = r_cmd - r;

    %% 5. PD Controller → Desired Torques
    tau_phi   = Kp_phi   * e_phi   + Kd_phi   * e_p;
    tau_theta = Kp_theta * e_theta + Kd_theta * e_q;
    tau_psi   = Kp_psi   * e_psi   + Kd_psi   * e_r; 
    
    % Altitude Controller
    e_z  = z_cmd  - z;
    e_vz = vz_cmd - vz;
    T_cmd = m*(g + Kp_z*e_z + Kd_z*e_vz);

    %% 6. Mixer → Compute Desired Motor Thrusts
    u = [T_cmd; tau_phi; tau_theta; tau_psi];
    T_vec = M \ u;

    T1 = T_vec(1);
    T2 = T_vec(2);
    T3 = T_vec(3);
    T4 = T_vec(4);

    % Clamp thrusts to physical limits
    T1 = min(max(T1, 0), T_motor_max);
    T2 = min(max(T2, 0), T_motor_max);
    T3 = min(max(T3, 0), T_motor_max);
    T4 = min(max(T4, 0), T_motor_max);

    %% 1. Compute torques from current motor thrusts
    % tau_x = roll, tau_y = pitch, tau_z = yaw
    tau_x = -Lxy*T2 + Lxy*T4;
    tau_y =  Lxy*T1 - Lxy*T3;
    tau_z =  k_tau*T1 - k_tau*T2 + k_tau*T3 - k_tau*T4;

    %% 2. Rotational Accelerations
    alpha_x = (tau_x - (Iy - Iz)*q*r) / Ix;
    alpha_y = (tau_y - (Iz - Ix)*p*r) / Iy;
    alpha_z = (tau_z - (Ix - Iy)*p*q) / Iz;

    %% 3. Euler Angle Rates (body → world)
    phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta); 
    theta_dot = q*cos(phi) - r*sin(phi);
    psi_dot   = (q*sin(phi)/cos(theta)) + (r*cos(phi)/cos(theta));

    %% 7. Compute Total Thrust
    T_total = T1 + T2 + T3 + T4;

    %% 8. Rotation Matrix
    R = [
        cos(theta)*cos(psi),  cos(theta)*sin(psi), -sin(theta);
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
        sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
        sin(phi)*cos(theta);
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), ...
        cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), ...
        cos(phi)*cos(theta)
    ];

%% --- Drag Coefficients ---
c1 = 0.2;    % linear drag coefficient
c2 = 0.05;   % quadratic drag coefficient

%% World Frame Forces
F_world = R*[0;0;T_total] - [0;0;W];

%% Compute blended drag in world frame (opposes velocity)
F_drag_x = -c1*vx - c2*vx*abs(vx);
F_drag_y = -c1*vy - c2*vy*abs(vy);
F_drag_z = -c1*vz - c2*vz*abs(vz);   % Veritcal drag

F_drag = [F_drag_x;
          F_drag_y;
          F_drag_z];

%% Total Forces
F_total = F_world + F_drag;

%% Accelerations
ax = F_total(1) / m;
ay = F_total(2) / m;
az = F_total(3) / m;


    %% 10. Integrate ALL states 
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

    %% 11. Store History
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
% 6. Plot Rotation Results
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
% 7. Plot Position Results
% ============================
figure("Name", "Position");
plot(x_hist, 'LineWidth', 1.5); hold on;
plot(y_hist, 'LineWidth', 1.5);
plot(z_hist, 'LineWidth', 1.5);
legend('x', 'y', 'z');
xlabel('Time Step');
ylabel('Position (m)');
grid on;
title('Position Over Time');
