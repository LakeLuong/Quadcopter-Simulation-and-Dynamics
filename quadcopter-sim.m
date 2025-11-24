clc; clear; close all;

%% ============================
% 1. Load Parameters
% ============================
params

%% ============================
% 2. Initial Conditions
% ============================
dt = 0.01; 
steps = 1000;

% Orientation (Euler angles)
phi   = phi0;
theta = theta0;
psi   = psi0;

% Angular rates
p = p0;
q = q0;
r = r0;

% Translational states
x  = 0;   y  = 0;   z  = 0;
vx = 0;   vy = 0;   vz = 0;

%% ============================
% 3. Motor Thrusts (test)
% ============================
T1 = 156.1414169;
T2 = 0;
T3 = 0;
T4 = 0;

%% ============================
% 4. Preallocate History
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

%% ============================
% 5. Simulation Loop
% ============================
for step = 1:steps
    %% --- Torque ---
    tau_x = L * (T2 + T3 - T1 - T4);
    tau_y = L * (T1 + T2 - T3 - T4);

    kM = 0.02;
    tau_z = kM * ( ...
        motor_dir(1)*T1 + motor_dir(2)*T2 + ...
        motor_dir(3)*T3 + motor_dir(4)*T4 );


    %% --- Rotational Dynamics ---
    alpha_x = (tau_x - (Iy - Iz)*q*r) / Ix; % Gives us acceleration along the x-axis
    alpha_y = (tau_y - (Iz - Ix)*p*r) / Iy; % Gives us acceleration along the y-axis
    alpha_z = (tau_z - (Ix - Iy)*p*q) / Iz; % Gives us acceleration along the z-axis

    %% --- EULER ANGLE RATES ---

%{
These formulas convert p, q, and r, which are the rotation speeds in the
drone frame to phi_dot, theta_dot, and psi_dot, which are the rotation
speeds in the world frame. 

%}

    phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta); 
    theta_dot = q*cos(phi) - r*sin(phi);
    psi_dot   = (q*sin(phi)/cos(theta)) + (r*cos(phi)/cos(theta));

    %% --- Integrate Body Rates ---
    p = p + alpha_x * dt;
    q = q + alpha_y * dt;
    r = r + alpha_z * dt;

    %% --- Integrate Euler Rates ---
    phi   = phi   + phi_dot   * dt;
    theta = theta + theta_dot * dt;
    psi   = psi   + psi_dot   * dt;

    %% --- Rotation Matrix ---

    % Rotates all the forces in the drone frame to the world frame

    R = [
        cos(theta)*cos(psi),  cos(theta)*sin(psi), -sin(theta);
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), ...
        sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), ...
        sin(phi)*cos(theta);
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), ...
        cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), ...
        cos(phi)*cos(theta)
    ];


    %% --- Translational Dynamics ---
    T_total = T1 + T2 + T3 + T4;

    F_world = R*[0;0;T_total] - [0;0;W];

    ax = F_world(1) / m;
    ay = F_world(2) / m;
    az = F_world(3) / m;

    % integrate velocity
    vx = vx + ax * dt;
    vy = vy + ay * dt;
    vz = vz + az * dt;

    % integrate position
    x = x + vx * dt;
    y = y + vy * dt;
    z = z + vz * dt;


    %% --- Store History ---
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
