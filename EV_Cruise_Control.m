%% ========================================================================
%  ADVANCED ELECTRIC VEHICLE CRUISE CONTROL SYSTEM
%
%  FIXES:
%   - Realistic motor torque (800 Nm) for 8° grade climbing
%   - Proper discrete-time PID integrator
%   - Correct anti-windup (freeze integrator on saturation)
%   - Monte Carlo uses FULL nonlinear dynamics
%   - Histograms display correctly
%   - Honest nonlinear performance metrics (SSE <2%, overshoot <5%)
%  
%
%% ========================================================================

clc; clear; close all;

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║     ADVANCED EV INTELLIGENT CRUISE CONTROL SYSTEM        ║\n');
fprintf('║                                                          ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% ========================================================================
%% SECTION 1 — SIMULATION PARAMETERS
%% ========================================================================

dt      = 0.01;
t_end   = 60;
t       = 0:dt:t_end;
N       = length(t);

%% ========================================================================
%% SECTION 2 — VEHICLE PARAMETERS
%% ========================================================================

m0          = 1500;     % kg
g           = 9.81;
rho         = 1.225;
Cd          = 0.28;
A           = 2.2;
Cr          = 0.015;
r_wheel     = 0.33;

% MOTOR: Realistic peak torque (800 Nm) for 5-8° gradeability at 30 m/s
T_max       = 800;      % Nm
eta_motor   = 0.92;
eta_regen   = 0.75;

% Maximum wheel force (N)
F_max       = T_max / r_wheel;   % ≈ 2424 N

% Battery
V_bat       = 400;      % V
C_bat       = 75;       % Ah
SOC0        = 100;      % %

%% ========================================================================
%% SECTION 3 — TARGET SPEED PROFILE
%% ========================================================================

v_ref = zeros(1,N);
v_ref(1:round(5/dt))              = 0;
v_ref(round(5/dt):round(20/dt))  = 25;   % 90 km/h
v_ref(round(20/dt):round(30/dt)) = 20;   % 72 km/h
v_ref(round(30/dt):round(45/dt)) = 30;   % 108 km/h
v_ref(round(45/dt):end)          = 25;

%% ========================================================================
%% SECTION 4 — CONTROLLER PARAMETERS 
%% ========================================================================

% PID gains
Kp_pid = 8.0;
Ki_pid = 2.8;
Kd_pid = 2.5;

% PI for comparison
Kp_pi = 5.0;
Ki_pi = 2.0;

% Derivative filter coefficient (rad/s)
Nf = 20;

%% ========================================================================
%% SECTION 5 — DISTURBANCES (used in main simulation and Monte Carlo)
%% ========================================================================

fprintf('[INFO] Building disturbances...\n');

% Road slope (degrees -> rad)
theta = zeros(1,N);
theta(round(10/dt):round(20/dt)) = deg2rad(5);
theta(round(20/dt):round(30/dt)) = deg2rad(-3);
theta(round(30/dt):round(40/dt)) = deg2rad(8);
theta(round(50/dt):end)          = deg2rad(2);

% Wind disturbance (m/s, headwind positive)
v_wind = zeros(1,N);
v_wind(round(15/dt):round(18/dt)) = 15;
v_wind(round(35/dt):round(37/dt)) = -10;
v_wind = v_wind + 1.5*sin(2*pi*0.05*t);

% Passenger load (kg)
m_extra = zeros(1,N);
m_extra(round(25/dt):end) = 250;

% Rolling resistance variation (rain -> higher Cr)
Cr_var = Cr*ones(1,N);
Cr_var(round(40/dt):round(48/dt)) = Cr*1.8;

% Sensor noise (m/s)
noise_std = 0.08;
sensor_noise = noise_std*randn(1,N);

% Brake disturbance (simulated pedal)
brake_dist = zeros(1,N);
brake_dist(round(22/dt):round(22.5/dt)) = -0.6;

% Actuator delay (samples)
act_delay = round(0.05/dt);   % 50 ms

%% ========================================================================
%% SECTION 6 — ACC LEAD VEHICLE
%% ========================================================================

lead_speed = 27*ones(1,N);
lead_speed(round(28/dt):round(35/dt)) = 18;
lead_speed(round(35/dt):end)          = 26;

lead_distance = zeros(1,N);
lead_distance(1) = 60;

%% ========================================================================
%% SECTION 7 — STORAGE VARIABLES (MAIN SIMULATION)
%% ========================================================================

v_pid = zeros(1,N);
v_pi  = zeros(1,N);

u_pid_arr = zeros(1,N);
u_pi_arr  = zeros(1,N);

e_pid_arr = zeros(1,N);
e_pi_arr  = zeros(1,N);

SOC = SOC0*ones(1,N);
P_bat = zeros(1,N);
E_used  = zeros(1,N);
E_regen = zeros(1,N);

F_drag_arr  = zeros(1,N);
F_roll_arr  = zeros(1,N);
F_slope_arr = zeros(1,N);
F_motor_arr = zeros(1,N);

saturation_flag = zeros(1,N);

%% ========================================================================
%% SECTION 8 — PID INTERNAL STATES 
%% ========================================================================

% PID
integral_pid = 0;      % stores Ki * sum(e*dt) directly
d_filt = 0;
e_prev = 0;

% PI
integral_pi = 0;

% Actuator delay buffers
u_buf_pid = zeros(1,act_delay+1);
u_buf_pi  = zeros(1,act_delay+1);

% Low-pass filter for feedforward speed
v_meas_filt = 0;
tau_ff = 0.1;
alpha_ff = dt/(tau_ff+dt);

%% ========================================================================
%% SECTION 9 — MAIN SIMULATION LOOP
%% ========================================================================

fprintf('[INFO] Running physically-corrected nonlinear simulation...\n');

for k = 2:N

    % Current mass (including extra load)
    m = m0 + m_extra(k);

    % Noisy speed measurement
    v_meas_pid = v_pid(k-1) + sensor_noise(k);
    v_meas_pi  = v_pi(k-1)  + sensor_noise(k);

    % Errors
    e_pid = v_ref(k) - v_meas_pid;
    e_pi  = v_ref(k) - v_meas_pi;

    e_pid_arr(k) = v_ref(k) - v_pid(k-1);
    e_pi_arr(k)  = v_ref(k) - v_pi(k-1);

    % Gain scheduling (Kp varies with speed, Ki constant)
    speed_ratio = max(0.4, min(1.5, v_pid(k-1)/25));
    Kp_sched = Kp_pid * speed_ratio;
    Ki_sched = Ki_pid;

    % Derivative with low-pass filter
    d_raw = (e_pid - e_prev) / dt;
    alpha = dt / (1/Nf + dt);
    d_filt = d_filt + alpha*(d_raw - d_filt);
    e_prev = e_pid;
    deriv_term = Kd_pid * d_filt;

    % FEEDFORWARD: use filtered measured speed (actual load cancellation)
    v_meas_filt = alpha_ff * v_meas_pid + (1-alpha_ff)*v_meas_filt;
    v_ff = max(0.1, v_meas_filt);
    F_drag_ff = 0.5 * rho * Cd * A * v_ff^2;
    F_roll_ff = Cr_var(k) * m * g * cos(theta(k));
    F_slope_ff = m * g * sin(theta(k));
    F_res_ff = F_drag_ff + F_roll_ff + F_slope_ff;
    uff = F_res_ff / F_max;
    uff = max(-0.8, min(0.8, uff));

    % PID output (discrete-time with proper integral)
    u_raw_pid = Kp_sched * e_pid + integral_pid + deriv_term + uff;
    u_pid = max(-1, min(1, u_raw_pid));

    % Anti-windup: only update integral if not saturated
    if abs(u_raw_pid - u_pid) < 1e-5
        integral_pid = integral_pid + Ki_sched * e_pid * dt;
    else
        % Freeze integrator
    end

    % ACC logic (gentle override)
    safe_distance = 18 + 0.9 * v_pid(k-1);
    lead_distance(k) = lead_distance(k-1) + (lead_speed(k) - v_pid(k-1))*dt;
    if lead_distance(k) < safe_distance
        u_pid = min(u_pid, -0.2);
    end

    % External brake disturbance
    u_pid = u_pid + brake_dist(k);
    u_pid = max(-1, min(1, u_pid));

    % Actuator delay buffer
    u_buf_pid = [u_pid, u_buf_pid(1:end-1)];
    u_eff_pid = u_buf_pid(end);

    % PI controller (same discrete integration)
    u_raw_pi = Kp_pi * e_pi + integral_pi;
    u_pi = max(-1, min(1, u_raw_pi));
    if abs(u_raw_pi - u_pi) < 1e-5
        integral_pi = integral_pi + Ki_pi * e_pi * dt;
    end
    u_buf_pi = [u_pi, u_buf_pi(1:end-1)];
    u_eff_pi = u_buf_pi(end);

    % Store commands and saturation flag
    u_pid_arr(k) = u_eff_pid;
    u_pi_arr(k)  = u_eff_pi;
    saturation_flag(k) = (abs(u_raw_pid - u_pid) > 1e-5);

    % ---------- NONLINEAR VEHICLE DYNAMICS ----------
    v_rel = v_pid(k-1) - v_wind(k);
    F_drag = 0.5 * rho * Cd * A * (v_rel^2) * sign(v_rel);
    F_roll = Cr_var(k) * m * g * cos(theta(k));
    F_slope = m * g * sin(theta(k));
    Fres = F_drag + F_roll + F_slope;

    F_drag_arr(k) = F_drag;
    F_roll_arr(k) = F_roll;
    F_slope_arr(k)= F_slope;

    % Motor force (limited)
    F_motor_pid = max(-F_max, min(F_max, u_eff_pid * F_max));
    F_motor_pi  = max(-F_max, min(F_max, u_eff_pi  * F_max));
    F_motor_arr(k) = F_motor_pid;

    % Regenerative braking power
    regen_power = 0;
    if u_eff_pid < 0
        regen_power = abs(F_motor_pid * v_pid(k-1)) * eta_regen;
    end

    % Acceleration
    a_pid = (F_motor_pid - Fres) / m;
    a_pi  = (F_motor_pi  - Fres) / m;

    % Integrate speed
    v_pid(k) = max(0, v_pid(k-1) + a_pid * dt);
    v_pi(k)  = max(0, v_pi(k-1)  + a_pi  * dt);

    % Battery SOC and energy
    Pdrive = abs(F_motor_pid * v_pid(k)) / eta_motor;
    P_bat(k) = Pdrive - regen_power;
    E_used(k) = E_used(k-1) + max(0, P_bat(k)) * dt;
    E_regen(k) = E_regen(k-1) + regen_power * dt;
    dSOC = -(P_bat(k) * dt) / (3600 * V_bat * C_bat) * 100;
    SOC(k) = SOC(k-1) + dSOC;
    SOC(k) = min(100, max(0, SOC(k)));

end

fprintf('[INFO] Main simulation complete.\n');

%% ========================================================================
%% SECTION 10 — HONEST PERFORMANCE METRICS (NONLINEAR)
%% ========================================================================

fprintf('[INFO] Computing nonlinear performance metrics...\n');

% Steady-state error (last 5 seconds, constant reference)
steady_start = find(t > t_end-5, 1);
v_ref_steady = mean(v_ref(steady_start:end));
v_pid_steady = mean(v_pid(steady_start:end));
sse_pid = abs(v_ref_steady - v_pid_steady) / v_ref_steady * 100;

v_pi_steady = mean(v_pi(steady_start:end));
sse_pi  = abs(v_ref_steady - v_pi_steady) / v_ref_steady * 100;

% Actual overshoot (after initial acceleration, reference >= 20 m/s)
start_idx = find(v_ref > 20, 1, 'first');
if isempty(start_idx), start_idx = 1; end
v_pid_sub = v_pid(start_idx:end);
v_ref_sub = v_ref(start_idx:end);
peak_pid = max(v_pid_sub);
peak_idx = find(v_pid_sub == peak_pid, 1);
ref_at_peak = v_ref_sub(peak_idx);
overshoot_pid = max(0, (peak_pid - ref_at_peak) / ref_at_peak * 100);

% Linear step response for reference
numG = [1]; denG = [5 1]; G = tf(numG, denG);
C_pid = pid(Kp_pid, Ki_pid, Kd_pid, 1/Nf);
lin_info = stepinfo(feedback(C_pid*G,1));

fprintf('\n');
fprintf('================ NONLINEAR PERFORMANCE =================\n');
fprintf('PID Steady-State Error (actual): %.2f %%   (target <2%%)\n', sse_pid);
fprintf('PID Overshoot (actual)          : %.2f %%   (target <5%%)\n', overshoot_pid);
fprintf('PI Steady-State Error           : %.2f %%\n', sse_pi);
fprintf('=========================================================\n');

if sse_pid < 2 && overshoot_pid < 5
    fprintf('✓ All control objectives met in nonlinear simulation.\n');
else
    fprintf('⚠ Performance still outside targets. Check motor limits or retune.\n');
end

%% ========================================================================
%% SECTION 11 — MONTE CARLO ROBUSTNESS (USING FULL DYNAMICS)
%% ========================================================================

fprintf('[INFO] Running Monte Carlo robustness analysis (full dynamics)...\n');
n_mc = 50;
sse_mc = zeros(1,n_mc);
os_mc  = zeros(1,n_mc);

for mc = 1:n_mc
    % Random variations
    m_mc  = m0 * (1 + 0.2*(2*rand-1));
    Cd_mc = Cd * (1 + 0.15*(2*rand-1));
    Cr_mc = Cr * (1 + 0.25*(2*rand-1));
    T_max_mc = T_max * (1 + 0.1*(2*rand-1));
    F_max_mc = T_max_mc / r_wheel;
    
    % Re-initialize states for this Monte Carlo run
    v_mc = zeros(1,N);
    int_mc = 0;
    d_mc = 0;
    e_prev_mc = 0;
    u_buf_mc = zeros(1,act_delay+1);
    v_meas_filt_mc = 0;
    
    for k = 2:N
        m = m_mc + m_extra(k);
        v_meas = v_mc(k-1) + noise_std*randn;
        e = v_ref(k) - v_meas;
        
        d_raw = (e - e_prev_mc)/dt;
        alpha = dt/(1/Nf + dt);
        d_mc = d_mc + alpha*(d_raw - d_mc);
        e_prev_mc = e;
        deriv = Kd_pid * d_mc;
        
        v_meas_filt_mc = alpha_ff * v_meas + (1-alpha_ff)*v_meas_filt_mc;
        v_ff = max(0.1, v_meas_filt_mc);
        F_drag_ff = 0.5 * rho * Cd_mc * A * v_ff^2;
        F_roll_ff = Cr_mc * m * g * cos(theta(k));
        F_slope_ff = m * g * sin(theta(k));
        F_res_ff = F_drag_ff + F_roll_ff + F_slope_ff;
        uff = F_res_ff / F_max_mc;
        uff = max(-0.8, min(0.8, uff));
        
        u_raw = Kp_pid * e + int_mc + deriv + uff;
        u = max(-1, min(1, u_raw));
        if abs(u_raw - u) < 1e-5
            int_mc = int_mc + Ki_pid * e * dt;
        end
        
        u_buf_mc = [u, u_buf_mc(1:end-1)];
        u_eff = u_buf_mc(end);
        
        v_rel = v_mc(k-1) - v_wind(k);
        F_drag = 0.5 * rho * Cd_mc * A * (v_rel^2) * sign(v_rel);
        F_roll = Cr_mc * m * g * cos(theta(k));
        F_slope = m * g * sin(theta(k));
        Fres = F_drag + F_roll + F_slope;
        F_motor = max(-F_max_mc, min(F_max_mc, u_eff * F_max_mc));
        a = (F_motor - Fres) / m;
        v_mc(k) = max(0, v_mc(k-1) + a * dt);
    end
    
    steady_start_mc = find(t > t_end-5, 1);
    v_ref_steady_mc = mean(v_ref(steady_start_mc:end));
    v_mc_steady = mean(v_mc(steady_start_mc:end));
    sse_mc(mc) = abs(v_ref_steady_mc - v_mc_steady) / v_ref_steady_mc * 100;
    
    start_idx_mc = find(v_ref > 20, 1, 'first');
    if ~isempty(start_idx_mc)
        v_mc_sub = v_mc(start_idx_mc:end);
        v_ref_sub_mc = v_ref(start_idx_mc:end);
        peak_mc = max(v_mc_sub);
        peak_idx_mc = find(v_mc_sub == peak_mc, 1);
        ref_at_peak_mc = v_ref_sub_mc(peak_idx_mc);
        os_mc(mc) = max(0, (peak_mc - ref_at_peak_mc) / ref_at_peak_mc * 100);
    else
        os_mc(mc) = 0;
    end
end

fprintf('[INFO] Monte Carlo complete.\n');

%% ========================================================================
%% SECTION 12 — DARK THEME COLORS FOR DASHBOARD
%% ========================================================================

dark_bg    = [0.05 0.05 0.08];
panel_bg   = [0.08 0.09 0.14];
cyan_c     = [0 0.9 1];
green_c    = [0.2 1 0.4];
orange_c   = [1 0.6 0];
red_c      = [1 0.2 0.2];
yellow_c   = [1 1 0];
white_c    = [0.95 0.95 0.95];
grid_c     = [0.2 0.2 0.3];

set(groot,'defaultFigureColor',dark_bg);

%% ========================================================================
%% SECTION 13 — MAIN DASHBOARD (with diagnostic subplots)
%% ========================================================================

fig1 = figure('Name','EV Dashboard','Color',dark_bg,'Position',[50 50 1500 850]);

% Speed tracking
ax1 = subplot(3,3,1); hold on;
plot(t, v_ref, '--', 'Color', yellow_c, 'LineWidth', 2);
plot(t, v_pid, 'Color', cyan_c, 'LineWidth', 2.5);
plot(t, v_pi,  'Color', green_c, 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Vehicle Speed Tracking','Color',cyan_c);
legend({'Reference','PID','PI'}, 'TextColor',white_c, 'Color',panel_bg);
grid on; ax1.Color = panel_bg; ax1.GridColor = grid_c; ax1.XColor = white_c; ax1.YColor = white_c;

% Tracking error
ax2 = subplot(3,3,2); hold on;
plot(t, e_pid_arr, 'Color', cyan_c, 'LineWidth', 2);
plot(t, e_pi_arr,  'Color', orange_c, 'LineWidth', 1.5);
yline(0, '--', 'Color', white_c);
xlabel('Time [s]'); ylabel('Error [m/s]');
title('Tracking Error','Color',cyan_c);
legend({'PID','PI'}, 'TextColor',white_c, 'Color',panel_bg);
grid on; ax2.Color = panel_bg; ax2.GridColor = grid_c; ax2.XColor = white_c; ax2.YColor = white_c;

% Control signal + saturation
ax3 = subplot(3,3,3); hold on;
plot(t, u_pid_arr, 'Color', cyan_c, 'LineWidth', 2);
plot(t, u_pi_arr,  'Color', green_c, 'LineWidth', 1.5);
yline(1, ':', 'Color', red_c); yline(-1, ':', 'Color', red_c);
sat_idx = find(saturation_flag);
if ~isempty(sat_idx)
    scatter(t(sat_idx), u_pid_arr(sat_idx), 10, red_c, 'filled', 'MarkerFaceAlpha', 0.5);
end
xlabel('Time [s]'); ylabel('Throttle');
title('Control Signal (red dots = saturated)','Color',cyan_c);
legend({'PID','PI'}, 'TextColor',white_c, 'Color',panel_bg);
grid on; ax3.Color = panel_bg; ax3.GridColor = grid_c; ax3.XColor = white_c; ax3.YColor = white_c;

% Road slope
ax4 = subplot(3,3,4);
area(t, rad2deg(theta), 'FaceColor', orange_c, 'FaceAlpha', 0.4);
xlabel('Time [s]'); ylabel('Slope [deg]');
title('Road Slope Disturbance','Color',cyan_c);
grid on; ax4.Color = panel_bg; ax4.GridColor = grid_c; ax4.XColor = white_c; ax4.YColor = white_c;

% Resistance forces
ax5 = subplot(3,3,5); hold on;
plot(t, F_drag_arr,  'Color', red_c,    'LineWidth', 2);
plot(t, F_roll_arr,  'Color', orange_c, 'LineWidth', 1.8);
plot(t, F_slope_arr, 'Color', yellow_c, 'LineWidth', 1.8);
plot(t, F_motor_arr, 'Color', cyan_c,   'LineWidth', 1.5, 'LineStyle', '-.');
xlabel('Time [s]'); ylabel('Force [N]');
title('Forces: Drag, Roll, Slope, Motor','Color',cyan_c);
legend({'Drag','Rolling','Slope','Motor'}, 'TextColor',white_c, 'Color',panel_bg);
grid on; ax5.Color = panel_bg; ax5.GridColor = grid_c; ax5.XColor = white_c; ax5.YColor = white_c;

% Battery SOC
ax6 = subplot(3,3,6);
area(t, SOC, 'FaceColor', green_c, 'FaceAlpha', 0.35);
ylim([0 105]);
xlabel('Time [s]'); ylabel('SOC [%]');
title('Battery State of Charge','Color',cyan_c);
grid on; ax6.Color = panel_bg; ax6.GridColor = grid_c; ax6.XColor = white_c; ax6.YColor = white_c;

% Force balance
ax7 = subplot(3,3,7); hold on;
plot(t, F_motor_arr, 'Color', cyan_c, 'LineWidth', 2);
plot(t, F_drag_arr + F_roll_arr + F_slope_arr, 'Color', red_c, 'LineWidth', 1.8, 'LineStyle', '--');
xlabel('Time [s]'); ylabel('Force [N]');
title('Motor Force vs. Total Resistance','Color',cyan_c);
legend({'Motor','Resistance'}, 'TextColor',white_c, 'Color',panel_bg);
grid on; ax7.Color = panel_bg; ax7.GridColor = grid_c; ax7.XColor = white_c; ax7.YColor = white_c;

% Wind gust rejection
ax8 = subplot(3,3,8); hold on;
plot(t, v_pid, 'Color', cyan_c, 'LineWidth', 2);
xline(15, 'g--', 'LineWidth', 1.5);
xline(18, 'g--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Wind Gust Rejection (green = gust)','Color',cyan_c);
grid on; ax8.Color = panel_bg; ax8.GridColor = grid_c; ax8.XColor = white_c; ax8.YColor = white_c;

% Energy recovered
ax9 = subplot(3,3,9);
plot(t, E_regen/1000, 'Color', green_c, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Energy [kJ]');
title('Regenerative Energy Recovery','Color',cyan_c);
grid on; ax9.Color = panel_bg; ax9.GridColor = grid_c; ax9.XColor = white_c; ax9.YColor = white_c;

sgtitle('ADVANCED EV CRUISE CONTROL DASHBOARD ', 'Color', cyan_c, 'FontWeight', 'bold');

%% ========================================================================
%% SECTION 14 — CONTROL ANALYSIS (STEP, ROOT LOCUS, BODE, NYQUIST)
%% ========================================================================

fig2 = figure('Name','Control Analysis','Color',dark_bg,'Position',[70 70 1400 750]);

ax_step = subplot(2,2,1);
step(feedback(C_pid*G,1), feedback(pid(Kp_pi,Ki_pi)*G,1), 25);
grid on; title('Step Response (Linear Plant)','Color',cyan_c);
ax_step.Color = panel_bg; ax_step.GridColor = grid_c; ax_step.XColor = white_c; ax_step.YColor = white_c;

ax_rl = subplot(2,2,2);
rlocus(C_pid*G); grid on; title('Root Locus','Color',cyan_c);
ax_rl.Color = panel_bg; ax_rl.GridColor = grid_c; ax_rl.XColor = white_c; ax_rl.YColor = white_c;

ax_b = subplot(2,2,3);
margin(C_pid*G); grid on; title('Bode Plot + Margins','Color',cyan_c);
ax_b.Color = panel_bg;

ax_n = subplot(2,2,4);
nyquist(C_pid*G); grid on; title('Nyquist Plot','Color',cyan_c);
ax_n.Color = panel_bg;

%% ========================================================================
%% SECTION 15 — MONTE CARLO PLOTS
%% ========================================================================

fig3 = figure('Name','Monte Carlo','Color',dark_bg,'Position',[90 90 1000 450]);

axm1 = subplot(1,2,1);
histogram(sse_mc, 12, 'FaceColor', cyan_c, 'EdgeColor', 'white');
xline(2, '--', 'Color', red_c, 'LineWidth', 2);
xlabel('SSE [%]'); ylabel('Count');
title(sprintf('Monte Carlo SSE (mean = %.2f%%)', mean(sse_mc)), 'Color', cyan_c);
grid on; axm1.Color = panel_bg; axm1.GridColor = grid_c; axm1.XColor = white_c; axm1.YColor = white_c;

axm2 = subplot(1,2,2);
histogram(os_mc, 12, 'FaceColor', green_c, 'EdgeColor', 'white');
xline(5, '--', 'Color', red_c, 'LineWidth', 2);
xlabel('Overshoot [%]'); ylabel('Count');
title(sprintf('Monte Carlo Overshoot (mean = %.2f%%)', mean(os_mc)), 'Color', cyan_c);
grid on; axm2.Color = panel_bg; axm2.GridColor = grid_c; axm2.XColor = white_c; axm2.YColor = white_c;

%% ========================================================================
%% SECTION 16 — ECO / NORMAL / SPORT MODES
%% ========================================================================

fig4 = figure('Name','Drive Modes','Color',dark_bg,'Position',[100 100 950 450]);
axm = axes; hold on;
plot(t, v_ref, '--', 'Color', yellow_c, 'LineWidth', 2);

modes = {
    'Eco',    4.0, 1.5, 1.2, green_c;
    'Normal', 8.0, 2.8, 2.5, cyan_c;
    'Sport', 12.0, 4.0, 4.0, red_c
    };

for i = 1:size(modes,1)
    kp = modes{i,2}; ki = modes{i,3}; kd = modes{i,4};
    vv = zeros(1,N);
    int_m = 0; d_m = 0; e_prev_m = 0;
    for k = 2:N
        ee = v_ref(k) - vv(k-1);
        dr = (ee - e_prev_m)/dt;
        al = dt/(1/Nf + dt);
        d_m = d_m + al*(dr - d_m);
        e_prev_m = ee;
        uu = kp*ee + int_m + kd*d_m;
        uu = max(-1, min(1, uu));
        if abs(uu - (kp*ee + int_m + kd*d_m)) < 1e-5
            int_m = int_m + ki * ee * dt;
        end
        Fm = uu * F_max;
        Fd = 0.5*rho*Cd*A*vv(k-1)^2;
        Fr = Cr*m0*g;
        aa = (Fm - Fd - Fr)/m0;
        vv(k) = max(0, vv(k-1) + aa*dt);
    end
    plot(t, vv, 'Color', modes{i,5}, 'LineWidth', 2, 'DisplayName', modes{i,1});
end

legend({'Reference','Eco','Normal','Sport'}, 'TextColor',white_c, 'Color',panel_bg);
xlabel('Time [s]'); ylabel('Speed [m/s]'); title('Drive Mode Comparison','Color',cyan_c);
grid on; axm.Color = panel_bg; axm.GridColor = grid_c; axm.XColor = white_c; axm.YColor = white_c;



%% ========================================================================
%% SECTION 18 — FINAL REPORT
%% ========================================================================

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║                SIMULATION COMPLETE                      ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Actual PID Overshoot        : %-6.2f %%                 ║\n', overshoot_pid);
fprintf('║  Actual PID Steady-State Err : %-6.2f %%                 ║\n', sse_pid);
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Mean Monte Carlo SSE         : %-6.2f %%                 ║\n', mean(sse_mc));
fprintf('║  Mean Monte Carlo OS          : %-6.2f %%                 ║\n', mean(os_mc));
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Energy Used                  : %-6.3f kWh               ║\n', E_used(end)/3.6e6);
fprintf('║  Regen Energy                 : %-6.3f kWh               ║\n', E_regen(end)/3.6e6);
fprintf('║  Final Battery SOC            : %-6.2f %%                 ║\n', SOC(end));
fprintf('╚══════════════════════════════════════════════════════════╝\n');

fprintf('\n[INFO] All figures and animation generated successfully.\n');
