clc;
clear;
close all;

%% 1. Generator and controller parameters
f0 = 50;       % Nominal frequency (Hz)
H  = 2.5;      % Inertia constant (s)
D  = 1.0;      % Load damping coefficient
Tg = 0.2;      % Governor time constant (s)
Te = 0.5;      % Engine time constant (s)
R  = 0.05;     % Droop coefficient (5 percent)

%% 2. Simulation settings
t_step = 1;            % Time at which the load is applied (s)
dt     = 0.01;         % Simulation time step (s)
t_end  = 30;           % Total simulation duration (s)
t      = 0:dt:t_end;
dPL    = 0.10;         % Base load disturbance (pu)

% Output folder for figures and tables
if ~exist('results', 'dir')
    mkdir('results');
end

%% 3. Transfer functions
s    = tf('s');
Ggen = 1/(2*H*s + D);
Ggov = 1/(Tg*s + 1);
Geng = 1/(Te*s + 1);

%% 4. Closed-loop frequency response
Gcontrol = Ggov * Geng / R;
Gfreq    = -Ggen / (1 + Ggen*Gcontrol);

%% 5. Sudden load increase
load_step = make_step(t, t_step, dPL);

%% 6. Simulate frequency deviation
df        = lsim(Gfreq, load_step, t);
frequency = f0*(1 + df);

%% 7. Governor and engine responses
% Governor increases its command when frequency falls
governor_output  = lsim(Ggov, -df/R, t);
% Engine responds to the governor command
mechanical_power = lsim(Geng, governor_output, t);

%% 8. Plot frequency, governor and mechanical power
fig1 = figure;

subplot(3,1,1);
plot(t, frequency, 'LineWidth', 1.5);
yline(f0, '--k', 'Nominal Frequency');
grid on;
ylabel('Frequency (Hz)');
title('Diesel Generator Response');

subplot(3,1,2);
plot(t, governor_output, 'LineWidth', 1.5);
grid on;
ylabel('Governor output (pu)');

subplot(3,1,3);
plot(t, mechanical_power, 'LineWidth', 1.5);
grid on;
ylabel('Mechanical power (pu)');
xlabel('Time (seconds)');

exportgraphics(fig1, fullfile('results', 'frequency_governor_engine.png'), ...
    'Resolution', 300);

%% 9. Display results and validate against theory
m = compute_metrics(frequency, t, f0, t_step);
f_theory = f0 * (1 - dPL/(D + 1/R));   % analytic steady-state frequency

fprintf('--- Base case: %.0f%% load, %.0f%% droop ---\n', 100*dPL, 100*R);
fprintf('Minimum frequency:           %.3f Hz (at t = %.2f s)\n', ...
        m.min_freq, m.t_min);
fprintf('Final frequency (simulated): %.3f Hz\n', m.final);
fprintf('Final frequency (theory):    %.3f Hz\n', f_theory);
fprintf('Difference sim vs theory:    %.5f Hz\n', abs(m.final - f_theory));
if m.min_freq < m.final - 1e-6
    fprintf('Response undershoots the final value by %.3f Hz.\n', ...
            m.final - m.min_freq);
else
    fprintf('Response does not undershoot the final value.\n');
end
fprintf('Final governor output:       %.4f pu\n', governor_output(end));
fprintf('Final mechanical power:      %.4f pu\n', mechanical_power(end));

%% 10. Compare different droop settings
R_values = [0.03, 0.05, 0.08];

fig2 = figure;
hold on;

for i = 1:length(R_values)
    R_test = R_values(i);

    Gfreq_test = closed_loop(Ggen, Ggov, Geng, R_test);
    df_test    = lsim(Gfreq_test, load_step, t);
    freq_test  = f0 * (1 + df_test);

    plot(t, freq_test, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Droop = %.0f%%', 100*R_test));

    mt = compute_metrics(freq_test, t, f0, t_step);
    fprintf('\nDroop = %.0f%%\n', 100*R_test);
    fprintf('Minimum frequency = %.3f Hz\n', mt.min_freq);
    fprintf('Final frequency   = %.3f Hz\n', mt.final);
end

yline(f0, '--k', 'Nominal Frequency', 'HandleVisibility', 'off');
grid on;
xlabel('Time (seconds)');
ylabel('Frequency (Hz)');
title('Effect of Governor Droop on Frequency Response');
legend('Location', 'best');
hold off;

exportgraphics(fig2, fullfile('results', 'droop_comparison.png'), ...
    'Resolution', 300);

%% 11. Compare different load disturbances (5% droop) and build table
load_values = [0.05, 0.10, 0.20];
n = length(load_values);

Load_Increase        = zeros(n,1);
Minimum_Frequency_Hz = zeros(n,1);
Frequency_Dip_Hz     = zeros(n,1);
Final_Frequency_Hz   = zeros(n,1);
Time_of_Minimum_s    = zeros(n,1);
Settling_Time_s      = zeros(n,1);
SteadyState_Error_Hz = zeros(n,1);

Gfreq_load = closed_loop(Ggen, Ggov, Geng, R);   % same for all cases

fig3 = figure;
hold on;

for i = 1:n
    dPL_test  = load_values(i);
    load_test = make_step(t, t_step, dPL_test);

    df_test   = lsim(Gfreq_load, load_test, t);
    freq_test = f0 * (1 + df_test);
    mt        = compute_metrics(freq_test, t, f0, t_step);

    plot(t, freq_test, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Load increase = %.0f%%', 100*dPL_test));

    % Header first, then its metrics
    fprintf('\nLoad increase = %.0f%%\n', 100*dPL_test);
    fprintf('Minimum frequency = %.3f Hz\n', mt.min_freq);
    fprintf('Time of minimum frequency: %.2f s\n', mt.t_min);
    fprintf('Final frequency = %.3f Hz\n', mt.final);
    fprintf('Frequency dip = %.3f Hz\n', mt.dip);
    fprintf('Settling time after load step: %.2f s\n', mt.ts);
    fprintf('Steady-state frequency error: %.4f Hz\n', mt.sse);

    Load_Increase(i)        = 100*dPL_test;
    Minimum_Frequency_Hz(i) = mt.min_freq;
    Frequency_Dip_Hz(i)     = mt.dip;
    Final_Frequency_Hz(i)   = mt.final;
    Time_of_Minimum_s(i)    = mt.t_min;
    Settling_Time_s(i)      = mt.ts;
    SteadyState_Error_Hz(i) = mt.sse;
end

yline(f0, '--k', 'Nominal Frequency', 'HandleVisibility', 'off');
grid on;
xlabel('Time (seconds)');
ylabel('Frequency (Hz)');
title('Frequency Response for Different Load Increases');
legend('Location', 'best');
hold off;

exportgraphics(fig3, fullfile('results', 'load_comparison.png'), ...
    'Resolution', 300);

%% 12. Performance comparison table
Results = table(Load_Increase, Minimum_Frequency_Hz, Frequency_Dip_Hz, ...
    Final_Frequency_Hz, Time_of_Minimum_s, Settling_Time_s, ...
    SteadyState_Error_Hz);

fprintf('\n');
disp(Results);
writetable(Results, fullfile('results', 'performance_table.csv'));

%% Local functions (must be at the end of the script)

function Gf = closed_loop(Ggen, Ggov, Geng, R)
% Closed-loop transfer function from load disturbance to frequency deviation
    Gc = Ggov * Geng / R;
    Gf = -Ggen / (1 + Ggen*Gc);
end

function u = make_step(t, t_step, amplitude)
% Step input of given amplitude starting at t_step
    u = zeros(size(t));
    u(t >= t_step) = amplitude;
end

function m = compute_metrics(freq, t, f0, t_step)
% Performance metrics for a frequency response (Hz)
    [m.min_freq, idx] = min(freq);
    m.t_min = t(idx);
    m.final = freq(end);
    m.dip   = f0 - m.min_freq;
    m.sse   = f0 - m.final;

    % Settling band: 2% of the final deviation from nominal,
    % centred on the final value
    tol = 0.02 * abs(m.final - f0);
    out = find(abs(freq - m.final) > tol, 1, 'last');
    if isempty(out)
        m.ts = 0;
    else
        m.ts = t(out) - t_step;
    end
end