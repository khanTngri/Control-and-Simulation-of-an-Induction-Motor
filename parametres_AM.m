%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Project: Control and Simulation of an Induction Motor (Motor No.5 – 4AP80-2)
% Author: Ing. Yevgeniy Makarenko
% Year: 2025
%
% Description:
% This script performs the analysis, parameter calculation, and simulation 
% of an induction motor using three control strategies:
%   1) Direct-on-line connection (DOL)
%   2) Scalar (V/f) control
%   3) Vector (Field-Oriented) control
%
% License:
% © 2025 Ing. Yevgeniy Makarenko
% This work is protected under a custom non-commercial license.
% You are free to use and modify this code for educational and research purposes.
% Commercial use or resale is prohibited.
% Contact: e.makarenko.em49@gmail.com
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Motor parameters
p = 2;                         % Number of pole pairs
P = 1100;                      % Motor power [W]
f = 50;                        % Supply frequency [Hz]
U = 380;                       % Line voltage [V]
I = 2.47;                      % Rated current [A]
ni = 0.765;                    % Efficiency
cosfi = 0.88;                  % Power factor
n = 2840;                      % Rated speed [rpm]
M = 3.7;                       % Rated torque [Nm]
R1 = 7.03;                     % Stator resistance [Ω]
R2 = 3.44;                     % Rotor resistance (referred to stator) [Ω]
Xh = 176.77;                   % Magnetizing reactance
Lh = 0.56;                     % Magnetizing inductance [H]
L1sig = 0.019;                 % Stator leakage inductance [H]
J = 0.00096 * 5;               % Moment of inertia [kg·m²]
Kt = 40;                       % Converter gain
w = 2 * pi / 60 * n;           % Angular speed [rad/s]
ws = 2 * p * f / p;            % Synchronous angular speed [rad/s]

%% Calculated parameters
Uf = U / f;
L1 = Lh + L1sig;               
L2 = L1;                       
L2sig = L1sig;                 
sig = 1 - (Lh^2 / (L1 * L2));  
T1 = L1 / R1;                  
T2 = L2 / R2;                  
Lm = 2 * Lh / 3;               
sig1 = L1sig / Lh;             
sig2 = L2sig / Lh;             

%% Current regulator for i1x and i1y
KI = R1 / (2 * sig * T1 * Kt);

%% Magnetizing current regulator
TrI = T2;
KrI = T2 / (2 * 2 * sig * T1);
TiI = TrI / KrI;

%% Calculation of rated magnetizing current
K = ((3 * p / 2) * (Lh / (1 + sig2)));
D = (-K^2 * 2 * I^2)^2 - (4 * K^2 * M^2);
i1y1 = sqrt((-(-K^2 * 2 * I^2) + sqrt(D)) / (2 * K^2));    
i1y2 = sqrt((-(-K^2 * 2 * I^2) - sqrt(D)) / (2 * K^2));    
i2mN1 = i1y1 / sqrt(2);                         
i2mN2 = i1y2 / sqrt(2);                         
i2mN = i2mN2;                                   

%% Speed (angular velocity) regulator
Trw = 4 * 2 * sig * T1;                               
Krw = (J * (1 + sig2)) / (6 * sig * T1 * p^2 * Lh * i2mN);  
Tiw = Trw / Krw;

%% Voltage limiter at inverter output
Ulim = 380;
%% Eunsure output folder exists
output_folder = 'img';
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end
%% Direct-on-line connection of the induction motor
sim("model_am_direct.slx");

figure(1)
plot(Time, omega, 'r', Time, wn_s, 'b', 'LineWidth', 2.5)
grid minor
wn_val = omega(end);
legend(sprintf('\\omega_{sim} = %.2f rad/s', wn_val), ...
       sprintf('\\omega_{ref} = %.2f rad/s (calculated)', w));
ylabel('\omega (rad/s)', 'FontWeight', 'bold')
xlabel('Time (s)', 'FontWeight', 'bold')
title('Simulated and reference angular velocity of the induction motor')
saveas(gcf, fullfile(output_folder, 'direct_velocity.png'))
%% Phase currents (Direct-on-line)
figure(2)
plot(Time, ia, 'r', Time, ib, 'b', Time, ic, 'g', 'LineWidth', 1.5)
grid minor
Ia_val = ia(end); Ib_val = ib(end); Ic_val = ic(end);
legend(sprintf('I_a = %.2f A', Ia_val), ...
       sprintf('I_b = %.2f A', Ib_val), ...
       sprintf('I_c = %.2f A', Ic_val), 'Location', 'best');
ylabel('Current (A)', 'FontWeight', 'bold')
xlabel('Time (s)', 'FontWeight', 'bold')
title('Phase currents during direct-on-line connection')
saveas(gcf, fullfile(output_folder, 'direct_currents.png'))
%% Scalar (V/f) control of induction motor
sim("model_am_scalar.slx");
figure(3)
plot(Time_scalar, ia_scalar, 'r', Time_scalar, ib_scalar, 'b', Time_scalar, ic_scalar, 'g', 'LineWidth', 0.5)
grid minor
legend('I_a', 'I_b', 'I_c', 'Location', 'best');
ylabel('Current (A)', 'FontWeight', 'bold')
xlabel('Time (s)', 'FontWeight', 'bold')
title('Phase currents under scalar (V/f) control')
saveas(gcf, fullfile(output_folder, 'scalar_currents.png'))
figure(4)
plot(Time, omega_scalar, 'r', 'LineWidth', 2.5)
grid minor
omega_scalar_val = omega_scalar(end);
legend(sprintf('\\omega_{V/F} = %.2f rad/s', omega_scalar_val));
ylabel('\omega (rad/s)', 'FontWeight', 'bold')
xlabel('Time (s)', 'FontWeight', 'bold')
title('Simulated and reference angular velocity (V/f control)')
saveas(gcf, fullfile(output_folder, 'scalar_velocity.png'))
%% Vector (Field-Oriented) control
sim("model_am_FOC.slx");
figure(5)
plot(Time_FOC, ia_FOC, 'r', Time_FOC, ib_FOC, 'b', Time_FOC, ic_FOC, 'g', 'LineWidth', 1.5)
grid minor
Ia_FOC_val = ia_FOC(end); Ib_FOC_val = ib_FOC(end); Ic_FOC_val = ic_FOC(end);
legend(sprintf('I_{a} = %.2f A', Ia_FOC_val), ...
       sprintf('I_{b} = %.2f A', Ib_FOC_val), ...
       sprintf('I_{c} = %.2f A', Ic_FOC_val), ...
       'Location', 'best');
ylabel('Current (A)', 'FontWeight', 'bold')
xlabel('Time (s)', 'FontWeight', 'bold')
title('Phase currents under vector control')
saveas(gcf, fullfile(output_folder, 'vector_currents.png'))
%% Speed and torque comparison under vector control
figure(6)
subplot(2,1,1)
plot(Time_FOC, w_act_FOC, 'b', Time_FOC, w_ref_FOC, 'r', 'LineWidth', 2.5)
grid minor
legend('w_{actual}', 'w_{reference}', 'Location', 'best');
ylabel('Speed (rad/s)', 'FontWeight', 'bold')
xlabel('Time (s)', 'FontWeight', 'bold')
title('Actual vs Reference Speed under Vector Control')
subplot(2,1,2)
plot(Time_FOC, Torque_FOC, 'r', 'Linewidth', 2.5)
grid minor
ylabel('Torque (Nm)', 'FontWeight', 'bold')
xlabel('Time (s)', 'FontWeight', 'bold')
title('Electromagnetic Torque under Vector Control')
saveas(gcf, fullfile(output_folder, 'vector_speed_torque.png'))
%% Detailed comparison plots
figure(7)
subplot(2,2,1)
plot(Time_FOC, w_ref_FOC, 'r', Time_FOC, w_act_FOC, 'b', ...
     Time_FOC, w2m_FOC, 'g', Time_FOC, wslip_FOC, 'm', 'LineWidth', 2.5)
grid minor
legend('w_{ref}', 'w_{act}', 'w_{2m}', 'w_{slip}', 'Location', 'best');
ylabel('Speed (rad/s)', 'FontWeight', 'bold')
xlabel('Time (s)', 'FontWeight', 'bold')
title('Speed comparison under vector control')

subplot(2,2,2)
plot(Time_FOC, i1x_FOC, 'r', 'LineWidth', 2.5)
grid minor
ylabel('i_{1x} (A)', 'FontWeight', 'bold')
xlabel('Time (s)', 'FontWeight', 'bold')
title('Flux-producing current i_{1x} under vector control')

subplot(2,2,4)
plot(Time_FOC, i1y_FOC, 'b', 'LineWidth', 2.5)
grid minor
ylabel('i_{1y} (A)', 'FontWeight', 'bold')
xlabel('Time (s)', 'FontWeight', 'bold')
title('Torque-producing current i_{1y} under vector control')

subplot(2,2,3)
plot(Time_FOC, fi2m_FOC, 'g', 'LineWidth', 2.5)
grid minor
ylabel('\phi_{2m} (rad)', 'FontWeight', 'bold')
xlabel('Time (s)', 'FontWeight', 'bold')
title('Magnetic flux angle \phi_{2m} under vector control')
saveas(gcf, fullfile(output_folder, 'vector_detailed_comparison.png'))