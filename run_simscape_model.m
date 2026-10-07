%% run_simscape_model.m
% Chay mo hinh Simscape Multibody tu full_pendulum_assem.xml
% + Dieu khien Fuzzy Logic (Single Unified FIS)
%
% Yeu cau: MATLAB + Simscape Multibody Toolbox + Fuzzy Logic Toolbox
%
% Buoc 1: Import XML -> tao Simulink model (.slx)
% Buoc 2: Gan FIS va chay simulation
% Buoc 3: Ve ket qua

clc; clear; close all;

%% =============================================
%% BUOC 1: Xay dung Fuzzy FIS (Single Unified)
%% =============================================

% Tao Mamdani FIS voi 4 dau vao, 1 dau ra
fis = mamfis('NumInputs', 4, 'NumInputMFs', 2, ...
             'NumOutputs', 1, 'NumOutputMFs', 4, ...
             'AddRule', 'none');

% --- Input 1: Theta (goc nghieng, rad) ---
fis.Inputs(1).Name = 'Theta';
fis.Inputs(1).Range = [-pi, pi];
fis.Inputs(1).MembershipFunctions(1).Name = 'Negative';
fis.Inputs(1).MembershipFunctions(1).Type = 'zmf';
fis.Inputs(1).MembershipFunctions(1).Parameters = [-0.20, 0.20];
fis.Inputs(1).MembershipFunctions(2).Name = 'Positive';
fis.Inputs(1).MembershipFunctions(2).Type = 'smf';
fis.Inputs(1).MembershipFunctions(2).Parameters = [-0.20, 0.20];

% --- Input 2: Theta_dot (van toc goc, rad/s) ---
fis.Inputs(2).Name = 'Theta_dot';
fis.Inputs(2).Range = [-15, 15];
fis.Inputs(2).MembershipFunctions(1).Name = 'Negative';
fis.Inputs(2).MembershipFunctions(1).Type = 'zmf';
fis.Inputs(2).MembershipFunctions(1).Parameters = [-3.5, 3.5];
fis.Inputs(2).MembershipFunctions(2).Name = 'Positive';
fis.Inputs(2).MembershipFunctions(2).Type = 'smf';
fis.Inputs(2).MembershipFunctions(2).Parameters = [-3.5, 3.5];

% --- Input 3: X (vi tri xe, m) ---
fis.Inputs(3).Name = 'X';
fis.Inputs(3).Range = [-0.5, 0.5];
fis.Inputs(3).MembershipFunctions(1).Name = 'Negative';
fis.Inputs(3).MembershipFunctions(1).Type = 'zmf';
fis.Inputs(3).MembershipFunctions(1).Parameters = [-0.25, 0.25];
fis.Inputs(3).MembershipFunctions(2).Name = 'Positive';
fis.Inputs(3).MembershipFunctions(2).Type = 'smf';
fis.Inputs(3).MembershipFunctions(2).Parameters = [-0.25, 0.25];

% --- Input 4: X_dot (van toc xe, m/s) ---
fis.Inputs(4).Name = 'X_dot';
fis.Inputs(4).Range = [-2, 2];
fis.Inputs(4).MembershipFunctions(1).Name = 'Negative';
fis.Inputs(4).MembershipFunctions(1).Type = 'zmf';
fis.Inputs(4).MembershipFunctions(1).Parameters = [-0.5, 0.5];
fis.Inputs(4).MembershipFunctions(2).Name = 'Positive';
fis.Inputs(4).MembershipFunctions(2).Type = 'smf';
fis.Inputs(4).MembershipFunctions(2).Parameters = [-0.5, 0.5];

% --- Output: Force (luc day xe, N) ---
fis.Outputs(1).Name = 'Force';
fis.Outputs(1).Range = [-12, 12];

fis.Outputs(1).MembershipFunctions(1).Name = 'NL';
fis.Outputs(1).MembershipFunctions(1).Type = 'gbellmf';
fis.Outputs(1).MembershipFunctions(1).Parameters = [3.0, 2, -10];

fis.Outputs(1).MembershipFunctions(2).Name = 'NM';
fis.Outputs(1).MembershipFunctions(2).Type = 'gbellmf';
fis.Outputs(1).MembershipFunctions(2).Parameters = [2.5, 2, -7];

fis.Outputs(1).MembershipFunctions(3).Name = 'PM';
fis.Outputs(1).MembershipFunctions(3).Type = 'gbellmf';
fis.Outputs(1).MembershipFunctions(3).Parameters = [2.5, 2, 7];

fis.Outputs(1).MembershipFunctions(4).Name = 'PL';
fis.Outputs(1).MembershipFunctions(4).Type = 'gbellmf';
fis.Outputs(1).MembershipFunctions(4).Parameters = [3.0, 2, 10];

% --- Luat dieu khien (8 luat) ---
% Luat goc - trong so cao (1.0)
rules_str = [...
    "If Theta is Negative then Force is NL", ...
    "If Theta is Positive then Force is PL", ...
    "If Theta_dot is Negative then Force is NM", ...
    "If Theta_dot is Positive then Force is PM", ...
    "If X is Negative then Force is NM", ...
    "If X is Positive then Force is PM", ...
    "If X_dot is Negative then Force is NM", ...
    "If X_dot is Positive then Force is PM"];

% Them luat voi trong so khac nhau
rule_weights_theta = [1.0, 1.0, 1.0, 1.0];
rule_weights_pos   = [0.35, 0.35, 0.35, 0.35];

fis = addRule(fis, rules_str(1:4));  % Luat goc
fis = addRule(fis, rules_str(5:8));  % Luat vi tri

% Chinh trong so luat
for i = 1:4
    fis.Rules(i).Weight = 1.0;    % Luat goc: trong so cao
end
for i = 5:8
    fis.Rules(i).Weight = 0.35;   % Luat vi tri: trong so thap
end

% Luu FIS
writeFIS(fis, 'cartpole_single_unified.fis');
disp('Da xay dung xong FIS!');

%% =============================================
%% BUOC 2: Ve Membership Functions
%% =============================================
figure('Name', 'Membership Functions', 'NumberTitle', 'off');
subplot(2,3,1); plotmf(fis, 'input', 1, 500);
title('Input 1: Theta (rad)'); grid on;

subplot(2,3,2); plotmf(fis, 'input', 2, 500);
title('Input 2: Theta\_dot (rad/s)'); grid on;

subplot(2,3,3); plotmf(fis, 'input', 3, 500);
title('Input 3: X (m)'); grid on;

subplot(2,3,4); plotmf(fis, 'input', 4, 500);
title('Input 4: X\_dot (m/s)'); grid on;

subplot(2,3,5); plotmf(fis, 'output', 1, 500);
title('Output: Force (N)'); grid on;

subplot(2,3,6); gensurf(fis, [1 2]);
title('Mat dieu khien: Theta vs Theta\_dot'); grid on;

%% =============================================
%% BUOC 3: Import Simscape Multibody Model
%% =============================================
disp('');
disp('=== IMPORT SIMSCAPE MULTIBODY MODEL ===');

xml_file = 'full_pendulum_assem.xml';

if ~isfile(xml_file)
    error('Khong tim thay file: %s\nHay dam bao file o cung thu muc!', xml_file);
end

disp(['Dang import: ' xml_file ' ...']);

try
    % Import XML -> Simulink model
    % smimport se tao file full_pendulum_assem.slx trong thu muc hien tai
    smimport(xml_file);
    disp('Import thanh cong! Model Simulink da duoc tao.');
    disp('Mo Simulink model: full_pendulum_assem.slx');
    
    % Mo model trong Simulink
    open_system('full_pendulum_assem');
    
    disp('');
    disp('=== HUONG DAN TIEP THEO ===');
    disp('1. Trong Simulink, them khoi "Fuzzy Logic Controller"');
    disp('   vao giua phan cam bien va co cau chap hanh');
    disp('2. Gan FIS: dat duong dan den cartpole_single_unified.fis');
    disp('3. Noi:');
    disp('   - Cam bien goc [Theta, Theta_dot] -> FIS Inputs 1,2');
    disp('   - Cam bien vi tri [X, X_dot] -> FIS Inputs 3,4');
    disp('   - FIS Output [Force] -> Actuator (luc doc theo ray)');
    disp('4. Chay simulation trong 10 giay');
    
catch ME
    disp('');
    warning('Import that bai: %s', ME.message);
    disp('');
    disp('=== CHUYEN SANG CHE DO MATLAB SIMULATION ===');
    disp('Chay simulation bang phuong trinh vi phan (khong can Simulink)...');
    
    % Ket qua tham so tu full_pendulum_assem.xml
    run_matlab_sim(fis);
end

%% =============================================
%% HAM PHU: Chay simulation MATLAB thuan tuy
%% (su dung tham so tu full_pendulum_assem.xml)
%% =============================================
function run_matlab_sim(fis)
    disp('');
    disp('--- Dang chay simulation MATLAB ---');
    
    % Tham so vat ly tu SolidWorks XML
    M = 0.05911769838837861;   % Khoi luong xe (kg)
    m = 0.019633826078982265;  % Khoi luong con lac (kg)
    l = 0.11062852776227509;   % Khoang cach COM tu khop xoay (m)
    g = 9.81;                  % Gia toc trong truong (m/s^2)
    
    % Moment quan tinh con lac quanh khop xoay
    % I_com (truc x trong DataFile3) = 197.33 kg*mm^2 = 1.9733e-4 kg*m^2
    I_com = 1.9733090023537565e-4;
    I_pole = I_com + m * l^2;  % Dinh ly Steiner
    
    % Ma sat
    b_cart = 0.05;   % Ma sat ray (N.s/m)
    b_pole = 0.002;  % Ma sat khop (N.m.s/rad)
    
    % Gioi han ray
    x_limit = 0.4875;  % m (tu XML: rail length = 0.975m -> ban kinh 0.4875m)
    F_max = 10;        % N (luc toi da)
    
    % Dieu kien ban dau
    theta0_deg = 20;  % Do nghieng ban dau (do)
    theta0 = theta0_deg * pi / 180;
    
    x0 = 0;     % Xe tai goc toa do
    dx0 = 0;    % Van toc xe ban dau
    dtheta0 = 0; % Van toc goc ban dau
    
    % Vecto trang thai: [x, dx, theta, dtheta]
    state = [x0; dx0; theta0; dtheta0];
    
    % Tham so ket qua de khap
    p = M + m;
    I_eff = I_pole;
    
    % Thoi gian simulation
    dt = 0.002;    % Time step (500 Hz)
    T_sim = 10;    % Thoi gian total (s)
    N = round(T_sim / dt);
    
    % Luu ket qua
    time_arr    = zeros(1, N);
    theta_arr   = zeros(1, N);
    dtheta_arr  = zeros(1, N);
    x_arr       = zeros(1, N);
    dx_arr      = zeros(1, N);
    force_arr   = zeros(1, N);
    
    fprintf('Bat dau simulation (theta0 = %.1f deg)...\n', theta0_deg);
    
    for k = 1:N
        t = (k-1) * dt;
        x      = state(1);
        dx     = state(2);
        theta  = state(3);
        dtheta = state(4);
        
        % Lay dau vao cho FIS
        in1 = max(-pi, min(pi, theta));
        in2 = max(-15, min(15, dtheta));
        in3 = max(-0.5, min(0.5, x));
        in4 = max(-2, min(2, dx));
        
        % Tinh luc tu FIS
        F_raw = evalfis(fis, [in1, in2, in3, in4]);
        
        % Bao hoa luc
        F = max(-F_max, min(F_max, F_raw));
        
        % Phuong trinh dong luc hoc con lac nguoc
        % (Linearized equations of motion)
        sin_th = sin(theta);
        cos_th = cos(theta);
        
        D = p * I_eff - (m * l * cos_th)^2;
        
        % Gia toc xe
        ddx = (I_eff * (F - b_cart * dx + m * l * dtheta^2 * sin_th) ...
               + m * l * cos_th * (m * g * l * sin_th - b_pole * dtheta)) / D;
        
        % Gia toc goc
        ddtheta = (p * (m * g * l * sin_th - b_pole * dtheta) ...
                   - m * l * cos_th * (F - b_cart * dx + m * l * dtheta^2 * sin_th)) / D;
        
        % Tich phan Euler
        state(1) = x + dx * dt;
        state(2) = dx + ddx * dt;
        state(3) = theta + dtheta * dt;
        state(4) = dtheta + ddtheta * dt;
        
        % Gioi han ray
        if abs(state(1)) > x_limit
            state(1) = sign(state(1)) * x_limit;
            state(2) = 0;
        end
        
        % Luu ket qua
        time_arr(k)   = t;
        theta_arr(k)  = theta * 180 / pi;
        dtheta_arr(k) = dtheta;
        x_arr(k)      = x;
        dx_arr(k)     = dx;
        force_arr(k)  = F;
    end
    
    % Kiem tra ket qua
    final_theta = abs(theta_arr(end));
    final_x = abs(x_arr(end));
    max_x = max(abs(x_arr));
    
    fprintf('\n--- KET QUA SIMULATION ---\n');
    fprintf('Goc cuoi: %.3f deg\n', final_theta);
    fprintf('Vi tri cuoi: %.4f m\n', x_arr(end));
    fprintf('Vi tri xe toi da: %.4f m (gioi han: %.4f m)\n', max_x, x_limit);
    
    if final_theta < 5 && max_x < x_limit
        fprintf('-> THANH CONG! Con lac on dinh.\n');
    else
        fprintf('-> THAT BAI. Can dieu chinh them.\n');
    end
    
    % =============================================
    % Ve do thi ket qua
    % =============================================
    figure('Name', 'Ket qua Simulation - Single Fuzzy Controller', ...
           'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);
    
    subplot(3,2,1);
    plot(time_arr, theta_arr, 'b', 'LineWidth', 1.5); hold on;
    yline(0, 'k--', 'LineWidth', 1);
    yline(5, 'r--', 'LineWidth', 0.8);
    yline(-5, 'r--', 'LineWidth', 0.8);
    xlabel('Thoi gian (s)'); ylabel('Goc (do)');
    title('Goc nghieng Theta'); grid on; legend('Theta', 'Set-point');
    
    subplot(3,2,2);
    plot(time_arr, dtheta_arr, 'm', 'LineWidth', 1.5);
    xlabel('Thoi gian (s)'); ylabel('rad/s');
    title('Van toc goc Theta\_dot'); grid on;
    
    subplot(3,2,3);
    plot(time_arr, x_arr * 100, 'g', 'LineWidth', 1.5); hold on;
    yline(x_limit * 100, 'r--', 'LineWidth', 1);
    yline(-x_limit * 100, 'r--', 'LineWidth', 1);
    xlabel('Thoi gian (s)'); ylabel('Vi tri (cm)');
    title('Vi tri xe X'); grid on;
    legend('X', 'Gioi han ray');
    
    subplot(3,2,4);
    plot(time_arr, dx_arr, 'c', 'LineWidth', 1.5);
    xlabel('Thoi gian (s)'); ylabel('m/s');
    title('Van toc xe X\_dot'); grid on;
    
    subplot(3,2,5);
    plot(time_arr, force_arr, 'r', 'LineWidth', 1.5);
    xlabel('Thoi gian (s)'); ylabel('Luc (N)');
    title('Luc dieu khien F'); grid on;
    
    subplot(3,2,6);
    plot(theta_arr, x_arr * 100, 'k', 'LineWidth', 1.2);
    xlabel('Goc (do)'); ylabel('Vi tri (cm)');
    title('Phase Portrait: Theta vs X'); grid on;
    
    sgtitle(sprintf('Single Unified Fuzzy Controller | theta_0 = %d deg | M=%.4f kg, m=%.4f kg, l=%.4f m', ...
            theta0_deg, M, m, l), 'FontSize', 12, 'FontWeight', 'bold');
    
    % Luu hinh
    saveas(gcf, 'simscape_fuzzy_result.png');
    fprintf('Da luu do thi: simscape_fuzzy_result.png\n');
end

