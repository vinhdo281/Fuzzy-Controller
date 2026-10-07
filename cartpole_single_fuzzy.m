%% =========================================================================
%  BỘ ĐIỀU KHIỂN FUZZY ĐƠN HỢP NHẤT (SINGLE UNIFIED FUZZY CONTROLLER)
%  CHO HỆ CON LẮC NGƯỢC TRÊN XE (CART-POLE INVERTED PENDULUM)
%  
%  Gộp cả vòng cân bằng góc (Theta) và vòng vị trí xe (Position) vào 
%  MỘT BỘ FIS DUY NHẤT (Không dùng 2 vòng Cascade lồng nhau).
%  
%  Cấu trúc: 4 Inputs [Theta, Theta_dot, x, x_dot] -> 1 Output [Force]
%  Phương pháp suy diễn: Mamdani với Sum-Aggregation (chống xung đột lực)
%  Giải mờ: Centroid (Trọng tâm)
% =========================================================================

clear; clc; close all;

%% 1. Khởi tạo Mamdani FIS (4 Inputs, 1 Output)
cpFIS = mamfis(...
    'Name', 'cartpole_single_unified', ...
    'NumInputs', 4, 'NumInputMFs', 2, ...
    'NumOutputs', 1, 'NumOutputMFs', 4, ...
    'AddRule', 'none', ...
    'AndMethod', 'min', ...
    'OrMethod', 'max', ...
    'ImplicationMethod', 'min', ...
    'AggregationMethod', 'sum', ...
    'DefuzzificationMethod', 'centroid');

%% 2. Khai báo Input 1: Theta (rad) - Góc nghiêng con lắc (Mở rộng cho góc lớn đến 20-25 độ)
cpFIS.Inputs(1).Name = 'Theta';
cpFIS.Inputs(1).Range = [-pi, pi];

cpFIS.Inputs(1).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(1).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(1).MembershipFunctions(1).Parameters = [-0.20, 0.20];

cpFIS.Inputs(1).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(1).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(1).MembershipFunctions(2).Parameters = [-0.20, 0.20];

%% 3. Khai báo Input 2: Theta_dot (rad/s) - Vận tốc góc con lắc
cpFIS.Inputs(2).Name = 'Theta_dot';
cpFIS.Inputs(2).Range = [-15, 15];

cpFIS.Inputs(2).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(2).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(2).MembershipFunctions(1).Parameters = [-3.5, 3.5];

cpFIS.Inputs(2).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(2).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(2).MembershipFunctions(2).Parameters = [-3.5, 3.5];

%% 4. Khai báo Input 3: x (m) - Sai số vị trí xe (x - x_ref)
cpFIS.Inputs(3).Name = 'x';
cpFIS.Inputs(3).Range = [-0.4, 0.4];

cpFIS.Inputs(3).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(3).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(3).MembershipFunctions(1).Parameters = [-0.25, 0.25];

cpFIS.Inputs(3).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(3).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(3).MembershipFunctions(2).Parameters = [-0.25, 0.25];

%% 5. Khai báo Input 4: x_dot (m/s) - Vận tốc di chuyển của xe
cpFIS.Inputs(4).Name = 'x_dot';
cpFIS.Inputs(4).Range = [-2.0, 2.0];

cpFIS.Inputs(4).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(4).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(4).MembershipFunctions(1).Parameters = [-0.5, 0.5];

cpFIS.Inputs(4).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(4).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(4).MembershipFunctions(2).Parameters = [-0.5, 0.5];

%% 6. Khai báo Output: Force (N) - Tăng lực điều khiển đáp ứng góc 20 độ
cpFIS.Outputs(1).Name = 'Force';
cpFIS.Outputs(1).Range = [-12, 12];

% Negative Large (-10 N): Lực hãm / đẩy cực mạnh (toàn tải động cơ)
cpFIS.Outputs(1).MembershipFunctions(1).Name = 'NL';
cpFIS.Outputs(1).MembershipFunctions(1).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(1).Parameters = [3.0, 2.0, -10.0];

% Negative Medium (-7 N): Lực phản xạ góc mạnh
cpFIS.Outputs(1).MembershipFunctions(2).Name = 'NM';
cpFIS.Outputs(1).MembershipFunctions(2).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(2).Parameters = [2.5, 2.0, -7.0];

% Positive Medium (+7 N): Lực phản xạ góc mạnh
cpFIS.Outputs(1).MembershipFunctions(3).Name = 'PM';
cpFIS.Outputs(1).MembershipFunctions(3).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(3).Parameters = [2.5, 2.0, 7.0];

% Positive Large (+10 N): Lực hãm / đẩy cực mạnh (toàn tải động cơ)
cpFIS.Outputs(1).MembershipFunctions(4).Name = 'PL';
cpFIS.Outputs(1).MembershipFunctions(4).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(4).Parameters = [3.0, 2.0, 10.0];

%% 7. Tập luật điều khiển hợp nhất (Unified Rule Base)
% Nguyên lý vật lý:
% - Nhóm góc (Theta, Theta_dot): Ưu tiên cao nhất (Weight = 1.0) giữ vững con lắc ngay cả ở 20 độ.
% - Nhóm vị trí (x, x_dot): Trọng số phối hợp (Weight = 0.35) kéo xe giữ trong tầm ray (-0.4m, +0.4m).
rules = [...
    "If Theta is Negative then Force is NM (1.0)"; ...
    "If Theta is Positive then Force is PM (1.0)"; ...
    "If Theta_dot is Negative then Force is NL (1.0)"; ...
    "If Theta_dot is Positive then Force is PL (1.0)"; ...
    "If x is Negative then Force is NM (0.35)"; ...
    "If x is Positive then Force is PM (0.35)"; ...
    "If x_dot is Negative then Force is NL (0.35)"; ...
    "If x_dot is Positive then Force is PL (0.35)"];

cpFIS = addRule(cpFIS, rules);

%% 8. Vẽ đồ thị các hàm liên thuộc & Mặt điều khiển 3D
figure('Name', 'Membership Functions', 'Position', [100 100 900 600]);
subplot(3, 2, 1); plotmf(cpFIS, 'input', 1, 1000); title('Input 1: Theta'); grid on;
subplot(3, 2, 2); plotmf(cpFIS, 'input', 2, 1000); title('Input 2: Theta\_dot'); grid on;
subplot(3, 2, 3); plotmf(cpFIS, 'input', 3, 1000); title('Input 3: x'); grid on;
subplot(3, 2, 4); plotmf(cpFIS, 'input', 4, 1000); title('Input 4: x\_dot'); grid on;
subplot(3, 2, [5 6]); plotmf(cpFIS, 'output', 1, 1000); title('Output: Force'); grid on;

figure('Name', 'Control Surface: Angle Plane');
gensurf(cpFIS, [1 2], 1);
title('Mặt quan hệ điều khiển theo Góc: (Theta, Theta\_dot) -> Force');
xlabel('Theta (rad)'); ylabel('Theta\_dot (rad/s)'); zlabel('Force (N)'); grid on;

figure('Name', 'Control Surface: Position vs Angle Plane');
gensurf(cpFIS, [3 1], 1);
title('Mặt quan hệ điều khiển tổng hợp: (x, Theta) -> Force');
xlabel('x (m)'); ylabel('Theta (rad)'); zlabel('Force (N)'); grid on;

%% 9. Lưu cấu hình cho Simulink
fis = cpFIS;
writeFIS(cpFIS, 'cartpole_single_unified.fis');
fprintf('\n==================================================================\n');
fprintf('  Đã khởi tạo thành công Single Unified Fuzzy Controller!\n');
fprintf('  File "cartpole_single_unified.fis" đã được lưu sẵn sàng cho Simulink.\n');
fprintf('==================================================================\n');

%% 10. Chạy mô phỏng kiểm thử trực tiếp trong MATLAB (Góc nghiêng 20 độ)
fprintf('  Đang chạy mô phỏng phi tuyến hệ xe - con lắc trong MATLAB (20 độ)...\n');

% Thông số vật lý chuẩn từ SolidWorks full_pendulum_assem_DataFile3.m
M_cart = 0.059118;   % Khối lượng xe (kg)
m_pole = 0.019634;   % Khối lượng con lắc (kg)
l_com  = 0.11063;    % Khoảng cách trọng tâm con lắc (m)
I_pole = 1.9733e-4;  % Momen quán tính con lắc (kg.m^2)
g_acc  = 9.81;       % Gia tốc trọng trường (m/s^2)
b_cart = 0.05;       % Hệ số ma sát ray trượt (N.s/m)
b_pole = 0.0003;     % Hệ số cản khớp quay (N.m.s/rad)

dt_sim = 0.002;
t_sim  = 0:dt_sim:5.0;
N_steps = length(t_sim);

x_arr   = zeros(1, N_steps);
xd_arr  = zeros(1, N_steps);
th_arr  = zeros(1, N_steps);
thd_arr = zeros(1, N_steps);
F_arr   = zeros(1, N_steps);

% Điều kiện đầu: Con lắc nghiêng 20 độ (0.349 rad)
th_arr(1) = deg2rad(20.0);

for k = 1:N_steps-1
    % Tính lực từ bộ điều khiển Fuzzy
    F_calc = evalfis(cpFIS, [th_arr(k), thd_arr(k), x_arr(k), xd_arr(k)]);
    F_arr(k) = max(min(F_calc, 10.0), -10.0);
    
    % Động lực học phi tuyến chính xác
    sin_th = sin(th_arr(k));
    cos_th = cos(th_arr(k));
    D_mat = (M_cart + m_pole)*(I_pole + m_pole*l_com^2) - (m_pole*l_com*cos_th)^2;
    
    f1 = F_arr(k) + m_pole*l_com*thd_arr(k)^2*sin_th - b_cart*xd_arr(k);
    f2 = m_pole*g_acc*l_com*sin_th - b_pole*thd_arr(k);
    
    xdd  = ((I_pole + m_pole*l_com^2)*f1 - m_pole*l_com*cos_th*f2) / D_mat;
    thdd = (-m_pole*l_com*cos_th*f1 + (M_cart + m_pole)*f2) / D_mat;
    
    xd_arr(k+1)  = xd_arr(k) + dt_sim*xdd;
    x_arr(k+1)   = x_arr(k) + dt_sim*xd_arr(k+1);
    thd_arr(k+1) = thd_arr(k) + dt_sim*thdd;
    th_arr(k+1)  = th_arr(k) + dt_sim*thd_arr(k+1);
end
F_arr(N_steps) = evalfis(cpFIS, [th_arr(N_steps), thd_arr(N_steps), x_arr(N_steps), xd_arr(N_steps)]);

% Vẽ đồ thị kết quả mô phỏng
figure('Name', 'Simulation Results in MATLAB: 20 deg Recovery', 'Position', [150 150 900 650]);

subplot(3, 1, 1);
plot(t_sim, rad2deg(th_arr), 'r-', 'LineWidth', 2); hold on;
yline(0, 'k:'); yline(1, 'g--'); yline(-1, 'g--');
title('Đáp ứng Góc nghiêng con lắc \theta(t) (Góc ban đầu \theta_0 = 20^\circ)');
ylabel('\theta (độ)'); grid on;
legend('\theta(t)', 'Điểm cân bằng 0^\circ', 'Dải xác lập \pm1^\circ', 'Location', 'northeast');

subplot(3, 1, 2);
plot(t_sim, x_arr, 'b-', 'LineWidth', 2); hold on;
yline(0, 'k:'); yline(0.40, 'r--', 'Giới hạn ray +0.4m'); yline(-0.40, 'r--', 'Giới hạn ray -0.4m');
title('Đáp ứng Vị trí xe x(t) trên thanh ray');
ylabel('x (m)'); grid on;
legend('Vị trí x(t)', 'Gốc 0m', 'Location', 'northeast');

subplot(3, 1, 3);
plot(t_sim, F_arr, 'm-', 'LineWidth', 1.8); hold on;
yline(0, 'k:'); yline(10, 'r:'); yline(-10, 'r:');
title('Lực tác động điều khiển F(t)');
xlabel('Thời gian t (s)'); ylabel('Lực F (N)'); grid on;
legend('Lực F(t)', 'Location', 'northeast');

fprintf('  --> Mô phỏng MATLAB hoàn tất thành công!\n');
fprintf('      Góc cuối cùng: %.3f độ\n', rad2deg(th_arr(end)));
fprintf('      Vị trí cuối cùng: %.4f m\n', x_arr(end));
fprintf('      Độ trượt ray cực đại: %.3f m (Giới hạn cho phép: +/-0.40 m)\n', max(abs(x_arr)));
fprintf('==================================================================\n');
