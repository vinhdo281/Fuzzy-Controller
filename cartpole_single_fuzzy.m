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

%% 2. Khai báo Input 1: Theta (rad) - Góc nghiêng con lắc
cpFIS.Inputs(1).Name = 'Theta';
cpFIS.Inputs(1).Range = [-pi, pi];

cpFIS.Inputs(1).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(1).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(1).MembershipFunctions(1).Parameters = [-0.15, 0.15];

cpFIS.Inputs(1).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(1).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(1).MembershipFunctions(2).Parameters = [-0.15, 0.15];

%% 3. Khai báo Input 2: Theta_dot (rad/s) - Vận tốc góc con lắc
cpFIS.Inputs(2).Name = 'Theta_dot';
cpFIS.Inputs(2).Range = [-15, 15];

cpFIS.Inputs(2).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(2).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(2).MembershipFunctions(1).Parameters = [-2.5, 2.5];

cpFIS.Inputs(2).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(2).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(2).MembershipFunctions(2).Parameters = [-2.5, 2.5];

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

%% 6. Khai báo Output: Force (N) - Lực tác động lên xe
cpFIS.Outputs(1).Name = 'Force';
cpFIS.Outputs(1).Range = [-10, 10];

% Negative Large (-8 N): Lực hãm / đẩy cực mạnh
cpFIS.Outputs(1).MembershipFunctions(1).Name = 'NL';
cpFIS.Outputs(1).MembershipFunctions(1).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(1).Parameters = [3.0, 2.0, -8.0];

% Negative Medium (-3.5 N): Lực vừa
cpFIS.Outputs(1).MembershipFunctions(2).Name = 'NM';
cpFIS.Outputs(1).MembershipFunctions(2).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(2).Parameters = [2.0, 2.0, -3.5];

% Positive Medium (+3.5 N): Lực vừa
cpFIS.Outputs(1).MembershipFunctions(3).Name = 'PM';
cpFIS.Outputs(1).MembershipFunctions(3).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(3).Parameters = [2.0, 2.0, 3.5];

% Positive Large (+8 N): Lực hãm / đẩy cực mạnh
cpFIS.Outputs(1).MembershipFunctions(4).Name = 'PL';
cpFIS.Outputs(1).MembershipFunctions(4).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(4).Parameters = [3.0, 2.0, 8.0];

%% 7. Tập luật điều khiển hợp nhất (Unified Rule Base)
% Nguyên lý vật lý:
% - Nhóm góc (Theta, Theta_dot): Ưu tiên cao nhất (Weight = 1.0) giữ vững con lắc.
% - Nhóm vị trí (x, x_dot): Ưu tiên vừa (Weight = 0.25). 
%   Khi xe ở bên phải (x > 0), đẩy xe sang phải (PM) làm con lắc nghiêng sang trái,
%   sau đó nhóm góc kéo xe chạy sang trái về lại gốc 0 (Cơ chế Nghiêng Để Lái hợp nhất).
rules = [...
    "If Theta is Negative then Force is NM (1.0)"; ...
    "If Theta is Positive then Force is PM (1.0)"; ...
    "If Theta_dot is Negative then Force is NL (1.0)"; ...
    "If Theta_dot is Positive then Force is PL (1.0)"; ...
    "If x is Negative then Force is NM (0.25)"; ...
    "If x is Positive then Force is PM (0.25)"; ...
    "If x_dot is Negative then Force is NL (0.25)"; ...
    "If x_dot is Positive then Force is PL (0.25)"];

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
