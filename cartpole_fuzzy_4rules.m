%% =========================================================================
%  BỘ ĐIỀU KHIỂN FUZZY 4 LUẬT CHO HỆ CON LẮC NGƯỢC (CART-POLE)
%  Mô hình Mamdani FIS: 2 Inputs (Theta, Theta_dot) -> 1 Output (Force)
% =========================================================================

clear; clc; close all;

%% 1. Khởi tạo Mamdani FIS (2 inputs, 1 output)
cpFIS = mamfis(...
    'Name', 'cartpole_4rules', ...
    'NumInputs', 2, 'NumInputMFs', 2, ...
    'NumOutputs', 1, 'NumOutputMFs', 4, ...
    'AddRule', 'none');

%% 2. Input 1: Theta - Thu hẹp về [-0.2, 0.2] để nhạy ở dải góc nhỏ (~10 độ)
cpFIS.Inputs(1).Name = 'Theta';
cpFIS.Inputs(1).Range = [-pi, pi];

cpFIS.Inputs(1).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(1).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(1).MembershipFunctions(1).Parameters = [-0.2, 0.2];

cpFIS.Inputs(1).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(1).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(1).MembershipFunctions(2).Parameters = [-0.2, 0.2];

figure('Name', 'Input 1: Theta');
plotmf(cpFIS, 'input', 1, 1000);
title('Input 1: Theta (rad)'); xlabel('Theta (rad)'); ylabel('Membership Degree'); grid on;

%% 3. Input 2: Theta_dot - Vận tốc góc dập dao động nhanh
cpFIS.Inputs(2).Name = 'Theta_dot';
cpFIS.Inputs(2).Range = [-15, 15];

cpFIS.Inputs(2).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(2).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(2).MembershipFunctions(1).Parameters = [-3, 3];

cpFIS.Inputs(2).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(2).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(2).MembershipFunctions(2).Parameters = [-3, 3];

figure('Name', 'Input 2: Theta_dot');
plotmf(cpFIS, 'input', 2, 1000);
title('Input 2: Theta\_dot (rad/s)'); xlabel('Theta\_dot (rad/s)'); ylabel('Membership Degree'); grid on;

%% 4. Output: Force - Lực tác động lên xe
cpFIS.Outputs(1).Name = 'Force';
cpFIS.Outputs(1).Range = [-10, 10];

% Negative Medium: Lực vừa (-4 N)
cpFIS.Outputs(1).MembershipFunctions(1).Name = 'NM';
cpFIS.Outputs(1).MembershipFunctions(1).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(1).Parameters = [2.5, 2, -4];

% Positive Medium: Lực vừa (+4 N)
cpFIS.Outputs(1).MembershipFunctions(2).Name = 'PM';
cpFIS.Outputs(1).MembershipFunctions(2).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(2).Parameters = [2.5, 2, 4];

% Negative Large: Lực hãm mạnh (-10 N)
cpFIS.Outputs(1).MembershipFunctions(3).Name = 'NL';
cpFIS.Outputs(1).MembershipFunctions(3).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(3).Parameters = [3.0, 2, -10];

% Positive Large: Lực hãm mạnh (+10 N)
cpFIS.Outputs(1).MembershipFunctions(4).Name = 'PL';
cpFIS.Outputs(1).MembershipFunctions(4).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(4).Parameters = [3.0, 2, 10];

figure('Name', 'Output: Force');
plotmf(cpFIS, 'output', 1, 1000);
title('Output 1: Force (N)'); xlabel('Force (N)'); ylabel('Membership Degree'); grid on;

%% 5. Luật điều khiển (4 luật suy diễn cơ bản)
rules = [...
    "If Theta is Negative then Force is NM"; ...
    "If Theta is Positive then Force is PM"; ...
    "If Theta_dot is Negative then Force is NL"; ...
    "If Theta_dot is Positive then Force is PL"];

cpFIS = addRule(cpFIS, rules);

%% 6. Vẽ mặt điều khiển và xuất file .fis
figure('Name', 'Control Surface');
gensurf(cpFIS);
title('Mặt quan hệ điều khiển (Control Surface)');
xlabel('Theta (rad)'); ylabel('Theta\_dot (rad/s)'); zlabel('Force (N)');
grid on;

% Lưu biến cấu hình và file .fis cho Simulink
fis = cpFIS;
writeFIS(cpFIS, 'cartpole.fis');
fprintf('\n=======================================================\n');
fprintf('  Đã nạp xong bộ thông số 4 luật tối ưu!\n');
fprintf('  File "cartpole.fis" đã được lưu thành công.\n');
fprintf('=======================================================\n');
