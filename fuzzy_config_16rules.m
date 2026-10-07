%% ============================================================
%%  FUZZY LOGIC CONTROLLER - CART-PENDULUM (HUST)
%%  Single Unified Mamdani FIS | 4 Inputs x 2 MFs = 16 Rules
%%  Copy toan bo file nay vao MATLAB va chay (F5)
%% ============================================================
clc; clear; close all;

%% ============================================================
%% PHAN 1: CAU HINH INPUTS / OUTPUTS
%% ============================================================

fis = mamfis('NumInputs', 4, 'NumInputMFs', 2, ...
             'NumOutputs', 1, 'NumOutputMFs', 4, ...
             'AddRule', 'none');

% ---- INPUT 1: Theta (goc nghieng, rad) ----
% Range: toan bo [-pi, pi]
% MF nhay trong vung [-0.20, 0.20] rad (~11.5 do)
fis.Inputs(1).Name            = 'Theta';
fis.Inputs(1).Range           = [-pi, pi];
fis.Inputs(1).MembershipFunctions(1).Name       = 'N';   % Am (nghieng trai)
fis.Inputs(1).MembershipFunctions(1).Type       = 'zmf';
fis.Inputs(1).MembershipFunctions(1).Parameters = [-0.20, 0.20];
fis.Inputs(1).MembershipFunctions(2).Name       = 'P';   % Duong (nghieng phai)
fis.Inputs(1).MembershipFunctions(2).Type       = 'smf';
fis.Inputs(1).MembershipFunctions(2).Parameters = [-0.20, 0.20];

% ---- INPUT 2: Theta_dot (van toc goc, rad/s) ----
% MF nhay trong vung [-3.5, 3.5] rad/s
fis.Inputs(2).Name            = 'Theta_dot';
fis.Inputs(2).Range           = [-15, 15];
fis.Inputs(2).MembershipFunctions(1).Name       = 'N';
fis.Inputs(2).MembershipFunctions(1).Type       = 'zmf';
fis.Inputs(2).MembershipFunctions(1).Parameters = [-3.5, 3.5];
fis.Inputs(2).MembershipFunctions(2).Name       = 'P';
fis.Inputs(2).MembershipFunctions(2).Type       = 'smf';
fis.Inputs(2).MembershipFunctions(2).Parameters = [-3.5, 3.5];

% ---- INPUT 3: X (vi tri xe, m) ----
% MF nhay trong vung [-0.25, 0.25] m
fis.Inputs(3).Name            = 'X';
fis.Inputs(3).Range           = [-0.5, 0.5];
fis.Inputs(3).MembershipFunctions(1).Name       = 'N';
fis.Inputs(3).MembershipFunctions(1).Type       = 'zmf';
fis.Inputs(3).MembershipFunctions(1).Parameters = [-0.25, 0.25];
fis.Inputs(3).MembershipFunctions(2).Name       = 'P';
fis.Inputs(3).MembershipFunctions(2).Type       = 'smf';
fis.Inputs(3).MembershipFunctions(2).Parameters = [-0.25, 0.25];

% ---- INPUT 4: X_dot (van toc xe, m/s) ----
% MF nhay trong vung [-0.5, 0.5] m/s
fis.Inputs(4).Name            = 'X_dot';
fis.Inputs(4).Range           = [-2, 2];
fis.Inputs(4).MembershipFunctions(1).Name       = 'N';
fis.Inputs(4).MembershipFunctions(1).Type       = 'zmf';
fis.Inputs(4).MembershipFunctions(1).Parameters = [-0.5, 0.5];
fis.Inputs(4).MembershipFunctions(2).Name       = 'P';
fis.Inputs(4).MembershipFunctions(2).Type       = 'smf';
fis.Inputs(4).MembershipFunctions(2).Parameters = [-0.5, 0.5];

% ---- OUTPUT: Force (luc day xe, N) ----
% 4 muc luc: NL=-10N, NM=-7N, PM=+7N, PL=+10N
fis.Outputs(1).Name  = 'Force';
fis.Outputs(1).Range = [-12, 12];

fis.Outputs(1).MembershipFunctions(1).Name       = 'NL';  % Am lon
fis.Outputs(1).MembershipFunctions(1).Type       = 'gbellmf';
fis.Outputs(1).MembershipFunctions(1).Parameters = [3.0, 2, -10];

fis.Outputs(1).MembershipFunctions(2).Name       = 'NM';  % Am vua
fis.Outputs(1).MembershipFunctions(2).Type       = 'gbellmf';
fis.Outputs(1).MembershipFunctions(2).Parameters = [2.5, 2, -7];

fis.Outputs(1).MembershipFunctions(3).Name       = 'PM';  % Duong vua
fis.Outputs(1).MembershipFunctions(3).Type       = 'gbellmf';
fis.Outputs(1).MembershipFunctions(3).Parameters = [2.5, 2, 7];

fis.Outputs(1).MembershipFunctions(4).Name       = 'PL';  % Duong lon
fis.Outputs(1).MembershipFunctions(4).Type       = 'gbellmf';
fis.Outputs(1).MembershipFunctions(4).Parameters = [3.0, 2, 10];

%% ============================================================
%% PHAN 2: 16 LUAT DIEU KHIEN (2^4 to hop)
%%
%% Logic chinh: Theta la yeu to quyet dinh chinh
%%   Theta=N (nghieng trai) -> day sang trai (luc am)
%%   Theta=P (nghieng phai) -> day sang phai (luc duong)
%%
%% Theta_dot bieu hien xu huong:
%%   Theta_dot cung chieu voi Theta -> khan cap (muc L)
%%   Theta_dot nguoc chieu Theta   -> tu hoi phuc (muc M)
%%
%% X, X_dot anh huong phu: lech vi tri lam giam muc luc
%% ============================================================
%
% Bang 16 luat: [Theta | Theta_dot | X | X_dot] -> Force
%
%  #  Theta  Theta_dot   X    X_dot  |  Force  |  Ly do
%  1    N       N        N     N     |   NL    |  Nguy hiem: goc am + quay am + lech trai + chay trai
%  2    N       N        N     P     |   NL    |  Goc am + quay am + dang quay tro lai -> van manh
%  3    N       N        P     N     |   NL    |  Goc am + quay am + lech phai -> van xu ly goc la chinh
%  4    N       N        P     P     |   NM    |  Goc am + quay am + lech phai + chay phai -> xe tu hoi phuc
%  5    N       P        N     N     |   NM    |  Goc am nhung dang quay ve -> giam luc
%  6    N       P        N     P     |   NM    |  Goc am, quay ve, xe cung dang tro lai
%  7    N       P        P     N     |   NM    |  Goc am, quay ve, vi tri o phai -> vua du
%  8    N       P        P     P     |   NM    |  Goc am, quay ve, xe phai + chay phai -> giu nhe
%  9    P       N        N     N     |   PM    |  Goc duong, dang quay ve -> luc vua
% 10    P       N        N     P     |   PM    |  Goc duong, quay ve, xe trai + chay phai
% 11    P       N        P     N     |   PM    |  Goc duong, quay ve, xe phai + dung yen
% 12    P       N        P     P     |   PM    |  Goc duong, quay ve, xe chay phai -> vua
% 13    P       P        N     N     |   PL    |  Nguy hiem: goc duong + quay duong + xe trai + dung yen
% 14    P       P        N     P     |   PL    |  Goc duong + quay duong + chay phai -> manh
% 15    P       P        P     N     |   PL    |  Goc duong + quay duong + xe phai -> manh
% 16    P       P        P     P     |   PL    |  Nguy hiem nhat: goc + quay + vi tri cung phai

rules = [
    "If Theta is N and Theta_dot is N and X is N and X_dot is N then Force is NL"
    "If Theta is N and Theta_dot is N and X is N and X_dot is P then Force is NL"
    "If Theta is N and Theta_dot is N and X is P and X_dot is N then Force is NL"
    "If Theta is N and Theta_dot is N and X is P and X_dot is P then Force is NM"
    "If Theta is N and Theta_dot is P and X is N and X_dot is N then Force is NM"
    "If Theta is N and Theta_dot is P and X is N and X_dot is P then Force is NM"
    "If Theta is N and Theta_dot is P and X is P and X_dot is N then Force is NM"
    "If Theta is N and Theta_dot is P and X is P and X_dot is P then Force is NM"
    "If Theta is P and Theta_dot is N and X is N and X_dot is N then Force is PM"
    "If Theta is P and Theta_dot is N and X is N and X_dot is P then Force is PM"
    "If Theta is P and Theta_dot is N and X is P and X_dot is N then Force is PM"
    "If Theta is P and Theta_dot is N and X is P and X_dot is P then Force is PM"
    "If Theta is P and Theta_dot is P and X is N and X_dot is N then Force is PL"
    "If Theta is P and Theta_dot is P and X is N and X_dot is P then Force is PL"
    "If Theta is P and Theta_dot is P and X is P and X_dot is N then Force is PL"
    "If Theta is P and Theta_dot is P and X is P and X_dot is P then Force is PL"
];

fis = addRule(fis, rules);

disp('=== FIS DA XAY DUNG XONG ===');
fprintf('So luat: %d\n', numel(fis.Rules));
disp(fis);

%% ============================================================
%% PHAN 3: VE DO THI MEMBERSHIP FUNCTIONS
%% ============================================================
figure('Name','MF - Inputs & Output','NumberTitle','off','Position',[50 50 1300 700]);

subplot(2,3,1); plotmf(fis,'input',1,500);
title('Input 1: Theta (rad)'); xlabel('rad'); grid on;

subplot(2,3,2); plotmf(fis,'input',2,500);
title('Input 2: Theta\_dot (rad/s)'); xlabel('rad/s'); grid on;

subplot(2,3,3); plotmf(fis,'input',3,500);
title('Input 3: X (m)'); xlabel('m'); grid on;

subplot(2,3,4); plotmf(fis,'input',4,500);
title('Input 4: X\_dot (m/s)'); xlabel('m/s'); grid on;

subplot(2,3,5); plotmf(fis,'output',1,500);
title('Output: Force (N)'); xlabel('N'); grid on;

subplot(2,3,6); gensurf(fis,[1 2],[],[],50);
title('Mat dieu khien: Theta vs Theta\_dot'); grid on;

%% ============================================================
%% PHAN 4: LUU FIS FILE (dung cho Simulink FLC block)
%% ============================================================
writeFIS(fis, 'cartpole_16rules.fis');
disp('Da luu: cartpole_16rules.fis  (dung cho Simulink Fuzzy Logic Controller block)');

%% ============================================================
%% PHAN 5: KIEM TRA NHANH - evalfis tai cac diem quan trong
%% ============================================================
disp(' ');
disp('=== KIEM TRA NHANH evalfis ===');
fprintf('%-30s -> Force = %+.2f N\n', 'Theta=+20deg, Thdot=+1 rad/s', ...
    evalfis(fis,[20*pi/180, 1, 0, 0]));
fprintf('%-30s -> Force = %+.2f N\n', 'Theta=-20deg, Thdot=-1 rad/s', ...
    evalfis(fis,[-20*pi/180, -1, 0, 0]));
fprintf('%-30s -> Force = %+.2f N\n', 'Theta=+5deg, Thdot=+0.5 rad/s', ...
    evalfis(fis,[5*pi/180, 0.5, 0, 0]));
fprintf('%-30s -> Force = %+.2f N\n', 'Theta=0, Thdot=0 (can bang)', ...
    evalfis(fis,[0, 0, 0, 0]));

disp(' ');
disp('=== TOM TAT CAU HINH ===');
disp('Inputs:');
disp('  1. Theta     : zmf/smf, nhay [-0.20, +0.20] rad');
disp('  2. Theta_dot : zmf/smf, nhay [-3.50, +3.50] rad/s');
disp('  3. X         : zmf/smf, nhay [-0.25, +0.25] m');
disp('  4. X_dot     : zmf/smf, nhay [-0.50, +0.50] m/s');
disp('Output:');
disp('  Force: NL=-10N, NM=-7N, PM=+7N, PL=+10N (gbellmf)');
disp('Rules: 16 luat day du (2^4 to hop)');
disp('  - Theta la yeu to quyet dinh chinh (trong so 1.0)');
disp('  - X, X_dot la yeu to phu (anh huong muc NM<->NL, PM<->PL)');
