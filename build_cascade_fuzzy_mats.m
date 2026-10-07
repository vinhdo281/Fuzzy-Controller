%% Build Cascade Fuzzy Controllers for Cart-Pendulum (MATLAB / Simulink)
% Parameters matching HUST solid model: full_pendulum_assem_DataFile3.m
% Cart mass: M = 0.0591 kg, Pole mass: m = 0.0196 kg, L_com = 0.1106 m

clear; clc;

fprintf('=======================================================\n');
fprintf('  Building Cascade Fuzzy Controller for HUST Cart-Pole \n');
fprintf('=======================================================\n');

%% 1. INNER LOOP FIS: Angle Stabilization (theta, theta_dot -> Force)
innerFIS = mamfis('Name', 'cartpole_inner_angle', ...
                  'AndMethod', 'min', ...
                  'OrMethod', 'max', ...
                  'ImplicationMethod', 'min', ...
                  'AggregationMethod', 'max', ...
                  'DefuzzificationMethod', 'centroid');

% Input 1: e_theta (rad) normalized range [-0.15, 0.15]
innerFIS = addInput(innerFIS, [-0.15 0.15], 'Name', 'e_theta');
innerFIS = addMF(innerFIS, 'e_theta', 'trapmf', [-0.25 -0.15 -0.08 -0.03], 'Name', 'NB');
innerFIS = addMF(innerFIS, 'e_theta', 'trimf',  [-0.08 -0.04 0],            'Name', 'NS');
innerFIS = addMF(innerFIS, 'e_theta', 'trimf',  [-0.03 0 0.03],             'Name', 'ZE');
innerFIS = addMF(innerFIS, 'e_theta', 'trimf',  [0 0.04 0.08],              'Name', 'PS');
innerFIS = addMF(innerFIS, 'e_theta', 'trapmf', [0.03 0.08 0.15 0.25],     'Name', 'PB');

% Input 2: de_theta (rad/s) normalized range [-1.5, 1.5]
innerFIS = addInput(innerFIS, [-1.5 1.5], 'Name', 'de_theta');
innerFIS = addMF(innerFIS, 'de_theta', 'trapmf', [-2.5 -1.5 -0.8 -0.3], 'Name', 'NB');
innerFIS = addMF(innerFIS, 'de_theta', 'trimf',  [-0.8 -0.4 0],          'Name', 'NS');
innerFIS = addMF(innerFIS, 'de_theta', 'trimf',  [-0.3 0 0.3],           'Name', 'ZE');
innerFIS = addMF(innerFIS, 'de_theta', 'trimf',  [0 0.4 0.8],            'Name', 'PS');
innerFIS = addMF(innerFIS, 'de_theta', 'trapmf', [0.3 0.8 1.5 2.5],     'Name', 'PB');

% Output: Force (N) range [-3.0, 3.0]
innerFIS = addOutput(innerFIS, [-3.0 3.0], 'Name', 'Force');
innerFIS = addMF(innerFIS, 'Force', 'trapmf', [-4.0 -3.0 -1.8 -0.8], 'Name', 'NB');
innerFIS = addMF(innerFIS, 'Force', 'trimf',  [-1.8 -0.8 0],         'Name', 'NS');
innerFIS = addMF(innerFIS, 'Force', 'trimf',  [-0.5 0 0.5],          'Name', 'ZE');
innerFIS = addMF(innerFIS, 'Force', 'trimf',  [0 0.8 1.8],           'Name', 'PS');
innerFIS = addMF(innerFIS, 'Force', 'trapmf', [0.8 1.8 3.0 4.0],     'Name', 'PB');

% Rule Base: MacVicar-Whelan 25 rules
ruleListInner = [
    1 1 1 1 1; 1 2 1 1 1; 1 3 1 1 1; 1 4 2 1 1; 1 5 3 1 1;
    2 1 1 1 1; 2 2 1 1 1; 2 3 2 1 1; 2 4 3 1 1; 2 5 4 1 1;
    3 1 1 1 1; 3 2 2 1 1; 3 3 3 1 1; 3 4 4 1 1; 3 5 5 1 1;
    4 1 2 1 1; 4 2 3 1 1; 4 3 4 1 1; 4 4 5 1 1; 4 5 5 1 1;
    5 1 3 1 1; 5 2 4 1 1; 5 3 5 1 1; 5 4 5 1 1; 5 5 5 1 1;
];
innerFIS = addRule(innerFIS, ruleListInner);
writeFIS(innerFIS, 'cartpole_inner_angle.fis');
fprintf('Created and saved cartpole_inner_angle.fis\n');


%% 2. OUTER LOOP FIS: Cart Position Control (e_x, de_x -> theta_ref)
outerFIS = mamfis('Name', 'cartpole_outer_pos', ...
                  'AndMethod', 'min', ...
                  'OrMethod', 'max', ...
                  'ImplicationMethod', 'min', ...
                  'AggregationMethod', 'max', ...
                  'DefuzzificationMethod', 'centroid');

% Input 1: e_x = x_ref - x (m) range [-0.3, 0.3]
outerFIS = addInput(outerFIS, [-0.3 0.3], 'Name', 'e_x');
outerFIS = addMF(outerFIS, 'e_x', 'trapmf', [-0.5 -0.3 -0.15 -0.05], 'Name', 'NB');
outerFIS = addMF(outerFIS, 'e_x', 'trimf',  [-0.15 -0.08 0],          'Name', 'NS');
outerFIS = addMF(outerFIS, 'e_x', 'trimf',  [-0.05 0 0.05],           'Name', 'ZE');
outerFIS = addMF(outerFIS, 'e_x', 'trimf',  [0 0.08 0.15],            'Name', 'PS');
outerFIS = addMF(outerFIS, 'e_x', 'trapmf', [0.05 0.15 0.3 0.5],     'Name', 'PB');

% Input 2: de_x = -x_dot (m/s) range [-0.5, 0.5]
outerFIS = addInput(outerFIS, [-0.5 0.5], 'Name', 'de_x');
outerFIS = addMF(outerFIS, 'de_x', 'trapmf', [-0.8 -0.5 -0.25 -0.08], 'Name', 'NB');
outerFIS = addMF(outerFIS, 'de_x', 'trimf',  [-0.25 -0.12 0],          'Name', 'NS');
outerFIS = addMF(outerFIS, 'de_x', 'trimf',  [-0.08 0 0.08],           'Name', 'ZE');
outerFIS = addMF(outerFIS, 'de_x', 'trimf',  [0 0.12 0.25],            'Name', 'PS');
outerFIS = addMF(outerFIS, 'de_x', 'trapmf', [0.08 0.25 0.5 0.8],     'Name', 'PB');

% Output: theta_ref (rad) range [-0.08, 0.08] rad (~ +/- 4.6 deg)
outerFIS = addOutput(outerFIS, [-0.08 0.08], 'Name', 'theta_ref');
outerFIS = addMF(outerFIS, 'theta_ref', 'trapmf', [-0.12 -0.08 -0.05 -0.02], 'Name', 'NB');
outerFIS = addMF(outerFIS, 'theta_ref', 'trimf',  [-0.05 -0.025 0],           'Name', 'NS');
outerFIS = addMF(outerFIS, 'theta_ref', 'trimf',  [-0.015 0 0.015],           'Name', 'ZE');
outerFIS = addMF(outerFIS, 'theta_ref', 'trimf',  [0 0.025 0.05],             'Name', 'PS');
outerFIS = addMF(outerFIS, 'theta_ref', 'trapmf', [0.02 0.05 0.08 0.12],     'Name', 'PB');

% Rule Base: MacVicar-Whelan 25 rules (Lean-to-steer)
ruleListOuter = ruleListInner; % Same standard anti-symmetric rule structure
outerFIS = addRule(outerFIS, ruleListOuter);
writeFIS(outerFIS, 'cartpole_outer_pos.fis');
fprintf('Created and saved cartpole_outer_pos.fis\n');

fprintf('Successfully built both FIS files for Simulink / MATLAB integration!\n');
