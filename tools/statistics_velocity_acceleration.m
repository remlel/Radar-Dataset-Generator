%% SCRIPT : Statistical Analysis of the Velocity & Acceleration

clear; clc; close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
datasetFolder = fullfile(projectRoot, '..', 'Datasets');

if ~exist(datasetFolder, 'dir')
    error('Dataset folder not found: %s', datasetFolder);
end

% 1. Loading dataset (.mat)
filename = fullfile(datasetFolder, 'Radar_Dataset_dt_0.1_1.mat');
if ~isfile(filename)
    error('The file %s cannot be found.', filename);
end
load(filename, 'dataset', 'dt');

% 2. Extracting velocities
Vx = dataset(:, :, 8);
Vy = dataset(:, :, 9);
Vz = dataset(:, :, 10);
V_mag = sqrt(Vx.^2 + Vy.^2 + Vz.^2);

% 3. Calculating accelerations
Ax = diff(Vx, 1, 2) / dt;
Ay = diff(Vy, 1, 2) / dt;
Az = diff(Vz, 1, 2) / dt;
A_mag = sqrt(Ax.^2 + Ay.^2 + Az.^2);

% 4. Cleaning for statistics
Vx = Vx(:);
Vx(isnan(Vx)) = [];
Vy = Vy(:);
Vy(isnan(Vy)) = [];
Vz = Vz(:);
Vz(isnan(Vz)) = [];
V_mag = V_mag(:);
V_mag(isnan(V_mag)) = [];

Ax = Ax(:);
Ax(isnan(Ax)) = [];
Ay = Ay(:);
Ay(isnan(Ay)) = [];
Az = Az(:);
Az(isnan(Az)) = [];
A_mag = A_mag(:);
A_mag(isnan(A_mag)) = [];

% 5. Statistics
V_var  = var(Vx) + var(Vy) + var(Vz);
V_std = sqrt(V_var);
V_mean = mean(V_mag);
V_max  = max(V_mag);

A_var  = var(Ax) + var(Ay) + var(Az);
A_std  = sqrt(A_var);
A_mean = mean(A_mag);
A_max  = max(A_mag); 

% 6. Display
fprintf('=== KINEMATIC STATISTICS ===\n');
fprintf('VELOCITY     : Std = %.2f m/s | Var = %.2f | Mean = %.2f m/s | Max = %.2f m/s\n', V_std, V_var, V_mean, V_max);
fprintf('ACCELERATION : Std = %.2f m/s² | Var = %.2f | Mean = %.2f m/s² | Max = %.2f m/s²\n', A_std, A_var, A_mean, A_max);