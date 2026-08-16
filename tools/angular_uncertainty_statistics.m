%% SCRIPT : Statistical Analysis of the Angular Uncertainties (MUSIC)

clear; clc; close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
datasetFolder = fullfile(projectRoot, '..', 'Datasets');

if ~exist(datasetFolder, 'dir')
    error('Dataset folder not found: %s', datasetFolder);
end

% 1. Loading sigma dataset (.mat)
filename = fullfile(datasetFolder, 'SigmaAngle_Dataset__.mat');
if ~isfile(filename)
    error('The file %s cannot be found.', filename);
end
load(filename, 'data_sigmaAngle'); 

% 2. Extracting data 
sigma_az_raw = data_sigmaAngle(:, :, 1);
sigma_el_raw = data_sigmaAngle(:, :, 2);

% 3. Cleaning and reshaping (1D vector) data
sigma_az = sigma_az_raw(~isnan(sigma_az_raw));
sigma_el = sigma_el_raw(~isnan(sigma_el_raw));

% 4. Calculating Statistical Metrics
stats = struct();
stats.Az = [mean(sigma_az), median(sigma_az), std(sigma_az), prctile(sigma_az, 25), prctile(sigma_az, 75)];
stats.El = [mean(sigma_el), median(sigma_el), std(sigma_el), prctile(sigma_el, 25), prctile(sigma_el, 75)];

fprintf('=== Angular Uncertainties Statistics ===\n');
fprintf('AZIMUTH    : Mean = %.4f° | Median = %.4f° | Standard deviation = %.4f°\n', stats.Az(1), stats.Az(2), stats.Az(3));
fprintf('ELEVATION : Mean = %.4f° | Median = %.4f° | Standard deviation = %.4f°\n', stats.El(1), stats.El(2), stats.El(3));
fprintf('------------------------------------------------\n');

% 5. Visualization
figure('Name', 'Etude de l''Incertitude MUSIC', 'Position', [100, 100, 1000, 400]);

% --- Azimuth Histogram ---
subplot(1,2,1);
histogram(sigma_az, 'Normalization', 'probability', 'FaceColor', '#0072BD', 'EdgeColor', 'w');
hold on;
xline(stats.Az(2), 'r', 'LineWidth', 2, 'Label', sprintf('Median: %.2f°', stats.Az(2)));
title('Distribution of the Azimuth Uncertainties (\sigma_{Az})');
xlabel('Uncertainty (degrees)'); ylabel('Probability');
grid on;

% --- Elevation Histogram ---
subplot(1,2,2);
histogram(sigma_el, 'Normalization', 'probability', 'FaceColor', '#D95319', 'EdgeColor', 'w');
hold on;
xline(stats.El(2), 'r', 'LineWidth', 2, 'Label', sprintf('Median: %.2f°', stats.El(2)));
title('Distribution of the Elevation Uncertainties (\sigma_{El})');
xlabel('Uncertainty (degrees)'); ylabel('Probability');
grid on;