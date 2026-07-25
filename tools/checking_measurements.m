function checking_measurements(dataset, data_check)

    figure('Name', 'Dataset Verification');
    
    % Range Display
    subplot(2,2,1);
    plot(data_check(:, 1), 'LineWidth', 2, 'DisplayName', 'Ground Truth (Reel Range)');
    hold on;
    plot(dataset(:, 1), 'x', 'DisplayName', 'Radar Measurement');
    title('Target Range through time');
    xlabel('Step (Time)'); ylabel('Range (m)');
    legend; grid on;
    
    % Velocity Display
    subplot(2,2,2);
    plot(data_check(:, 2), 'LineWidth', 2, 'DisplayName', 'Ground Truth (Reel Speed)');
    hold on;
    plot(dataset(:, 2), 'x', 'DisplayName', 'Radar Measurement');
    title('Radial Velocity through time');
    xlabel('Step (Time)'); ylabel('Velocity (m/s)');
    legend; grid on;
    
    % Azimuth Display
    subplot(2,2,3);
    plot(data_check(:, 3), 'LineWidth', 2, 'DisplayName', 'Ground Truth (Reel Azimuth)');
    hold on;
    plot(dataset(:, 3), 'x', 'DisplayName', 'Radar Measurement');
    title('Azimuth through time');
    xlabel('Step (Time)'); ylabel('Azimuth (°)');
    legend; grid on;
    
    % Elevation Display
    subplot(2,2,4);
    plot(data_check(:, 4), 'LineWidth', 2, 'DisplayName', 'Ground Truth (Reel Elevation)');
    hold on;
    plot(dataset(:, 4), 'x', 'DisplayName', 'Radar Measurement');
    title('Elevation through time');
    xlabel('Step (Time)'); ylabel('Elevation (°)');
    legend; grid on;

end