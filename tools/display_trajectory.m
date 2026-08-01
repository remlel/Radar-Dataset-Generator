function display_trajectory(radarPlatform, TgtTrajectory, params, patn, software)
    
    % Defining parameters & vector
    t_sim = params.step : params.step : params.Total_time;
    num_pts = length(t_sim);
    pos_history = zeros(3, num_pts);
    
    for k = 1:num_pts
        [pos, ~, ~] = TgtTrajectory();
        pos_history(:, k) = pos';
    end
    
    % Fetching waypoints
    WPs = TgtTrajectory.Waypoints; 
    
    % 3. 3D Display
    figure('Name', 'Trajectory 3D', 'Color', 'w');
    hold on; grid on; axis equal;
    
    % Plot of the radar
    scatter3(radarPlatform.Position(1), radarPlatform.Position(2), radarPlatform.Position(3), ...
        100, '^r', 'filled', 'DisplayName', 'Radar');
    
    % Plot of the Waypoints generated
    scatter3(WPs(:,1), WPs(:,2), WPs(:,3), 50, 'ok', 'filled', 'DisplayName', 'Waypoints');
    
    % Plot of the trajectory
    plot3(pos_history(1,:), pos_history(2,:), pos_history(3,:), '-b', 'LineWidth', 1.5, ...
        'DisplayName', 'Trajectory (Splines)');
    
    % Plot of the FOV 
    R_max = patn.R_max00 * 0.7;

    % Azimuth limits
    plot3([0, R_max*cosd(patn.az_max + software.scan_margin)], ...
          [0, R_max*sind(patn.az_max + software.scan_margin)], ...
          [0, 0], '--r', 'HandleVisibility', 'off');
    
    plot3([0, R_max*cosd(patn.az_min - software.scan_margin)], ...
          [0, R_max*sind(patn.az_min - software.scan_margin)], ...
          [0, 0], '--r', 'DisplayName', 'Limits FOV Azimuth');
    
    % Elevation limits
    plot3([0, R_max*cosd(patn.el_max + software.scan_margin)], ...
          [0, 0], ...
          [0, R_max*sind(patn.el_max + software.scan_margin)], ...
          '--r', 'HandleVisibility', 'off');
    
    plot3([0, R_max*cosd(patn.el_min - software.scan_margin)], ...
          [0, 0], ...
          [0, R_max*sind(patn.el_min - software.scan_margin)], ...
          '--r', 'DisplayName', 'Limits FOV Elevation');
    
    % Labels
    xlabel('X axis - Boresight (m)');
    ylabel('Y axis - Cross-range (m)');
    zlabel('Z axis - Elevation (m)');
    title('Display of randomly generated trajectory of the target');
    legend('Location', 'best');
    view(3); 

end