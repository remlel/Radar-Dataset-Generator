function [radarPlatform, TgtTrajectory, target] = init_scenario(params, patn)
    
    % 1. Radar (Fixed & Placed at origin)
    radarPlatform = struct('Position', [0;0;0], 'Velocity', [0;0;0]);

    % 2. Generating a valid scenario : waypoints and time of arrivals
    max_accel_allowed = 50;         % Max acceleration allowed (m/s²)
    is_valid = false;

    while ~is_valid
        
        % Generating waypoints and time of arrivals
        [waypoints, time_of_arrivals] = generate_random_waypoints(params, radarPlatform, patn);

        % Defining objects and target's trajectory
        TgtTrajectory = waypointTrajectory('SampleRate', params.Update_rate, 'Waypoints', waypoints, 'TimeOfArrival', time_of_arrivals);
    
        % Reading each accelerations
        [~, ~, ~, accel] = lookupPose(TgtTrajectory, 0:params.step:params.Total_time);
           
        % Magnitude of acceleration at each instant
        accel_magnitude = vecnorm(accel, 2, 2); 
        
        % Evaluating realism
        if max(accel_magnitude) <= max_accel_allowed
            is_valid = true;
        end
    end
        
    % 4. Radar Cross Section (RCS) 
    target = phased.RadarTarget('MeanRCS', 1, 'PropagationSpeed', params.c, 'OperatingFrequency', params.fc);

end