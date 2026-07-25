function [radarPlatform, TgtTrajectory, target] = init_scenario(params, patn)

    % Generation of a detection scenario respecting tow constraints :
    %   -> time constraint : duration = 15 s
    %   -> spatial constraint : 45° angle from x axis & maximum range of 470 m form radar
    
    % 1. Radar (Fixed & Placed at origin)
    radarPlatform = struct('Position', [0;0;0], 'Velocity', [0;0;0]);

    % 2. Generating waypoints and time of arrivals
    [waypoints, time_of_arrivals] = generate_random_waypoints(params, radarPlatform, patn);

    % 3. Defining objects and target's trajectory
    TgtTrajectory = waypointTrajectory('SampleRate', params.Update_rate, 'Waypoints', waypoints, 'TimeOfArrival', time_of_arrivals);
    
    % 4. Radar Cross Section (RCS) 
    target = phased.RadarTarget('MeanRCS', 1, 'PropagationSpeed', params.c, 'OperatingFrequency', params.fc);

end