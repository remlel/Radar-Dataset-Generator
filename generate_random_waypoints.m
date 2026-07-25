function [waypoints, time_of_arrivals] = generate_random_waypoints(params, radarPlatform, patn)

    % 1. Constraints
    min_wp = 3;                                                     % Min number of waypoints
    max_wp = 7;                                                     % Max number of waypoints
    R_max00 = 150;                                                  % Max detection range on the boresight axis
    V_max = 35;                                                     % Max target velocity
    G_az_el = patn.RmaxOverRmax00_vec * R_max00;                 % Max range vector : azimuth & elevation dependency 
    R_min = 10;                                                     % Min range to radar                                           
    Angle_max = patn.max_angle;                                  % Max angle (azimuth & elevation) from boresight axis

    % 2. Generating the number of intervals
    N_waypoints = randi([min_wp, max_wp]);
    time_of_arrivals = zeros(1, N_waypoints);
    waypoints = zeros(N_waypoints, 3);

    % 3. Generating first point
    pt1 = get_random_starting_point(radarPlatform.Position, R_min, Angle_max, G_az_el, patn);
    waypoints(1, :) = pt1;
    time_of_arrivals(1, 1) = params.step;
    
    % 4. Defining running parameters for waypoints calculation
    mean_travel_time = (params.Total_time-params.step)/(N_waypoints-1);
    time_deviation = mean_travel_time * 0.4;

    % 5. Loop calculating next waypoints until penultimate one
    for i = 1:(N_waypoints-2)
        
        % Generating next way point
        travel_time = round((mean_travel_time-time_deviation) + 2*time_deviation*rand(),3);
        R_step_max = V_max * travel_time; 
        R_step_min = min(3, R_step_max * 0.2);
        prev_pt = waypoints(i, :);
        pt = get_random_point_in_cone(prev_pt, R_step_min, R_step_max, 180);
        
        % Checking if point is in the radar area
        bool = verif_point(pt, radarPlatform, G_az_el, patn);
        
        % Generating new points until it is in the radar area
        while bool == 1
            pt = get_random_point_in_cone(prev_pt, R_step_min, R_step_max, 180);
            bool = verif_point(pt, radarPlatform, G_az_el, patn);
        end
        
        % Adding generated point to waypoints vector
        waypoints(i+1, :) = pt;
        time_of_arrivals(1, i+1) = time_of_arrivals(1, i) + travel_time;

        % Updating mean travel time and time deviation -> avoid strong discrepencies (for the final travel time)
        time_left = params.Total_time - time_of_arrivals(1, i+1);
        intervals_left = N_waypoints-1-i;
        mean_travel_time = time_left / intervals_left;
        time_deviation = mean_travel_time * 0.4;

    end
    
    % 6. Generating final point
    prev_pt = waypoints(end-1, :);
    R_step_max = V_max * mean_travel_time;      % mean_travel_time = remaining time to total time
    pt = get_random_point_in_cone(prev_pt, R_step_min, R_step_max, 180);
    bool = verif_point(pt, radarPlatform, G_az_el, patn);
        
    while bool == 1
        pt = get_random_point_in_cone(prev_pt, R_step_min, R_step_max, 180);
        bool = verif_point(pt, radarPlatform, G_az_el, patn);
    end

    % 7. Adding final generated point to waypoints vector
    waypoints(N_waypoints, :) = pt;
    time_of_arrivals(1, N_waypoints) = params.Total_time;

end


function pt = get_random_starting_point(vector, R_min, Angle_max, G_az_el, patn)

    % 1. Generates random angles
    az = -Angle_max + rand() * (2 * Angle_max);
    el = -Angle_max + rand() * (2 * Angle_max);

    % 2. Generates a random authorised distance to radar
    R_max = interp2(patn.az_vec, patn.el_vec, G_az_el, az, el);
    R = R_min + rand() * (R_max - R_min);
    
    pt = calculate_point(vector, R, az, el);
end

function pt = get_random_point_in_cone(vector, R_min, R_max, Angle_max)
    
    % 1. Generates a random authorised distance to radar
    R = R_min + rand() * (R_max - R_min);
    
    % Same for both angles
    az = -Angle_max + rand() * (2 * Angle_max);
    el = -Angle_max + rand() * (2 * Angle_max);
    
    pt = calculate_point(vector, R, az, el);
end

function pt = calculate_point(vector, R, az, el)

    % Extracting coordonates
    x0 = vector(1);
    y0 = vector(2);
    z0 = vector(3);

    % Conversion Spherical -> Cartesian (X axis = Boresight of radar)
    x = x0 + R * cosd(el) * cosd(az);
    y = y0 + R * cosd(el) * sind(az);
    z = z0 + R * sind(el);
    
    pt = [x, y, z];
end

function bool = verif_point(pt, radarPlatform, G_az_el, patn)

    % Checks if the input point is in the radar area

    [range, angle] = rangeangle(pt', radarPlatform.Position);

    az = angle(1);
    el = angle(2);
    
    R_max = interp2(patn.az_vec, patn.el_vec, G_az_el, az, el);

    if isnan(R_max) || range > R_max
        bool = 1; % Rejected point
    else
        bool = 0; % Accepted point
    end
end