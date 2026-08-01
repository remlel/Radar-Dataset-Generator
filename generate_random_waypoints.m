function [waypoints, time_of_arrivals] = generate_random_waypoints(params, radarPlatform, patn)

    % 1. Constraints (User Inputs)
    min_wp = 3;                                                     % Min number of waypoints
    max_wp = 7;                                                     % Max number of waypoints
    V_max = 35;                                                     % Max target velocity
    G_az_el = patn.RmaxOverRmax00_vec * patn.R_max00;               % Max range vector : azimuth & elevation dependency 
    R_min = 10;                                                     % Min range to radar
    Angle_max = 180;                                                % Half-angle of the cone for the next point

    % 2. Generating the number of intervals
    N_waypoints = randi([min_wp, max_wp]);
    time_of_arrivals = zeros(1, N_waypoints);
    waypoints = zeros(N_waypoints, 3);

    % 3. Generating first point
    pt1 = get_random_starting_point(radarPlatform.Position, R_min, G_az_el, patn);
    waypoints(1, :) = pt1;
    time_of_arrivals(1, 1) = params.step;
    
    % 4. Defining running parameters for waypoints calculation
    mean_travel_time = (params.Total_time-params.step)/(N_waypoints-1);
    time_deviation = mean_travel_time * 0.4;

    % 5. Loop calculating next waypoints until penultimate one
    for i = 1:(N_waypoints-2)
        
        % Generating next waypoint's parameters
        travel_time = round((mean_travel_time-time_deviation) + 2*time_deviation*rand(),3);
        R_step_max = V_max * travel_time; 
        R_step_min = min(3, R_step_max * 0.2);
        prev_pt = waypoints(i, :);
       
        % Generating new points until it is in the radar area
        bool = 0;
        while bool == 0
            if i == 1
                heading = randn(1, 3);
            else
                heading = waypoints(i, :) - waypoints(i-1, :);
            end
            pt = get_random_point_in_motion_cone(prev_pt, heading, R_step_min, R_step_max, Angle_max);
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
    R_step_min = min(3, R_step_max * 0.2);
    
    bool = 0;
    while bool == 0
        heading = waypoints(end-1, :) - waypoints(end-2, :);
        pt = get_random_point_in_motion_cone(prev_pt, heading, R_step_min, R_step_max, Angle_max);
        bool = verif_point(pt, radarPlatform, G_az_el, patn);
    end

    % 7. Adding final generated point to waypoints vector
    waypoints(N_waypoints, :) = pt;
    time_of_arrivals(1, N_waypoints) = params.Total_time + 0.01;

end


function pt = get_random_starting_point(Pos_Init, R_min, G_az_el, patn)

    % 1. Generates random angles
    az = patn.az_min + rand() * (patn.az_max-patn.az_min);
    el = patn.el_min + rand() * (patn.el_max-patn.el_min);

    % 2. Generates a random authorised distance to radar
    R_max = interp2(patn.az_vec, patn.el_vec, G_az_el, az, el);
    R = R_min + rand() * (R_max - R_min);
    
    extension = calculate_extension(R, az, el);

    pt = Pos_Init(:)' + extension;
end

function pt = get_random_point_in_motion_cone(current_pt, heading, R_min, R_max, Angle_max)
    
    % 1. Generates a random authorised distance
    R = R_min + rand() * (R_max - R_min);
    
    % 2. Local angles (Local X axis is the heading)
    az = -Angle_max + rand() * (2 * Angle_max);
    el = -Angle_max + rand() * (2 * Angle_max);
    
    % 3. Local vector (in the direction of motion)
    extension_local = calculate_extension(R, az, el);
    
    % 4. Normalize heading vector (Base X axis)
    u = heading / norm(heading);
    
    % 5. Create orthonormal basis (Rotation matrix)
    if abs(u(1)) < 0.9          
        tmp = [1, 0, 0];
    else                    % Finding an arbitrary vector not parallel to u (=> build the Y and Z axes)
        tmp = [0, 1, 0];
    end
    v = cross(u, tmp);     % Vector orthogonal to u
    v = v / norm(v);       % Local Y axis (after normalization)
    w = cross(u, v);       % Local Z axis
   
    % 6. Apply rotation and translate to current point
    Rot = [u; v; w];                % Transformation matrix from local to global
    extension_global = extension_local * Rot;
    pt = current_pt + extension_global;
end

function extension = calculate_extension(R, az, el)

    x_ex = R * cosd(el) * cosd(az);
    y_ex = R * cosd(el) * sind(az);
    z_ex = R * sind(el);

    extension = [x_ex, y_ex, z_ex];
end

function bool = verif_point(pt, radarPlatform, G_az_el, patn)

    % Checks if the input point is in the radar area

    [range, angle] = rangeangle(pt', radarPlatform.Position);

    az = angle(1);
    el = angle(2);
    
    R_max = interp2(patn.az_vec, patn.el_vec, G_az_el, az, el);

    if isnan(R_max) || range > R_max
        bool = 0;  % Rejected point
    else
        bool = 1;  % Accepted point
    end
end