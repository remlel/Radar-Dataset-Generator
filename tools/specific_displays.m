clear; clc; close all;

[hardware, software, params, patn] = config_radar();
[radarPlatform, TgtTrajectory, target] = init_scenario(params, patn);

%---------------------------------------------%

% 1. Displaying antenna URA
%viewArray(hardware.array)

%---------------------------------------------%

% 2. Displaying radiation pattern

% 2.1 Element
%pattern(hardware.element, params.fc);

% 2.2 Array
%pattern(hardware.array, params.fc);
%hold on

R = 50;
phi = linspace(0,2*pi,300);
theta = deg2rad(45);

x = R*cos(theta)*ones(size(phi));
y = R*sin(theta).*cos(phi);
z = R*sin(theta).*sin(phi);

%plot3(x,y,z,'r','LineWidth',3)

% 2.3 Azimuth cut
%patternAzimuth(hardware.array, params.fc)

%---------------------------------------------%

% 3. Displaying generated trajectory
display_trajectory(radarPlatform, TgtTrajectory, params, patn,software);

%---------------------------------------------%

% 4. Evaluating the realism of the generated scenarios

max_accel_allowed = 50;         % Max acceleration allowed (m/s²)
valid_scenarios = 0;            % Initiating counter of valid scenarios
number_scenarios = 0;          % Initiating counter of scenarios
loop = false;

if loop == true
    for i = 1:1000
        
        % Generating waypoints
        [waypoints, time_of_arrivals] = generate_random_waypoints(params, radarPlatform, patn);
        
        % Creating trajectory
        traj = waypointTrajectory(waypoints, time_of_arrivals);
        
        % Reading each accelerations
        [~, ~, ~, accel] = lookupPose(traj, 0:params.step:params.Total_time);
           
        % Magnitude of acceleration at each instant
        accel_magnitude = vecnorm(accel, 2, 2); 
        
        % Evaluating realism
        if max(accel_magnitude) <= max_accel_allowed
            valid_scenarios = valid_scenarios + 1;
        end
    
        number_scenarios = number_scenarios + 1;
    end
    
    fprintf('Ratio of valid scenarios : %.1f%%\n', 100*(valid_scenarios/number_scenarios));
end
