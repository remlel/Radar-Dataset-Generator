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
display_trajectory(radarPlatform, TgtTrajectory, params);


