%% SIMULATION LOOP

clear; clc; close all;

% Creating path towards sub-folders
projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot));


%=====================================%
%         USER'S CHOICES
%=====================================%

% Dictates number of scenarios simulated :
Num_scenarios = 50;

% Launchs measurements checking :
Checking = false;
Num_checking = min(3, Num_scenarios);            % Indicates how many scenarios will be checked

% Launchs calculation of angular standard deviation :
Sigma = false;  
Num_sigma = min(30, Num_scenarios);              % Indicates for how many scenarios standard deviations will be calculated
                            
% Plots RD map :            
Map = false;                                     % If false, no map will be plotted
Num_scenarios_map = min(2, Num_scenarios);       % Indicates the number of scenarios where (a) map(s) will be plotted
Num_maps_scenario = 3;                           % Indicates for a given scenario the number of plotted maps

% Saves Data :
saving = true;                                   % If false, no data will be saved

%=====================================%


% 1. Radar Configuration 
[hardware, software, params, patn] = config_radar();

% 2. Creating datasets
dataset = zeros(Num_scenarios, params.Num_step, 10);              % Number scenarios * Number of RD maps per scenario * (Number measurements [r, v, az, el] + Number state groundtruth [x,y,z, vx,vy,vz]) 
if Checking == true
    data_check = zeros(Num_checking, params.Num_step, 4);          % // * [r_real, v_real, az_real, el_real]
end
if Sigma == true    
    data_sigmaAngle = zeros(Num_sigma, params.Num_step, 2);        % // * [sigmaAz, sigmaEl]
end

% 3. Initiating Counter
Num_no_detection = 0;
Num_total_measures = Num_scenarios * params.Update_rate * params.Total_time;

% 4. Scenario loop
disp('Loop of scenarios launched !')
for s = 1 : Num_scenarios

    % 1. Initialization of scenario
    [radarPlatform, TgtTrajectory, target] = init_scenario(params, patn);
    radarPos = radarPlatform.Position; 
    radarVel = radarPlatform.Velocity;
    
    %% MACROSCOPIC LOOP : 1 SCENARIO
    
    for idx = 1:params.Num_step                        
    
        % --- RESET OF HARDWARE BUFFERS ---
        release(hardware.channel);
        release(hardware.waveform);
        release(hardware.transmitter);
        release(hardware.radiator);
        release(hardware.collector);
        release(hardware.receiver);
    
        % 1. Initialization of data matrix (Fast-Time x Slow-Time)        
        numSamples = round(params.sweepTime * params.fs);                           % for 1 Chirp
        rxData = zeros(numSamples, params.numChirps, hardware.num_elements);        % discretization : range / speed / array
        
        % 2. Updating Target's State
        [tgtPos, ~, tgtVel] = TgtTrajectory();
        tgtPos = tgtPos';
        tgtVel = tgtVel';
    
        % 3. Capturing ground truth (State + Measurements (Checking)) 
        x = tgtPos(1);
        y = tgtPos(2);
        z = tgtPos(3);
        vx = tgtVel(1);
        vy = tgtVel(2);
        vz = tgtVel(3);
    
        if Checking == true && s <= Num_checking
            [ch_range, ch_ang] = rangeangle(tgtPos, radarPos);                  % Range and Angles towards target in the radar referential 
            ch_tgtDir = (radarPos - tgtPos) / norm(radarPos - tgtPos);          % Directional vector tgt -> radar  
            ch_speed = dot(tgtVel - radarVel, ch_tgtDir);                       % Absolute radial velocity (positive velocity = rapprochement)
        end
    
        % 4. Loop on impulsions (Chirps) -> build 1 frame (1 RD map) / Variation of target's state neglected
        for i = 1:params.numChirps
            
            % 4.1 Updating locally target's position
            dt_chirp = (i - 1) * params.sweepTime;
            tgtPos_chirp = tgtPos + (tgtVel * dt_chirp);
    
            % 4.2 Calculation of radar/target angles  (simulation parameter)
            [~, tgtAng] = rangeangle(tgtPos_chirp, radarPos);
            
            % 4.3 Generation of emission signal
            txSig = hardware.waveform();
            txSigAmp = hardware.transmitter(txSig);
            txSigRad = hardware.radiator(txSigAmp, tgtAng);
            
            % 4.4 Propagation towards target 
            rxSigCh = hardware.channel(txSigRad, radarPos, tgtPos_chirp, radarVel, tgtVel);
            
            % 4.5 Reflexion on target
            rxSigTgt = target(rxSigCh);
            
            % 4.6 Hardware reception 
            rxSigCol = hardware.collector(rxSigTgt, tgtAng);
            rxSig = hardware.receiver(rxSigCol);
            
            % 4.7 Dechirping : Mixing of received signal and emitted signal  -> Baseband conversion
            rxData(:, i, :) = dechirp(rxSig, txSig);
        
        end
        
        %% SIGNAL PROCESSING
        
        % 1. Créating Range Doppler Map
        rxData_Antenna1 = rxData(:, :, 1);                                     
        [resp1, rngGrid, dopGrid] = software.rngDopResp(rxData_Antenna1);    
        if Map == true && idx <= Num_maps_scenario && s <= Num_scenarios_map  
            plot_range_doppler_map(resp1, rngGrid, dopGrid);
        end
    
        % 2. Obtaining responses for each antenna
        resp = zeros(numSamples, params.numChirps, hardware.num_elements);
        for k = 1:hardware.num_elements
            resp(:, :, k) = software.rngDopResp(rxData(:, :, k));
        end
        
        % 3. Target Detection 
        [row, col] = target_detection_cfar(software, resp(:, :, 1), idx);    % Only on Antenna 1 : distance between antennas neglected
        
        % 4. Retrieval of measurements upon detection
        if ~(isempty(row) || isempty(col))
    
            % ---| Exctraction of ROI (Antenna 1) |---
            %roi = extract_roi(resp1, row, col);
            
            % 4.1 Obtaining Distance & Speed
            range = rngGrid(row);
            speed = dopGrid(col);
            
            % 4.2 DOA : Obtaining spatial snapshot (antennas) + MUSIC
            snapshot = reshape(resp(row, col, :), 1, hardware.num_elements);           
            [spectrum, doa] = software.estimator(snapshot);
            Az = -doa(1);
            El = -doa(2);
            if Sigma == true && s <= Num_sigma
                [sigmaAz, sigmaEl] = estimate_uncertainty_doa(spectrum);
            end
    
        end
       
        %% STORAGE & UPDATING
        
        % 1. Populating the dataset: measurements
        if isempty(row) || isempty(col)
            
            % 1.1 Target not detected
            dataset(s, idx, 1:4) = NaN;       
            if Sigma == true && s <= Num_sigma
                data_sigmaAngle(s, idx, 1:2) = NaN;
            end
            Num_no_detection = Num_no_detection + 1;
    
        else
    
            % 1.2 Target detected
            dataset(s, idx, 1) = range;
            dataset(s, idx, 2) = speed;
            dataset(s, idx, 3) = Az;
            dataset(s, idx, 4) = El;
            
            % 1.3 Retrieval of angles standard deviation if desired
            if Sigma == true && s <= Num_sigma
                data_sigmaAngle(s, idx, 1) = sigmaAz;
                data_sigmaAngle(s, idx, 2) = sigmaEl;
            end
    
        end
    
        % 2. Populating the dataset: state ground truth        
        dataset(s, idx, 5) = x;
        dataset(s, idx, 6) = y;
        dataset(s, idx, 7) = z;
        dataset(s, idx, 8) = vx;
        dataset(s, idx, 9) = vy;
        dataset(s, idx, 10) = vz;
        
        % 3. Populating the dataset: checking
        if Checking == true && s <= Num_checking
            data_check(s, idx, 1) = ch_range;
            data_check(s, idx, 2) = ch_speed;
            data_check(s, idx, 3) = ch_ang(1);
            data_check(s, idx, 4) = ch_ang(2);
        end
    
    end

    % Regular display of simulation advancement 
    if mod(s, 10) == 0
        fprintf('Generated scenarios : %d / %d\n', s, Num_scenarios);
    end

end

% 1. Lauching checking if desired :
if Checking == true
    for i = 1:Num_checking
        checking_measurements(squeeze(dataset(i, :, :)), squeeze(data_check(i, :, :)));
    end
end

% 2. Launching saving if desired :
if saving == true

    % 1. Saving dataset
    datasetFolder = fullfile(projectRoot,'..','Datasets');
    if ~exist(datasetFolder, 'dir')
        mkdir(datasetFolder);
        fprintf('Created dataset folder: %s\n', datasetFolder);
    end

    % 1.1 Base file name
    dt_str = num2str(params.step);
    base_filename = sprintf('Radar_Dataset_dt_%s', dt_str);
    
    % 1.2 Iterating until finding available file name
    idx = 1;
    while true
        % Generating potential names
        filename_csv = fullfile(datasetFolder, sprintf('%s_%d.csv', base_filename, idx));
        filename_mat = fullfile(datasetFolder, sprintf('%s_%d.mat', base_filename, idx));
       
        if ~exist(filename_csv, 'file') && ~exist(filename_mat, 'file')
            break; 
        end
        idx = idx + 1; 
    end

    % 1.3 .csv format
    dataset_2D = reshape(permute(dataset, [2 1 3]), [], 10);
    writematrix(dataset_2D, filename_csv);
    
    % 1.4 .mat format
    dt = params.step; 
    save(filename_mat, 'dataset', 'dt');
    
    fprintf('Dataset saved !\n');
    
    % 2. Saving Sigma dataset 
    if Sigma == true
    
        % 2.1 .csv format
        data_sigmaAngle_2D = reshape(permute(data_sigmaAngle, [2 1 3]), [], 2);
        filename_sigma_csv = fullfile(datasetFolder, sprintf('SigmaAngle_Dataset_%d.csv', idx));
        writematrix(data_sigmaAngle_2D, filename_sigma_csv);
        
        % 2.2 .mat format
        filename_sigma_mat = fullfile(datasetFolder, sprintf('SigmaAngle_Dataset_%d.mat', idx));
        save(filename_sigma_mat, 'data_sigmaAngle');
        
        fprintf('Sigma data saved !\n');
    end
    
end

disp('Simulation over !');
fprintf('Total number of no detection : %d\n', Num_no_detection);
fprintf('Detection ratio : %.1f%%\n', 100*(1 - Num_no_detection/Num_total_measures));