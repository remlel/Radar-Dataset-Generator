function [hardware, software, params, patn] = config_radar()


    % ---| FMCW RADAR PARAMETERS (Frequency Modulated) |---

    % Simulation of an automotive- or drone-type radar
    params.fc = 77e9;                             % Carrier frequency
    params.c = physconst('LightSpeed');    
    params.lambda = params.c/params.fc;                  

    % Waveform parameters (Chirp)
    params.bw = 150e6;                            % Bandwidth (=> range resolution)
    params.sweepTime = 20e-6;                     % Chirp duration
    params.prf = 1/(params.sweepTime);            % Pulse repetition frequency
    params.fs = 50e6;                             % Sampling rate 

    params.numChirps = 128;                       % Number of chirps per frame (=> Doppler resolution)
    
    % Simulation parameters
    params.Update_rate = 10;
    params.step = 1/params.Update_rate;
    params.Total_time = 15;
    params.Num_step = floor(params.Total_time * params.Update_rate);

    % ---| HARDWARE MODELS AND PROPAGATION |---

    hardware.waveform = phased.FMCWWaveform('SweepTime', params.sweepTime, 'SweepBandwidth', params.bw, 'SampleRate', params.fs);
    
    hardware.size = 2;
    hardware.num_elements = hardware.size^2;

    % Antenna arraying
    hardware.element = phased.CosineAntennaElement('FrequencyRange', [50e9 100e9], 'CosinePower', [1.5 1.5]);
    hardware.array = phased.URA('Element', hardware.element, 'Size', [hardware.size, hardware.size], 'ElementSpacing', [params.lambda/2, params.lambda/2]);       % Uniform Rectangular Area
    
    hardware.radiator = phased.Radiator('Sensor', hardware.array, 'OperatingFrequency',params.fc);
    hardware.collector = phased.Collector('Sensor',hardware.array, 'OperatingFrequency',params.fc);

    hardware.transmitter = phased.Transmitter('PeakPower', 0.05, 'Gain', 8);
    hardware.receiver = phased.ReceiverPreamp('Gain', 8, 'NoiseFigure', 6, 'SampleRate', params.fs);

    % Free-space propagation model (round-trip)
    hardware.channel = phased.FreeSpace('PropagationSpeed', params.c, 'OperatingFrequency', params.fc, 'SampleRate', params.fs, 'TwoWayPropagation', true);
    
    % ---| ANTENNA PATTERN |---
    
    % Defines the angular borders of the pattern vector (-> scenario generation space)
    patn.az_max = 40;
    patn.az_min = -40;
    patn.el_max = 40;
    patn.el_min = -40;

    patn.az_vec = patn.az_min:1:patn.az_max;
    patn.el_vec = patn.el_min:1:patn.el_max;

    G_vec = pattern(hardware.array, params.fc, patn.az_vec, patn.el_vec);

    G00 = pattern(hardware.array, params.fc, 0, 0);

    patn.RmaxOverRmax00_vec = 10.^((G_vec - G00) / 20);

    % ---| SOFTWARE |---

    % Estimate DOA
    scan_margin = 5;
    music_az_scan = (patn.az_min-scan_margin):(patn.az_max+scan_margin);
    music_el_scan = (patn.el_min-scan_margin):(patn.el_max+scan_margin);
    software.estimator = phased.MUSICEstimator2D('SensorArray', hardware.array, 'OperatingFrequency', params.fc, 'AzimuthScanAngles', music_az_scan, ...
        'ElevationScanAngles', music_el_scan, 'DOAOutputPort', true, 'NumSignalsSource','Property', 'NumSignals',1);
    
    % CFAR
    software.tr_range_size = 20;
    software.tr_dop_size = 20;
    software.tr_size = [software.tr_range_size software.tr_dop_size];
    software.gu_range_size = 2;
    software.gu_dop_size = 2;
    software.gu_size = [software.gu_range_size software.gu_dop_size];
    
    software.cfar = phased.CFARDetector2D('TrainingBandSize', software.tr_size, 'GuardBandSize', software.gu_size, 'ProbabilityFalseAlarm',1e-6);
    
    % FFT fast-time / FFT slow-time / Windowing
    software.rngDopResp = phased.RangeDopplerResponse('RangeMethod', 'FFT', ...
        'DopplerOutput', 'Speed', 'SweepSlope', params.bw/params.sweepTime, ...
        'RangeWindow', 'Hann', 'DopplerWindow', 'Hann', ...
        'PropagationSpeed', params.c, 'OperatingFrequency', params.fc, 'SampleRate', params.fs);

end