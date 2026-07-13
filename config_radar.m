function [hardware, software, params] = config_radar()


    % ---| PARAMÈTRES DU RADAR FMCW (Fréquence Modulée) |---

    % Simulation d'un radar type automobile ou drone 
    params.fc = 77e9;                             % Fréquence porteuse 
    params.c = physconst('LightSpeed');    
    params.lambda = params.c/params.fc;                  

    % Paramètres de la forme d'onde (Chirp)
    params.bw = 150e6;                            % Bande passante (=> résolution en distance)
    params.sweepTime = 20e-6;                     % Temps d'un chirp
    params.prf = 1/(params.sweepTime);            % Fréquence de répétition des impulsions
    params.fs = 50e6;                             % Fréquence d'échantillonnage 

    params.numChirps = 128;                       % Nombre de chirps par trame (=> résolution Doppler)


    % ---| MODÈLES MATÉRIELS ET PROPAGATION |---

    hardware.waveform = phased.FMCWWaveform('SweepTime', params.sweepTime, 'SweepBandwidth', params.bw, 'SampleRate', params.fs);

    hardware.num_elements = 16;

    % Mise en réseaux d'antennes
    element = phased.CosineAntennaElement('FrequencyRange', [50e9 100e9], 'CosinePower', [1.5 1.5]);
    array = phased.URA('Element', element, 'Size', [4, 4], 'ElementSpacing', [params.lambda/2, params.lambda/2]);       % Uniform Rectangular Area
    
    hardware.radiator = phased.Radiator('Sensor', array, 'OperatingFrequency',params.fc);
    hardware.collector = phased.Collector('Sensor',array, 'OperatingFrequency',params.fc);

    hardware.transmitter = phased.Transmitter('PeakPower', 0.05, 'Gain', 20);
    hardware.receiver = phased.ReceiverPreamp('Gain', 20, 'NoiseFigure', 6, 'SampleRate', params.fs);

    % Modèle de propagation dans l'espace vide (Aller-Retour)
    hardware.channel = phased.FreeSpace('PropagationSpeed', params.c, 'OperatingFrequency', params.fc, 'SampleRate', params.fs, 'TwoWayPropagation', true);


    % ---| SOFTWARE |---

    % Estimer DOA
    software.estimator = phased.MUSICEstimator2D('SensorArray', array, 'OperatingFrequency', params.fc, 'AzimuthScanAngles', -45:45, ...
        'ElevationScanAngles', -45:45, 'DOAOutputPort', true, 'NumSignalsSource','Property', 'NumSignals',1);
    
    % CFAR
    software.tr_range_size = 20;
    software.tr_dop_size = 20;
    software.tr_size = [software.tr_range_size software.tr_dop_size];
    software.gu_range_size = 2;
    software.gu_dop_size = 2;
    software.gu_size = [software.gu_range_size software.gu_dop_size];
    
    software.cfar = phased.CFARDetector2D('TrainingBandSize', software.tr_size, 'GuardBandSize', software.gu_size, 'ProbabilityFalseAlarm',1e-6);
    
    % FFT fast-time / FFT slow-time / Fenêtrage
    software.rngDopResp = phased.RangeDopplerResponse('RangeMethod', 'FFT', ...
        'DopplerOutput', 'Speed', 'SweepSlope', params.bw/params.sweepTime, ...
        'RangeWindow', 'Hann', 'DopplerWindow', 'Hann', ...
        'PropagationSpeed', params.c, 'OperatingFrequency', params.fc, 'SampleRate', params.fs);

end