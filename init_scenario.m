function [radarPlatform, targetPlatform, target] = init_scenario(params)

    % DÉFINITION DES PLATEFORMES ET DE LA CINÉMATIQUE

    % Radar (Placé à l'origine, fixe)
    radarPlatform = phased.Platform('InitialPosition', [0; 0; 0], 'Velocity', [0; 0; 0]);

    % Cible  
    targetPlatform = phased.Platform('InitialPosition', [50; 20; 30], 'Velocity', [-15; -5; 0]);

    % Surface Equivalente Radar (RCS) 
    target = phased.RadarTarget('MeanRCS', 1, 'PropagationSpeed', params.c, 'OperatingFrequency', params.fc);

end