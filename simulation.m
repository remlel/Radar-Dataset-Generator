%% BOUCLE DE SIMULATION 

clear; clc; close all;

% Configuration radar
[hardware, software, params] = config_radar();

% Initialisation du scenario
[radarPlatform, targetPlatform, target] = init_scenario(params);

% Initialisation de la matrice de données (Fast-Time x Slow-Time)
sig = hardware.waveform();      % 1 Chirp
numSamples = length(sig);
rxData = zeros(numSamples, params.numChirps, hardware.num_elements);

% Boucle sur les impulsions (Chirps) -> construire 1 trame (1 carte)
for i = 1:params.numChirps
    
    % 1. Mise à jour des positions 
    [radarPos, radarVel] = radarPlatform(params.sweepTime);
    [tgtPos, tgtVel] = targetPlatform(params.sweepTime);

    % 2. Calcul de l'angle Radar / Cible (paramètre de simulation)
    [~, tgtAng] = rangeangle(tgtPos, radarPos);
    
    % 2. Génération du signal émis
    txSig = hardware.waveform();
    txSigAmp = hardware.transmitter(txSig);
    txSigRad = hardware.radiator(txSigAmp, tgtAng);
    
    % 3. Propagation vers la cible et retour
    rxSigCh = hardware.channel(txSigRad, radarPos, tgtPos, radarVel, tgtVel);
    
    % 4. Réflexion sur la cible
    rxSigTgt = target(rxSigCh);
    
    % 5. Réception matérielle 
    rxSigCol = hardware.collector(rxSigTgt, tgtAng);
    rxSig = hardware.receiver(rxSigCol);
    
    % Dechirping) : Mélange du signal reçu avec le signal émis -> Passage en bande de base
    rxData(:, i, :) = dechirp(rxSig, txSig);

end

%theta = estimator(snapshot);

%% 5. TRAITEMENT DU SIGNAL 

% ---| Affichage Carte Range Doppler |---
rxData_Antenna1 = rxData(:, :, 1);                                     
[resp1, rngGrid, dopGrid] = software.rngDopResp(rxData_Antenna1);    
%Affichage_Carte_Range_Doppler(resp1, rngGrid, dopGrid);

% Obtention des Responses pour chaque antenne
resp = zeros(numSamples, params.numChirps, hardware.num_elements);
for k = 1:hardware.num_elements
    resp(:, :, k) = software.rngDopResp(rxData(:, :, k));
end

% Détection de la cible 
[row, col] = detection_target_CFAR(software, resp(:, :, 1));    % Uniquement sur Antenne 1 : distance entre antennes négligée

% ---| Exctraction de la ROI (Antenne 1) |---
%roi = extractROI(resp1, row, col);

% Obtention Disatance & Vitesse
range = rngGrid(row);
speed = dopGrid(col);

% DOA : Obtention snapshot spatial (antennes) + MUSIC
snapshot = reshape(resp(row, col, :), 1, 16);           
[spectrum, doa] = software.estimator(snapshot);
Az = doa(1);
El = doa(2);
[sigmaAz, sigmaEl] = estimateUncertainty(spectrum, Az, El);


        

