function Affichage_Carte_Range_Doppler(resp, rngGrid, dopGrid)

    figure;
    imagesc(dopGrid, rngGrid, mag2db(abs(resp)));
    axis xy; 
    colorbar;
    title('Carte Range-Doppler Initiale');
    xlabel('Vitesse radiale (m/s) [- = rapprochement]');
    ylabel('Distance (m)');
    ylim([0 200]);
    clim([-60 max(mag2db(abs(resp(:))))]); % Ajustement du contraste (coloration minimale = -60dB) => visibilité cible

end
    