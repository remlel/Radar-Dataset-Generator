function plot_range_doppler_map(resp, rngGrid, dopGrid)

    figure;
    imagesc(dopGrid, rngGrid, mag2db(abs(resp)));
    axis xy; 
    colorbar;
    title('Range-Doppler Map');
    xlabel('Radial Velocity (m/s) [+ = rapprochement]');
    ylabel('Distance (m)');
    ylim([0 150]);
    clim([-75 max(mag2db(abs(resp(:))))]); % Adjusting contrast (minimal coloration = -60dB) => better target visibility

end
    