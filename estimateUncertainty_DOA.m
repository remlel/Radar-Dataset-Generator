function [sigmaAz, sigmaEl] = estimateUncertainty_DOA(spectrum)

    % Valide pour des pas non unitaire & spectres non carré
    [Ny, Nx] = size(spectrum); 
    Az = linspace(-45, 45, Nx);
    El = linspace(-45, 45, Ny);

    spec_dB = 10*log10(max(spectrum, eps));

    [maxVal, idx] = max(spec_dB(:));
    [row, col] = ind2sub(size(spec_dB), idx);

    threshold = maxVal - 3;

    coupeAz = spec_dB(row, :);
    coupeEl = spec_dB(:, col).'; 

    % ================= AZIMUT =================

    left = col;
    while left > 1 && coupeAz(left) > threshold
        left = left - 1;
    end

    % Gestion des bords
    if coupeAz(left) > threshold
        azLeft = Az(1);
    elseif left < col && coupeAz(left) ~= coupeAz(left+1)
        azLeft = interp1([coupeAz(left) coupeAz(left+1)], [Az(left) Az(left+1)], threshold);
    else
        azLeft = Az(left);
    end

    right = col;
    while right < Nx && coupeAz(right) > threshold
        right = right + 1;
    end

    if coupeAz(right) > threshold
        azRight = Az(Nx); 
    elseif right > col && coupeAz(right-1) ~= coupeAz(right)
        azRight = interp1([coupeAz(right-1) coupeAz(right)], ...
                          [Az(right-1) Az(right)], threshold);
    else
        azRight = Az(right);
    end

    largeurAz = azRight - azLeft;
    sigmaAz = largeurAz / 2.355;

    % ================= ELEVATION =================

    down = row;
    while down > 1 && coupeEl(down) > threshold
        down = down - 1;
    end

    if coupeEl(down) > threshold
        elDown = El(1);
    elseif down < row && coupeEl(down) ~= coupeEl(down+1)
        elDown = interp1([coupeEl(down) coupeEl(down+1)], ...
                         [El(down) El(down+1)], threshold);
    else
        elDown = El(down);
    end

    up = row;
    while up < Ny && coupeEl(up) > threshold
        up = up + 1;
    end

    if coupeEl(up) > threshold
        elUp = El(Ny);
    elseif up > row && coupeEl(up-1) ~= coupeEl(up)
        elUp = interp1([coupeEl(up-1) coupeEl(up)], ...
                       [El(up-1) El(up)], threshold);
    else
        elUp = El(up);
    end

    largeurEl = elUp - elDown;
    sigmaEl = largeurEl / 2.355;

end