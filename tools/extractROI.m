function roi = extractROI(resp, row, col)

    % Définition de la taille de la ROI
    half_size = 16;
    roi_size = 2*half_size + 1;

    if isempty(row)
        roi = zeros(roi_size, roi_size);
        return;
    end

    r_min = row - half_size;
    r_max = row + half_size;
    c_min = col - half_size;
    c_max = col + half_size;

    if r_min < 1 || c_min < 1 || r_max > size(resp, 1) || c_max > size(resp, 2)
        roi = zeros(roi_size, roi_size);
        return;
    end

    roi = resp(r_min:r_max, c_min:c_max);

end