function [row, col] = target_detection_cfar(software, resp, Nstep)

    % 1. CUT Cells Delimitation
    row_min = software.tr_range_size + software.gu_range_size + 1;
    row_max = size(resp,1) - software.tr_range_size - software.gu_range_size;
    col_min = software.tr_dop_size + software.gu_dop_size + 1;
    col_max = size(resp,2) - software.tr_dop_size - software.gu_dop_size;

    numRowTest = row_max - row_min + 1;
    numColTest = col_max - col_min + 1;
    numTestCell = numRowTest * numColTest;
    cutidx = zeros(2, numTestCell);    

    k = 1;
    for r = row_min:row_max
        for c = col_min:col_max
            cutidx(:,k) = [r;c];
            k = k+1;
        end
    end
    
    % 2. Target position on RD map
    RD = abs(resp).^2;
    mask = software.cfar(RD, cutidx);

    detected = find(mask);
    
    if isempty(detected)
        warning("No target detected for step index number : %d", Nstep);
        row = [];
        col = [];
        return;
    end
    
    row = cutidx(1, detected);
    col = cutidx(2, detected);

    % Keeps cell of maximum amplitude (in case of : multiple targets or migrations)
    values = RD(sub2ind(size(RD),row,col));
    [~,idx] = max(values);
    row = row(idx);
    col = col(idx);

end