function H_LS = despreadFDCDM(H_LS,fdCDM,noPRGBundling,nRefSymPRG)
%DESPREADFDCDM performs the FD-CDM despreading on the LS estimate H_LS
%based on the FD-CDM length fdCDM.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    if noPRGBundling
        % No PRG bundling - process the whole grid

        H_LS = despreadFDCDMCore(H_LS,fdCDM);

    else
        % PRG bundling - no need to check interpolation policy as FD-CDM
        % despreading does not rely on the absolute locations of the
        % reference symbols. PRGs with the same number of reference symbols
        % on the current symbol on the current port can all be processed in
        % parallel

        % Calculate the index boundaries inside H_LS for each PRG
        prgEdges = cumsum([[1;nRefSymPRG(1:end-1)] nRefSymPRG],1);

        uNRefSymPRG = unique(nRefSymPRG);
        for i = 1:numel(uNRefSymPRG)

            % Find all PRGs with the same number of LS estimates
            nRefSymThisSize = uNRefSymPRG(i);
            idx = find(nRefSymPRG==nRefSymThisSize);

            % Find the RB index range corresponding to these PRGs
            kRangeThisSize = [];
            for j = 1:numel(idx)
                kRangeThisSize = [kRangeThisSize prgEdges(idx(j),1):prgEdges(idx(j),2)]; %#ok<AGROW>
            end

            % Reshape to put multiple PRGs on multiple pages to despread in
            % parallel
            h_LS = reshape(H_LS(kRangeThisSize),nRefSymThisSize,1,[]);

            % FD-CDM despreading multiple PRGs in parallel
            h_LS = despreadFDCDMCore(h_LS,fdCDM);

            % Reshape back into a single column
            h_LS = reshape(h_LS,[],1,1);

            % Assign back into proper locations inside H_LS
            H_LS(kRangeThisSize) = h_LS;

        end

    end

end

%% Local functions

% Core calculation for FD-CDM despreading on the current symbol in the
% current port, to be performed either across the whole frequency range of
% the grid or on a (group of) PRG(s) (with the same number of reference
% symbols), 'B'. 'B' is either a column vector, or an array of the size
% M-by-1-by-N, where M is the number of reference symbols in each PRGs and
% N is the number of such PRGs.
function B = despreadFDCDMCore(B,fdCDM)

    numSym = size(B,1);
    numPage = size(B,3);
    m = mod(numSym,fdCDM);

    for a = 0:double(m~=0)

        if (~a)

            % whole CDM lengths (may be empty)
            k_LS = 1:(numSym-m);
            nkCDM = fdCDM;

        else

            % part CDM length (may be empty)
            k_LS = numSym + (-m+1:0);
            nkCDM = m;

        end

        % Extract the LS estimates and reshape so that each
        % column contains an FD-CDM group
        x = reshape(B(k_LS,:,:),nkCDM,[],numPage);

        % Average across 1st dimension (i.e. across the
        % FD-CDM group) and repeat the averaged value
        x = repmat(mean(x,1),[nkCDM 1 1]);

        % Reshape back into a single column
        B(k_LS,:,:) = reshape(x,[],1,numPage);

    end

end