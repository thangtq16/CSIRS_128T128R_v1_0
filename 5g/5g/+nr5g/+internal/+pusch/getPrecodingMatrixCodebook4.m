function [allW,validTPMIs] = getPrecodingMatrixCodebook4()
%getPrecodingMatrixCodebook4 Compute all precoding matrices for PUSCH
% codebook transmission, according to Table 6.3.1.5-47 of TS 38.211
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023-2024 The MathWorks, Inc.

%#codegen

    nports = 8; % Set this value explicitly for the sake of readability
    layersWithAdditionalTPMI = [1,2,3,4]; % An extra TPMI is allowed for 1, 2, 3, and 4 layers
    numMatrices = sum(arrayfun(@(k)nchoosek(nports,k),1:nports)) + numel(layersWithAdditionalTPMI);
    allW = coder.nullcopy(cell(numMatrices,1));
    validTPMIs = cell(nports,1);
    lastTPMI = -1;
    for nlayers = 1:nports
        % Get the allowed number of TPMI values for nlayers
        Delta = nchoosek(nports,nlayers);

        % Get the list of valid TPMI values for nlayers
        numValidTPMIs = Delta + any(nlayers==layersWithAdditionalTPMI);
        validTPMIs{nlayers} = nan(1,numValidTPMIs);
        validTPMIs{nlayers}(1:Delta) = lastTPMI + (1:Delta);
        lastTPMI = validTPMIs{nlayers}(Delta);
        if any(nlayers==layersWithAdditionalTPMI)
            validTPMIs{nlayers}(end) = 254 + nlayers;
        end

        % Get all nonzero locations for each matrix W
        tmp = nchoosek(1:nports,nlayers);
        layerPortMapIndex = tmp;
        
        % Construct the value of L for each matrix W and sort them in ascending
        % order
        L = sum((2.^(layerPortMapIndex-1)),2);
        [~,Lind] = sort(L);

        % Get allW for this number of layers
        for t = 1:Delta
            portInd = layerPortMapIndex(Lind(t),:);
            W = zeros(nports,nlayers);
            for ll = 1:nlayers
                W(portInd(ll),ll) = 1;
            end
            allW{validTPMIs{nlayers}(t)+1} = W;
        end
    end

    % Add last matrices for TPMI values of 255 to 257
    
    % TPMI = 255 is used for one layer
    TPMI = 255;
    allW{TPMI+1} = ones(nports,1);

    % TPMI = 256 is used for two layers
    TPMI = 256;
    allW{TPMI+1} = [1 1 0 0 1 1 0 0;
                   0 0 1 1 0 0 1 1]';

    % TPMI = 257 is used for three layers
    TPMI = 257;
    allW{TPMI+1} = [1 1 0 0 1 1 0 0;
                   0 0 1 0 0 0 1 0;
                   0 0 0 1 0 0 0 1]';

    % TPMI = 258 is used for four layers
    TPMI = 258;
    allW{TPMI+1} = [1 0 0 0 1 0 0 0;
                   0 1 0 0 0 1 0 0;
                   0 0 1 0 0 0 1 0;
                   0 0 0 1 0 0 0 1]';

    % Normalize all matrices by 1/(2*sqrt(2))
    allW = cellfun(@(x)(x/(2*sqrt(2))), allW, 'UniformOutput', false);

end