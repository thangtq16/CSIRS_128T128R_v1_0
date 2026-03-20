function [ind,info] = getIndices(carrier,prach,opts)
%getIndices Get the PRACH indices.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.

%  Copyright 2022 The MathWorks, Inc.

%#codegen

    % Initialize empty linear indices and output vectors
    kk = []; ll = []; pp = [];
    PRBSet = zeros(0,1);
    info = struct();
    
    % Get the grid size
    carrierNRB = double(carrier.NSizeGrid);
    carrierSCS = double(carrier.SubcarrierSpacing);
    gridsize = nr5g.internal.prach.gridSize(carrierNRB, carrierSCS, prach, 1);
    
    % If PRACH is active in this subframe:
    if nr5g.internal.prach.isActive(prach)
        
        % Get frequency locations
        table = nr5g.internal.prach.getTable6332x(1);
        tableIndex = table.LRA == prach.LRA & ...
                     table.PRACHSubcarrierSpacing == prach.SubcarrierSpacing & ...
                     table.PUSCHSubcarrierSpacing == carrierSCS;
        
        K = carrierSCS / prach.SubcarrierSpacing;
        [k1, PRBSet] = getFreqParameters(carrier, prach, tableIndex);
        errorFiller = {prach.LRA,sprintf('%g',PRBSet(1)),sprintf('%g',PRBSet(end)),sprintf('%g',carrierNRB),prach.RBOffset,prach.FrequencyStart,'FrequencyIndex',prach.FrequencyIndex};
        if any(prach.LRA == [571,1151]) && prach.FrequencyRange=="FR1"
            errorFiller{end-1} = 'RBSetOffset';
            errorFiller{end} = prach.RBSetOffset;
        end
        coder.internal.errorIf(PRBSet(end)>=carrierNRB,'nr5g:nrPRACH:InvalidPRBSet',errorFiller{:});
        
        %kbar Additional parameter needed for the OFDM baseband signal
        % generation (TS 38.211 Section 5.3.2) and given by TS 38.211 Table
        % 6.3.3.2-1.
        kbar = table.kbar(tableIndex);
        
        % Get frequency locations k
        coder.varsize('k',[1 1151],[0 1]);
        first = K*k1 + kbar; % PRACH-specific frequency shift
        k = 0:(prach.LRA-1);
        zeroFreq = double(gridsize(1)/2); % Set the origin of the frequency domain to the middle of the grid
        k = ceil(k + zeroFreq(1) + first(1));
        
        % Get time locations
        L = nr5g.internal.prach.getNumOFDMSymbols(prach);
        l = prach.SymbolLocation(1) + (0:L-1) - double(gridsize(2))*prach.ActivePRACHSlot; % Consider a single PRACH slot
        
        % Get antenna ports locations
        % Note that only one antenna port is used for PRACH. However, this
        % value can be later used for beamforming
        p = zeros(size(k));
        
        % Retrieve 0-based linear indices
        [ll,kk] = meshgrid(l, k);
        pp = repmat(p(:), 1, L);
    end
    
    ind = nr5g.internal.applyIndicesOptions(gridsize, opts, kk, ll, pp);
    info.PRBSet = PRBSet;
end

function [k1, PRBSet] = getFreqParameters(carrier, prach, tableIndex)
    % Get the additional parameter 'k1' and the set of PRBs for PUSCH
    % occupied by PRACH.
    % The former is used to generate the frequency locations, as discussed
    % in TS 38.211 Section 5.3.2, whereas the latter is used in the
    % informational output to give the corresponding set of PRBs for the
    % PUSCH that the PRACH occupies.
    
    %NRB Number of resource blocks occupied which is given by the
    % parameter allocation expressed in number of RBs for PUSCH.
    % TS 38.211 Table 6.3.3.2-1.
    table = nr5g.internal.prach.getTable6332x(1);
    NRB = table.NRBAllocation(tableIndex);
    
    %PRBSet Set of PRBs for PUSCH occupied by PRACH (0-based)
    coder.varsize('PRBSet',[1 192],[0 1]);
    if any(prach.LRA==[571, 1151]) && prach.FrequencyRange=="FR1"
        rbsetOffset = prach.RBSetOffset;
    else % (LRA==139,839) || (LRA==571,1151 && FR2)
        rbsetOffset = prach.FrequencyIndex*NRB;
    end
    PRBFirst = prach.RBOffset + prach.FrequencyStart + rbsetOffset;
    PRBSet = PRBFirst(1) + (0:NRB-1);
    
    %k0 Auxiliary parameter for the computation of k1
    % Note that information about the entire list of carriers is not
    % available at this stage and so we assume that mu==mu0. That is, k0 = 0.
    k0 = 0;
    
    % Get k1
    k1 = k0 + (PRBSet(1) - double(carrier.NSizeGrid)/2)*12;
end