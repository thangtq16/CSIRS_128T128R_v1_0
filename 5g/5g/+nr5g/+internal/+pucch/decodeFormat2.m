function [uci,symbols,detMet] = decodeFormat2(carrier,pucch,ouci,sym,nVar,thres)
%decodeFormat2 Physical uplink control channel format 2 decoding
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [UCI,SYMBOLS,DETMET] = decodeFormat2(CARRIER,PUCCH,OUCI,SYM,NVAR,THRES)
%   returns the cell containing the vector of soft bits, UCI, resulting
%   from the inverse of physical uplink control channel processing for
%   format 2, as defined in TS 38.211 6.3.2.5. The function also returns
%   the received constellation symbols, SYMBOLS. CARRIER is a scalar
%   nrCarrierConfig object. PUCCH is a scalar nrPUCCH2Config object. SYM is
%   the received equalized symbols. OUCI is the number of UCI payload bits.
%   NVAR is noise variance. THRES is the threshold value in range 0 to 1.
%   When OUCI is in range 3 to 11, the function performs discontinuous
%   transmission (DTX) detection using normalized correlation coefficient
%   metric for all the possible symbol sequences. The maximum of normalized
%   correlation coefficients is returned as the detection metric, DETMET.
%   When DETMET is greater than or equal to the THRES, the function
%   performs symbol demodulation and descrambling. When DETMET is less than
%   THRES, the function treats the input SYM as DTX and returns empty soft
%   bits. For all other values of OUCI, the function directly performs
%   symbol demodulation and descrambling, without DTX detection.

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    % Cache the scrambling identity, radio network temporary identifier
    % (RNTI), and number of input symbols
    if isempty(pucch.NID)
        nid = double(carrier.NCellID);
    else
        nid = double(pucch.NID(1));
    end

    rnti = pucch.RNTI;
    numCols = 2^double(ouci);

    % Check for non-empty symbols, when ouci is 0
    symPresence = ~isempty(sym);
    coder.internal.errorIf((ouci == 0) && symPresence, ...
        'nr5g:nrPUCCHDecode:InvalidSYMWithoutUCI');

    % Perform threshold check for PUCCH format 2 symbols, when ouci is in
    % range 3 to 11
    dtType = class(sym);
    if (ouci(1) >= 3 && ouci(1) <= 11) && symPresence        
        % Create/update cache of PUCCH reference symbols
        numSym = size(sym,1); 
        refSym = referenceSymbols(carrier,pucch,ouci,numSym);

        % Get the energy of received symbols and reference symbols
        symRef = cast(refSym,dtType);
        eSym = sum(abs(sym).^2);
        eSymRef = sum(abs(symRef).^2);
        % Get the normalized correlation coefficient for all the possible
        % reference symbols. Add eps to the normalization to avoid dividing
        % by 0.
        ncc = zeros(1,numCols,dtType);
        for ul = 1:numCols
            normE = sqrt(eSym*eSymRef(ul));
            ncc(ul) = abs(sum(sym.*conj(symRef(:,ul))))/(normE+eps);
        end
        % Compare the maximum normalized correlation coefficient against a
        % threshold
        detMet = max(ncc);
        thresholdFlag = detMet(1) >= thres;
    else
        detMet = zeros(1,1,dtType);
        thresholdFlag = true;
    end

    % Perform PUCCH format 2 decoding
    symbols = sym;
    if thresholdFlag

        % Despreading of single-interlace transmissions
        despreading = pucch.Interlacing && numel(pucch.InterlaceIndex)==1;
        if despreading
            symbols = nr5g.internal.interlacing.symbolDespread(carrier,pucch,sym);
        end
        % Perform symbol demodulation
        symdemod = nrSymbolDemodulate(symbols,'QPSK',nVar);

        % Perform descrambling
        opts.MappingType = 'signed';
        opts.OutputDataType = dtType;
        c = nrPUCCHPRBS(nid,rnti,length(symdemod),opts);
        uci = {symdemod .* c};

    else
        % When detection metric is less than threshold, return empty UCI as
        % output
        uci = {zeros(0,1,dtType)};
    end

end

% PUCCH reference symbols for ML decoding
function refSymbols = referenceSymbols(carrier,pucch,ouci,numSym)

    % Create a structure containing parameters affecting the generation of
    % reference symbols. If any of the following parameters has changed
    % from a previous call, a new set of PUCCH reference symbols will be
    % generated.
    newCache = struct('NumUCI', ouci(1), 'NumSym', numSym, 'RNTI', pucch.RNTI,...
            'NID',0,'Interlacing', pucch.Interlacing, 'InterlaceIndex', pucch.InterlaceIndex, 'RBSetIndex', pucch.RBSetIndex, ...
            'SpreadingFactor', pucch.SpreadingFactor, 'OCCI', pucch.OCCI, 'Mrb', zeros(0,1));
    coder.varsize('newCache.InterlaceIndex','newCache.RBSetIndex','newCache.Mrb','newCache.NID');

    % Get PUCCH reference symbols
    refSymbols = nr5g.internal.pucch.decoderReferenceSymbols(carrier,pucch,ouci,numSym,newCache);
    
end