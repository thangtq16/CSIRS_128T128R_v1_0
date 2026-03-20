function [uci,symbols,detMet] = decodeFormat3(carrier,pucch,ouci,sym,nVar,thres)
%decodeFormat3 Physical uplink control channel format 3 decoding
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [UCI,SYMBOLS,DETMET] = decodeFormat3(CARRIER,PUCCH,OUCI,SYM,NVAR,THRES)
%   returns the cell containing the vector of soft bits, UCI, resulting
%   from the inverse of physical uplink control channel processing for
%   format 3, as defined in TS 38.211 6.3.2.6. The function also returns
%   the received constellation symbols, SYMBOLS and detection metric
%   DETMET. CARRIER is a scalar nrCarrierConfig object. PUCCH is a scalar
%   nrPUCCH3Config object. OUCI is the number of UCI payload bits. It is
%   either a scalar or two-element vector. SYM is the received equalized
%   symbols. NVAR is noise variance. THRES is the detection threshold in
%   range 0 to 1. When first element of OUCI is in range 3 to 11, the
%   function performs discontinuous transmission (DTX) detection using
%   normalized correlation coefficient metric for all the possible symbol
%   sequences. The maximum of normalized correlation coefficients is
%   returned as the detection metric, DETMET. When DETMET is greater than
%   or equal to the THRES, the function performs transform deprecoding,
%   symbol demodulation, and descrambling. When DETMET is less than THRES,
%   the function treats the input SYM as DTX and returns empty soft bits.
%   For all other values of OUCI, the function directly performs transform
%   deprecoding, symbol demodulation, and descrambling, without DTX
%   detection.

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    % Cache the scrambling identity and radio network temporary identifier
    % (RNTI)
    if isempty(pucch.NID)
        nid = double(carrier.NCellID);
    else
        nid = double(pucch.NID(1));
    end
    rnti = pucch.RNTI;
    numCols = 2^double(ouci(1));

    % Check for non-empty symbols, when ouci is 0
    symPresence = ~isempty(sym);
    coder.internal.errorIf(all(ouci == 0) && symPresence, ...
        'nr5g:nrPUCCHDecode:InvalidSYMWithoutUCI');

    % Get the data type of input symbols, number of unique physical
    % resource blocks (PRBs), and modulation order
    dtType = class(sym);
    % Perform threshold check for PUCCH format 3 symbols, when ouci is in
    % range 3 to 11
    if (isscalar(ouci) || (ouci(2) == 0)) && (ouci(1) >= 3 && ouci(1) <= 11) && symPresence        
        % Create/update cache of PUCCH reference symbols
        numSymbols = size(sym,1);
        refSym = referenceSymbols(carrier,pucch,ouci,numSymbols);
        
        % Get the energy of received symbols and reference symbols
        format3SymType = cast(refSym,dtType);
        eSym = sum(abs(sym).^2);
        eformat3Sym = sum(abs(format3SymType).^2);
        % Get the normalized correlation coefficient for all the possible
        % reference symbols. Add eps to the normalization to avoid dividing
        % by 0.
        ncc = zeros(1,numCols,dtType);
        for ul = 1:numCols
            normE = sqrt(eSym*eformat3Sym(ul));
            ncc(ul) = abs(sum(sym.*conj(format3SymType(:,ul))))/(normE+eps);
        end
        % Compare the maximum normalized correlation coefficient against a
        % threshold
        detMet = max(ncc);
        thresholdFlag = detMet(1) >= thres;
    else
        detMet = zeros(1,1,dtType);
        thresholdFlag = true;
    end

    % Perform PUCCH format 3 decoding
    if thresholdFlag

        % Determine the number of RB allocated
        prbset = nr5g.internal.allocatedPhysicalResourceBlocks(carrier, pucch);
        Mrb = numel(prbset);

        % Perform transform deprecoding
        symTransDeprecode = nrTransformDeprecode(sym,Mrb);

        % Blockwise despreading of single-interlace transmissions
        [sf,occi] = nr5g.internal.pucch.occConfiguration(pucch);
        if ~isempty(sf)
            symbols = nr5g.internal.pucch.blockWiseDespread(symTransDeprecode,Mrb,sf(1),occi);
        else
            symbols = symTransDeprecode;
        end

        % Perform symbol demodulation
        symdemod = nrSymbolDemodulate(symbols(:),pucch.Modulation,nVar);

        % Perform descrambling
        opts.MappingType = 'signed';
        opts.OutputDataType = dtType;
        c = nrPUCCHPRBS(nid,rnti,length(symdemod),opts);
        uci = {symdemod .* c};
    else
        % When the detection metric is less than threshold, return empty
        % UCI and symbols as output
        uci = {zeros(0,1,dtType)};
        symbols = zeros(0,1,dtType);
    end

end

% PUCCH reference symbols for ML decoding
function refSymbols = referenceSymbols(carrier,pucch,ouci,numSym)

    % Create a structure containing parameters affecting the generation of
    % reference symbols. If any of the following parameters has changed
    % from a previous call, a new set of PUCCH reference symbols will be
    % generated.
    newCache = struct('NumUCI', ouci(1), 'NumSym', numSym, ...
            'RNTI', pucch.RNTI,'Modulation', pucch.Modulation, 'GroupHopping', pucch.GroupHopping, 'HoppingID', pucch.HoppingID,...
            'NID',0,'Interlacing', pucch.Interlacing, 'InterlaceIndex', pucch.InterlaceIndex, 'RBSetIndex', pucch.RBSetIndex, ...
            'SpreadingFactor', pucch.SpreadingFactor, 'OCCI', pucch.OCCI, 'Mrb', zeros(0,1));
    coder.varsize('newCache.InterlaceIndex','newCache.Mrb','newCache.NID');

    % Get PUCCH reference symbols
    refSymbols = nr5g.internal.pucch.decoderReferenceSymbols(carrier,pucch,ouci,numSym,newCache);
    
end