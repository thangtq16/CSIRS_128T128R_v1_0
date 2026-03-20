function [uci,symDeprecode,detMet] = decodeFormat4(carrier,pucch,ouci,sym,nVar,thres)
%decodeFormat4 Physical uplink control channel format 4 decoding
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [UCI,SYMBOLS,DETMET] = decodeFormat4(CARRIER,PUCCH,SYM,OUCI,NVAR,THRES)
%   returns the cell containing the vector of soft bits, UCI, resulting
%   from the inverse of physical uplink control channel processing for
%   format 4, as defined in TS 38.211 6.3.2.6. The function also returns
%   the received constellation symbols, SYMBOLS and detection metric,
%   DETMET. CARRIER is a scalar nrCarrierConfig object. PUCCH is a scalar
%   nrPUCCH4Config object. OUCI is the number of UCI payload bits. It is
%   either a scalar or two-element vector. SYM is the received equalized
%   symbols. NVAR is noise variance. THRES is the detection threshold in
%   range 0 to 1. When first element of OUCI is in range 3 to 11, the
%   function performs discontinuous transmission (DTX) detection using
%   normalized correlation coefficient metric for all the possible symbol
%   sequences. The maximum of normalized correlation coefficients is
%   returned as the detection metric, DETMET. When DETMET is greater than
%   or equal to the THRES, the function performs transform deprecoding,
%   blockwise despreading, symbol demodulation, and descrambling. When
%   DETMET is less than THRES, the function treats the input SYM as DTX and
%   returns empty soft bits. For all other values of OUCI, the function
%   directly performs transform deprecoding, blockwise despreading, symbol
%   demodulation and descrambling, without DTX detection.

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    % Cache the scrambling identity and radio network temporary identifier (RNTI)
    if isempty(pucch.NID)
        nid = double(carrier.NCellID);
    else
        nid = double(pucch.NID(1));
    end
    rnti = pucch.RNTI;
    Gd = size(sym,1);
    numCols = 2^double(ouci(1));

    % Persistent variables to store format 4 reference symbols
    persistent format4Sym format4NID format4RNTI format4Gd format4NumUCI ...
         format4NumPRB format4ModScheme format4SF format4OCCI format4Check

    % Check for non-empty symbols, when ouci is 0
    symPresence = ~isempty(sym);
    coder.internal.errorIf(all(ouci == 0) && symPresence, ...
        'nr5g:nrPUCCHDecode:InvalidSYMWithoutUCI');

    % Get the data type of input symbols, number of unique physical
    % resource blocks (PRBs), modulation order, and spreading factor
    dtType = class(sym);
    Mrb = numel(unique(pucch.PRBSet(:)));
    qm = nr5g.internal.getQm(pucch.Modulation);
    sf = double(pucch.SpreadingFactor);
    % Perform threshold check for PUCCH format 4 symbols, when ouci is in
    % range 3 to 11
    if (isscalar(ouci) || (ouci(2) == 0)) && (ouci(1) >= 3 && ouci(1) <= 11) && symPresence
        % Check the product of modulation order and Gd is an integer
        % multiple of spreading factor
        G = Gd*qm/sf;
        coder.internal.errorIf(fix(G) ~= G, ...
            'nr5g:nrPUCCHDecode:InvalidG',qm,Gd,sf);

        % Check ouci and Gd values (qm*Gd/sf must be greater than first element of ouci)
        coder.internal.errorIf(G <= ouci(1), ...
            'nr5g:nrPUCCHDecode:InvalidSYMLenF4',qm,Gd,sf,ouci(1));

        % Initialize number of UCI bits for format 4
        if isempty(format4NumUCI)
            format4NumUCI = ouci(1);
        end

        % Initialize number of symbols
        if isempty(format4Gd)
            format4Gd = Gd;
        end

        % Initialize RNTI
        if isempty(format4RNTI)
            format4RNTI = rnti;
        end

        % Initialize NID
        if isempty(format4NID)
            format4NID = nid;
        end

        % Initialize number of PRBs
        if isempty(format4NumPRB)
            format4NumPRB = Mrb;
        end

        % Initialize modulation scheme
        if isempty(format4ModScheme)
            format4ModScheme = pucch.Modulation;
        end

        % Initialize spreading factor
        if isempty(format4SF)
            format4SF = sf;
        end

        % Initialize OCCI
        if isempty(format4OCCI)
            format4OCCI = pucch.OCCI;
        end

        % Initialize this variable to check if format 4 symbols are
        % generated correctly
        if isempty(format4Check)
            format4Check = 0;
        end

        % Initialize format 4 reference symbols
        if isempty(format4Sym) || ~format4Check || (format4NumUCI ~= ouci(1)) || (format4Gd ~= Gd) || ...
                (format4NID ~= nid) || (format4RNTI ~= rnti) || (format4OCCI ~= pucch.OCCI) ||...
                (format4NumPRB ~= Mrb) || ~strcmpi(pucch.Modulation,format4ModScheme) || (format4SF ~= sf)
            format4Check = 0; %#ok<NASGU>
            format4Sym = complex(zeros(Gd,numCols));
            for ul = 0:numCols-1
                enc = nrUCIEncode(int2bit(ul,ouci(1),false),G);
                format4Sym(:,ul+1) = nrPUCCH(carrier,pucch,enc);
            end
            % Assign the values back, to ensure the computation of all the
            % possible symbols is performed only once and for a change in
            % one of the values from previous call
            format4NumUCI = ouci(1);
            format4Gd = Gd;
            format4NID = nid;
            format4RNTI = rnti;
            format4NumPRB = Mrb;
            format4ModScheme = pucch.Modulation;
            format4SF = sf;
            format4OCCI = pucch.OCCI;
            format4Check = 1;
        end

        % Get the energy of received symbols and reference symbols
        format4SymType = cast(format4Sym,dtType);
        eSym = sum(abs(sym).^2);
        eformat4Sym = sum(abs(format4SymType).^2);
        % Get the normalized correlation coefficient for all the possible
        % reference symbols. Add eps to the normalization to avoid dividing
        % by 0.
        ncc = zeros(1,numCols,dtType);
        for ul = 1:numCols
            normE = sqrt(eSym*eformat4Sym(ul));
            ncc(ul) = abs(sum(sym.*conj(format4SymType(:,ul))))/(normE+eps);
        end
        % Compare the maximum normalized correlation coefficient against a
        % threshold
        detMet = max(ncc);
        thresholdFlag = detMet(1) >= thres;
    else
        detMet = zeros(1,1,dtType);
        thresholdFlag = true;
    end

    if thresholdFlag
        % Perform transform deprecoding
        symTransDeprecode = nrTransformDeprecode(sym,Mrb);

        % Blockwise despreading
        symDeprecode = nr5g.internal.pucch.blockWiseDespread(symTransDeprecode,Mrb,sf,double(pucch.OCCI));

        % Perform symbol demodulation
        softBits = nrSymbolDemodulate(symDeprecode,pucch.Modulation,nVar);

        % Perform descrambling
        opts.MappingType = 'signed';
        opts.OutputDataType = dtType;
        c = nrPUCCHPRBS(nid,rnti,length(softBits),opts);
        uci = {softBits .* c};
    else
        % When the detection metric is less than threshold, return empty
        % UCI and symbols as output
        uci = {zeros(0,1,dtType)};
        symDeprecode = zeros(0,1,dtType);
    end

end