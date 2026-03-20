function [uci,symbol,detMet] = decodeFormat1(carrier,pucch,ouci,sym,nVar,thres)
%decodeFormat1 Physical uplink control channel format 1 decoding
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [UCI,SYMBOL,DETMET] = decodeFormat1(CARRIER,PUCCH,OUCI,SYM,NVAR,THRES)
%   returns the cell containing the vector of hard bits, UCI, resulting
%   from the demodulation of received constellation symbol, SYMBOL, after
%   performing matched filtering with input SYM. CARRIER is a scalar
%   nrCarrierConfig object. PUCCH is a scalar nrPUCCH1Config object. SYM is
%   the received equalized symbols. NVAR is noise variance. OUCI is the
%   number of UCI payload bits. THRES is the threshold value in range 0 to
%   1. The function uses normalized correlation coefficients and returns
%   the maximum of normalized correlation coefficients as the detection
%   metric, DETMET. When DETMET is greater than or equal to the THRES, the
%   function performs matched filtering and symbol demodulation. When
%   DETMET is less than THRES, the function treats the input SYM as
%   discontinuous transmission (DTX) and returns empty bit and symbol. For
%   decoding of only positive scheduling request (SR) transmission, place
%   OUCI as 1 and ensure the UCI cell contain zero.

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    % Check for non-empty symbols, when ouci is 0
    coder.internal.errorIf((ouci == 0) && ~isempty(sym), ...
        'nr5g:nrPUCCHDecode:InvalidSYMWithoutUCI');

    % Get the low peak-to-average-power (low-PAPR) sequence along with the
    % block-wise spreading
    dtType = class(sym);
    modSym = cast((1/sqrt(2))*(-1+1j),dtType); % Conjugate of BPSK symbol for bit 1
    refSym = modSym.*nrPUCCH(carrier,pucch,1,'OutputDataType',dtType);
    cnjRef = conj(refSym); % Conjugate of reference sequence

    % Check the length of SYM
    [symLen,nCols] = size(sym);
    refSeqLen = size(refSym,1);
    coder.internal.errorIf(symLen ~= refSeqLen, ...
        'nr5g:nrPUCCHDecode:InvalidSYMLen',symLen,refSeqLen);

    % Get the energy of received symbols and reference sequence
    refSymP = repmat(refSym,1,nCols);
    eRxSym = sum(abs(sym).^2);
    eRefSym = sum(abs(refSymP).^2);
    normE = sqrt(eRxSym.*eRefSym);
    % Get the normalized correlation coefficient across the entire PUCCH
    % sequence. Add eps to the normalization to avoid dividing by 0.
    detMet = mean(abs(sum(sym.*conj(refSymP)))./(normE+eps));
    symbol = zeros(0,1,dtType);
    if detMet(1) >= thres
        % Get the modulated symbol for each resource element (RE) with
        % which the sequence is multiplied. Then, average across all REs
        symbol = mean(sym.*cnjRef,[1 2]);

        % Perform symbol demodulation, based on modulation scheme
        if ouci == 1
            modScheme = 'BPSK';
        else
            modScheme = 'QPSK';
        end
        uci = {nrSymbolDemodulate(symbol,modScheme,nVar,'DecisionType','hard')};
    else
        % When the detection metric is less than threshold, return empty
        % UCI and symbol as output
        uci = {cast(zeros(0,1),'int8')};
    end

end
