function [uci,detMet] = decodeFormat0(carrier,pucch,ouci,sym,thres)
%decodeFormat0 Physical uplink control channel format 0 decoding
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [UCI,DETMET] = decodeFormat0(CARRIER,PUCCH,OUCI,SYM,THRES) returns the
%   cell containing the vector of hard bits, UCI, resulting from the
%   detection metric, DETMET, for the input symbols SYM. The function uses
%   the average of normalized correlation coefficients across all antennas
%   and returns the maximum of normalized correlation coefficients as the
%   detection metric, DETMET. CARRIER is a scalar nrCarrierConfig object.
%   PUCCH is a scalar nrPUCCH0Config object. SYM is the matrix of received
%   symbols with number of columns indicating the number of receive
%   antennas. OUCI is a scalar or two-element vector with first element
%   indicating the number of hybrid automatic repeat request acknowledgment
%   (HARQ-ACK) bits and second element indicating number of scheduling
%   request (SR) bits. THRES is the threshold value in range 0 to 1. When
%   DETMET is greater than or equal to the threshold THRES, the hard bits
%   of ACK and SR are returned as cell array in UCI. When DETMET is less
%   than THRES, the input SYM is treated as discontinuous transmission
%   (DTX) and empty value is returned in UCI.

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    % Parse number of HARQ-ACK and SR bits
    oack = ouci(1);
    osr = ouci(2);

    % Get the data type of input SYM and output UCI
    dtType = class(sym);
    outDtType = 'int8';

    % Check for non-empty symbols, when both HARQ-ACK and SR payload bit
    % length are 0
    coder.internal.errorIf((oack == 0) && (osr == 0) && ~isempty(sym), ...
        'nr5g:nrPUCCHDecode:InvalidSYMWithoutUCI');

    srOnly = 0;
    if (oack == 0) && osr
        % SR only transmission
        srOnly = 1; % Flag to indicate if there is only SR transmission
    end

    % Check the number of rows in SYM
    Mrb = numel(nr5g.internal.allocatedPhysicalResourceBlocks(carrier,pucch));
    Msc = 12*Mrb;
    nPUCCHSym = double(pucch.SymbolAllocation(2));
    seqLen = Msc*nPUCCHSym;
    symLen = size(sym,1);
    coder.internal.errorIf(symLen ~= seqLen, ...
        'nr5g:nrPUCCHDecode:InvalidSYMLen',symLen,seqLen);

    % Generate reference symbols for all the possible combinations of
    % HARQ-ACK and SR. Also, compute the normalized correlation coefficient
    % across all antennas for each combination. The correlation is computed
    % independently for each block of 12 subcarriers and 1 OFDM symbol to
    % exploit diversity of frequency hopping configurations in frequency
    % selective channels.
    nACK = 2^oack;
    if ~srOnly
        tmpACK = dec2bin(0:nACK-1,oack) == '1';
    else
        tmpACK = false(1,0);
    end
    nSR = 2^osr;
    c = zeros([nSR nACK],dtType);
    symRB = reshape(sym,Msc,[]);
    eSymRB = sum(abs(symRB).^2);
    for srIdx = 1:nSR
        for ackIdx = 1:nACK
            % Generate reference symbols for HARQ-ACK and SR
            refSymTmp = nrPUCCH(carrier,pucch,{tmpACK(ackIdx,:)' srIdx-1},"OutputDataType",dtType);
            
            if ~isempty(refSymTmp) % Don't compute correlation for -SR only
                refSymRB = repmat(reshape(refSymTmp,Msc,[]),1,size(sym,2));
                eRefSymRB = sum(abs(refSymRB).^2);
                normE = sqrt(eSymRB.*eRefSymRB); 
                % Get the mean of normalized correlation coefficients
                % across all antennas for these reference symbols. Add eps
                % to the normalization to avoid dividing by 0.
                c(srIdx,ackIdx) = mean(abs(sum(symRB.*conj(refSymRB)))./(normE+eps));
            end
        end
    end


    % Get HARQ-ACK and SR bits, depending on the normalized correlation
    % coefficient (i.e. detection metric) and threshold
    detMet = max(c,[],'all');
    if detMet(1) >= thres % Detection of ACK bits with or without SR
        [rIdx,cIdx] = find(c == repmat(detMet(1),nSR,nACK));
        % HARQ-ACK
        rxACK = tmpACK(cIdx,:)';
        % SR
        if osr
            rxSR = (rIdx == 2);
        else
            % No SR transmission
            rxSR = false(0,1);
        end
    else % When the detection metric is less than threshold:
        % Return empty HARQ-ACK
        rxACK = false(0,1);
        % Return negative SR for SR-only transmissions or empty otherwise.
        rxSR = false(srOnly,1);
    end
    uci = {cast(rxACK,outDtType) cast(rxSR,outDtType)}; % {HARQ-ACK SR}

end
