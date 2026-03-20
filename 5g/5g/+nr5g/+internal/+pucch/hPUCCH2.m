function sym = hPUCCH2(uciCW,nid,rnti,nIRB,sf,occi,varargin)
%hPUCCH2 Physical uplink control channel format 2
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    % Scrambling, TS 38.211 Section 6.3.2.5.1
    c = nrPUCCHPRBS(nid,rnti,length(uciCW));
    btilde = xor(uciCW,c);

    % Modulation, TS 38.211 Section 6.3.2.5.2
    d = nrSymbolModulate(btilde,'QPSK',varargin{:});

    % Symbol spreading for operation in unlicensed spectrum, as described
    % in TS 38.211 Section 6.3.2.5.2A (Rel-16). Symbol spreading applies to
    % single-interlace transmissions only, which is indicated here by
    % content of the spreading factor.
    noSpreading = isempty(nIRB) || isempty(sf);
    if noSpreading

        sym = d;

    else % Symbol spreading

        % Verify that the number of input bits is compatible with the bit
        % capacity. The number of REs in each RB and OFDM symbol available
        % for PUCCH with no DM-RS is 8 according to TS 38.211 Section
        % 6.4.1.3.2.2.
        seqLength = length(d);
        modulation = 'QPSK';
        numRB = length(nIRB);
        nRE = 8; % Number of RE per PRB available for PUCCH
        formatPUCCH = 2;
        nr5g.internal.pucch.validateSpreadingConfig(seqLength,modulation,numRB,nRE,sf,formatPUCCH);

        % NIRB-dependent spreading sequence for each PUCCH modulated symbol
        numOFDMSymbols = sf*seqLength/(numRB*nRE);
        wn = nr5g.internal.interlacing.interlacedSpreadingSequences(nIRB,nRE,sf,occi,numOFDMSymbols);

        % Spread PUCCH modulated symbols. The mapping is frequency first.
        z = (d.*wn).';
        sym = z(:);

    end

end