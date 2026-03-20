function pdcchDMRS = dmrsSymbols(carrier,pdcch,symRBIdx,opts)
%dmrsSymbols Compute PDCCH DM-RS symbols
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   See also nrPDCCHResources, nrPDCCHSpace.

%   Copyright 2019-2021 The MathWorks, Inc.

%   Reference:
%   [1] 3GPP TS 38.211, "3rd Generation Partnership Project; Technical
%   Specification Group Radio Access Network; NR; Physical channels and
%   modulation. Sections 7.4.1.3.

%#codegen

    % Get parameters
    crstDuration = pdcch.CORESET.Duration;
    nsym = double(pdcch.SearchSpace.StartSymbolWithinSlot); % 0-based
    nStartBWP = uint32(pdcch.NStartBWP);
    nStartGrid = uint32(carrier.NStartGrid);
    cstRB = nr5g.internal.pdcch.getCORESETPRB(pdcch.CORESET,nStartBWP); % 0-based PRBs of CORESET in BWP
    c0offset = uint32(cstRB(1)); % Lowest-numbered RB in the CORESET (BWP-wise)

    % Same PRB indices across all symbols, adjusted by firstSymLoc
    if strcmpi(opts.IndexOrientation,'carrier')
        crbo = nStartGrid;                       % CRB 0 offset
        prbInd = symRBIdx(:,1) - nsym*uint32(carrier.NSizeGrid); % 1-based
        bwpOffset = nStartBWP - nStartGrid;
        c0offset = c0offset + bwpOffset;                         % Shift CORESET0 offset by BWP offset
    else % BWP
        crbo = uint32(pdcch.NStartBWP);                          % CRB 0 offset
        prbInd = symRBIdx(:,1) - nsym*uint32(pdcch.NSizeBWP);    % 1-based
    end

    % If CORESET ID is 0, the reference point is subcarrier 0 of the
    % lowest-numbered resource block in the CORESET. 
    if pdcch.CORESET.CORESETID == 0
        crbo = uint32(0);
        prbInd = prbInd - c0offset;
    end
    numPRB = max(prbInd);     % number of PRBs per symbol
    if isempty(pdcch.DMRSScramblingID)
        nID = double(carrier.NCellID);
    else
        nID = double(pdcch.DMRSScramblingID);
    end
    seqSampPerRB = 3*2;       % 3REs per RB, 2 bits per DM-RS symbol

    dmrsSym = complex(zeros(3*length(prbInd),crstDuration, ...
        opts.OutputDataType));
    NSlot = mod(double(carrier.NSlot),carrier.SlotsPerFrame);

    for idx = 1:crstDuration
        % Generate sequence, Sections 7.4.1.3.1, 7.4.1.3.2
        cinit = mod(2^17*(carrier.SymbolsPerSlot*NSlot ...
                    +nsym+1)*(2*nID+1)+2*nID,2^31);
        %   Binary mapping, logical output
        cSeq = nrPRBS(cinit,seqSampPerRB*[crbo numPRB]);
        cSeqRB = reshape(cSeq,seqSampPerRB,[]);
        cSequence = cSeqRB(:,prbInd);

        % Modulate and assign per OFDM symbol
        dmrsSym(:,idx) = nrSymbolModulate(cSequence(:),'QPSK', ...
            'OutputDataType',opts.OutputDataType);
        nsym = nsym + 1;
    end
    pdcchDMRS = dmrsSym(:);

end