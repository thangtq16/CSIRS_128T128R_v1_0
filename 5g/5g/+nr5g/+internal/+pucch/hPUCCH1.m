function sym = hPUCCH1(ack,sr,symAllocation,cp,nslot,nid,groupHopping,initialCS,freqHopping,occi,Mrb,nIRB,varargin)
%hPUCCH1 Physical uplink control channel format 1
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    lenACK = length(ack);
    lenSR = length(sr);

    % Return empty output either for empty inputs or for negative SR
    % transmission only.
    if (lenACK == 0) && ((lenSR == 0) || (sr(1) == 0))
        % Empty sequence
        seq = zeros(0,1);
    else

        symStart = double(symAllocation(1));    % OFDM symbol index corresponding to start of PUCCH
        nPUCCHSym = double(symAllocation(2));   % Number of symbols allocated for PUCCH

        % If interlacing is enabled, set Mrb = 1 and disable frequency
        % hopping as it is not compatible with interlacing (TS 38.213
        % Section 9.2.1)
        if ~isempty(nIRB)
            Mrb = 1;
            freqHopping = 'disabled';
        end

        % Check ack and sr inputs and get the control information
        if lenACK == 0
            % Positive SR transmission
            uciIn = 0;
        else
            % HARQ-ACK transmission with/without SR
            uciIn = double(ack);
        end

        % Get the modulated symbol for the control information
        if length(uciIn) == 2
            d = (1/sqrt(2))*((1-2*uciIn(1))+1j*(1-2*uciIn(2))); % QPSK
        else
            d = (1/sqrt(2))*((1-2*uciIn)+1j*(1-2*uciIn));       % BPSK
        end

        % Get the hopping parameters
        seqCS = 0;               % Sequence cyclic shift
        info = nrPUCCHHoppingInfo(cp,nslot,nid,groupHopping,initialCS,seqCS,nIRB);

        % Get the spreading factor in each hop
        if strcmpi(freqHopping,'disabled')
            nSF0 = floor(nPUCCHSym/2);
        else
            nSF0 = floor(nPUCCHSym/4);
        end
        nSF1 = floor(nPUCCHSym/2) - nSF0;

        % Get the low-PAPR sequence
        Msc = Mrb*12; % Total number of subcarriers
        ind = symStart+2:2:14;   % First symbol in PUCCH 1 transmission is for DMRS
        alpha = info.Alpha(:,ind(1:nSF0));
        lps1 = nrLowPAPRS(info.U(1),info.V(1),alpha(:),Msc);

        % Add second half of the sequence when frequency hopping is enabled
        if strcmpi(freqHopping,'enabled')
            lps2 = nrLowPAPRS(info.U(2),info.V(2),info.Alpha(ind(nSF0+1:nSF0+nSF1)),Msc);
            r = [lps1 lps2];
        else
            r = reshape(lps1,[],nSF0);
        end

        % Multiply the sequence with complex-valued symbol
        y = d(1)*r;

        % Check if occi is greater than or equal to nSF0, then error has to be
        % thrown
        coder.internal.errorIf(occi>=nSF0,'nr5g:nrPUCCH:InvalidOCCIPUCCH1',occi,nSF0);

        % Get the orthogonal sequence from spreading factor and orthogonal
        % cover code index
        oSeq1 = nr5g.internal.PUCCH1Spreading(nSF0,occi);
        if strcmpi(freqHopping,'disabled')
            oSeq = oSeq1;
        else
            oSeq2 = nr5g.internal.PUCCH1Spreading(nSF1,occi);
            oSeq = [oSeq1 oSeq2];
        end

        % Get the PUCCH format 1 sequence
        seq = y.*oSeq;
        seq = seq(:);
    end

    % Apply options
    if nargin > 12
        fcnName = 'nrPUCCH1';
        opts = nr5g.internal.parseOptions(fcnName,{'OutputDataType'},varargin{:});
        sym = cast(seq,opts.OutputDataType);
    else
        sym = seq;
    end

end
