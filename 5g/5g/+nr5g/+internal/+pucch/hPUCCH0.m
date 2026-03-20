function sym = hPUCCH0(ack,sr,symAllocation,cp,nslot,nid,groupHopping,initialCS,freqHopping,Mrb,nIRB,varargin)
%hPUCCH0 Physical uplink control channel format 0
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    lenACK = length(ack);
    lenSR = length(sr);

    symStart = double(symAllocation(1));    % OFDM symbol index corresponding to start of PUCCH
    nPUCCHSym = double(symAllocation(2));   % Number of symbols allocated for PUCCH

    % If interlacing is enabled, set Mrb = 1 and disable frequency hopping
    % as it is not compatible with interlacing (TS 38.213 Section 9.2.1)
    if ~isempty(nIRB)
        Mrb = 1;
        freqHopping = 'disabled';
    end

    % Return empty output either for empty inputs or for negative SR
    % transmission only.
    if (lenACK==0) && ((lenSR==0) || (sr(1)==0))
        % Empty sequence
        seq = zeros(0,1);
    else

        % Get the possible cyclic shift values for the length of ack input
        csTable = getCyclicShiftTable(lenACK);

        % Get the sequence cyclic shift based on ack and sr inputs
        if lenACK==0
            seqCS = csTable(1,1);
        elseif (lenSR==0) || (sr(1) ==0)
            uciValue = comm.internal.utilities.convertBit2Int(ack,lenACK);
            seqCS = csTable(1,uciValue+1);
        else
            uciValue = comm.internal.utilities.convertBit2Int(ack,lenACK);
            seqCS = csTable(2,uciValue+1);
        end

        % Get the hopping parameters
        info = nrPUCCHHoppingInfo(cp,nslot,nid,groupHopping,initialCS,seqCS(1),nIRB);

        % Get the PUCCH format 0 sequence
        Msc = Mrb*12; % Total number of subcarriers
        alpha = info.Alpha(:,symStart+ (1:nPUCCHSym));

        lps = nrLowPAPRS(info.U(1),info.V(1),alpha(:),Msc);
        if strcmpi(freqHopping,'enabled') && (nPUCCHSym == 2)
            lps1 = nrLowPAPRS(info.U(2),info.V(2),info.Alpha(symStart+nPUCCHSym),Msc);
            seq = [lps(:,1);lps1];
        else
            seq = lps(:);
        end
    end

    % Apply options
    if nargin > 11
        fcnName = 'nrPUCCH0';
        opts = nr5g.internal.parseOptions(fcnName,{'OutputDataType'},varargin{:});
        sym = cast(seq,opts.OutputDataType);
    else
        sym = seq;
    end

end

function csTable = getCyclicShiftTable(len)
%   csTable = getCyclicShiftTable(LEN) provides the possible sequence
%   cyclic shift values based on the length LEN.

    if len == 1
        csTable = [0 6;
                   3 9];
    else
        csTable = [0 3  9 6;
                   1 4 10 7];
    end

end