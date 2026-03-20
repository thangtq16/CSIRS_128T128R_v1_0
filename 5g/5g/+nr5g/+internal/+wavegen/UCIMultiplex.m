function [out,info] = UCIMultiplex(carrier,pucch,uciPart1,uciPart2)
%UCIMultiplex UCI multiplexing on physical uplink control channel for formats 3 and 4
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [OUT,INFO] = UCIMultiplex(CARRIER,PUCCH,UCIPART1,UCIPART2) provides the
%   multiplexed output OUT for UCIPART1 and UCIPART2 as per TS 38.212,
%   Section 6.3.1.6. The output INFO is a structure containing the 1-based
%   indices of UCI part 1 (INFO.UCI1Indices) and UCI part 2
%   (INFO.UCI2Indices).
%
%   The inputs are:
%   CARRIER  - Carrier configuration
%   PUCCH    - Uplink control channel configuration
%   UCIPART1 - Coded UCI bits that correspond to CSI part 1, with or without
%              HARQ-ACK and SR bits. Use empty ([]) to indicate no UCI part
%              1 transmission
%   UCIPART2 - Coded UCI bits that correspond to CSI part 2. Use empty ([])
%              to indicate no UCI part 2 transmission
%
%   If both UCIPART1 and UCIPART2 are nonempty, the length of the
%   multiplexed codeword OUT is the product between the modulation order
%   (qm), the number of resource elements for each OFDM symbol carrying UCI
%   information (nSymbolUCI), and the number of OFDM symbols carrying UCI
%   information (nPUCCHSymUCI), as discussed in TS 38.212 Section 6.3.1.6.
%   For PUCCH format 3, nSymbolUCI is the number of subcarriers allocated
%   for this PUCCH. For PUCCH format 4, nSymbolUCI is the ratio between the
%   the number of subcarriers allocated for this PUCCH and the spreading
%   factor. The number of OFDM symbols carrying UCI information
%   (nPUCCHSymUCI) is the difference between the number of OFDM symbols
%   allocated for PUCCH and the number of OFDM symbols allocated for DM-RS.
%
%   Example:
%   % Perform UCI multiplexing of UCI part 1 and part 2 for format 3 with
%   % rate matched lengths 874 and 1070 respectively, for QPSK modulation
%   % with 11 OFDM symbols allocated for PUCCH, 9 resource blocks, and
%   % symbols 2 and 7 allocated for DM-RS.
%
%   pucch3 = nrPUCCH3Config;
%   pucch3.PRBSet = 0:8;
%   pucch3.SymbolAllocation = [2 11];
%   uci1 = -1*ones(874,1);
%   uci2 = -2*ones(1070,1);
%   cw = nr5g.internal.wavegen.UCIMultiplex(pucch3,uci1,uci2);
%
%   % Check the number of bits that are not uci1 or uci2
%   nnz(~((cw==-1)|(cw==-2)))
%
%   See also nrWaveformGenerator, nrPUCCH3Config, nrPUCCH4Config,
%   nrWavegenPUCCH3Config, nrWavegenPUCCH4Config, nrPUCCH.

% Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    % Initialize info output
    info = struct;
    info.UCI1Indices = uint32(zeros(0,1));
    info.UCI2Indices = uint32(zeros(0,1));
    coder.varsize('info.UCI1Indices','info.UCI2Indices');
    
    % Get the input lengths
    g1 = numel(uciPart1);
    g2 = numel(uciPart2);

    % If UCI part 1 is empty, return an empty output
    if g1==0
        out = zeros(0,1,'like',uciPart1);
        return;
    end
    
    % If no UCI part 2, codeword is the encoded UCI part 1
    if g2==0 || isa(pucch,'nrPUCCH2Config')
        out = uciPart1;
        return;
    end
    
    % Get the number of UCI symbols
    prbset = nr5g.internal.allocatedPhysicalResourceBlocks(carrier, pucch);
    nSymbolUCI = 12*numel(prbset);

    % Scale by spreading factor if needed
    sf = nr5g.internal.pucch.occConfiguration(pucch);
    if ~isempty(sf)
        nSymbolUCI = nSymbolUCI/sf(1);
    end
    
    % Modulation order
    qm = nr5g.internal.getQm(pucch.Modulation);

    % Get the set of symbol indices based on DM-RS configuration and the
    % number of symbols allocated for PUCCH transmission
    interlacing = nr5g.internal.interlacing.isInterlaced(pucch);
    freqHopping = ~interlacing && strcmpi(pucch.FrequencyHopping,'intraSlot');
    dmrsSym = nr5g.internal.pucch.dmrsSymbolIndicesFormats34(pucch.SymbolAllocation,...
        freqHopping,pucch.AdditionalDMRS);
    [nset,s1,s2,s3] = getSymbolSets(pucch.SymbolAllocation(2),dmrsSym-pucch.SymbolAllocation(1));

    % Get the number of UCI OFDM symbols in each set
    nUCI = [numel(s1) numel(s2) numel(s3)];

    % Get the total number of OFDM symbols carrying UCI
    nPUCCHSymUCI = sum(nUCI);

    % Cumulative sum of number of UCI OFDM symbols in each set
    cSumNUCI = cumsum(nUCI);

    % Find the set index that covers uciPart1
    j = 0;
    for i = 1:nset
        if cSumNUCI(i)*nSymbolUCI*qm >= g1
            j = i;
            break;
        end
    end

    % If the UCI part1 length is more than the total number of REs
    % allocated
    if ~j
        j = nset;
    end

    % Find nBarSymbolUCI and M
    jMinus1 = j-1; % j minus 1
    if j == 1
        temp = 0;
    else
        temp = cSumNUCI(jMinus1)*nSymbolUCI*qm;
    end
    nBarSymbolUCI = floor((g1-temp)/(nUCI(jMinus1+1)*qm));
    M = mod((g1-temp)/qm,nUCI(jMinus1+1));

    % Get the sets to perform multiplexing
    uciset = sort([s1 s2 s3]);
    sets = {s1,s2,s3};
    setJminus1 = [];
    for i = 1:jMinus1
        setJminus1 = [setJminus1 sets{i}]; %#ok
    end

    % Initialize the intermediate value gBar
    if nBarSymbolUCI > nSymbolUCI
        gBar = zeros(nPUCCHSymUCI,nBarSymbolUCI+(M>0),qm,'like',uciPart1);
    else
        gBar = zeros(nPUCCHSymUCI,nSymbolUCI,qm,'like',uciPart1);
    end
    gBar1 = gBar;

    % Perform multiplexing
    n1 = 0;
    n2 = 0;
    for l = 0:nPUCCHSymUCI-1

        % Get the symbol index from the set of UCI symbols
        symIndex = uciset(l+1);

        % Check for the set which contains the symbol index and account
        % the UCI
        if ismember(symIndex,setJminus1)
            % If symbol index belongs to the union of sets up to j-1, UCI
            % part 1 is accounted
            for k = 0:nSymbolUCI-1
                for v = 0:qm-1
                    if n1 < g1 % To avoid out of bounds indexing
                        gBar(l+1,k+1,v+1) = uciPart1(n1+1);
                        gBar1(l+1,k+1,v+1) = -1; % Map -1 for UCI part 1
                    end
                    n1 = n1+1;
                end
            end
        elseif ismember(symIndex,sets{j})
            % If symbol index belongs to set j, both UCI part 1 and UCI
            % part 2 are accounted
            if M > 0
                gamma = 1;
            else
                gamma = 0;
            end
            M = M-1;
            for k = 0:nBarSymbolUCI+gamma-1
                for v = 0:qm-1
                    if n1 < g1 % To avoid out of bounds indexing
                        gBar(l+1,k+1,v+1) = uciPart1(n1+1);
                        gBar1(l+1,k+1,v+1) = -1; % Map -1 for UCI part 1
                    end
                    n1 = n1+1;
                end
            end
            for k = nBarSymbolUCI+gamma:nSymbolUCI-1
                for v = 0:qm-1
                    if n2 < g2 % To avoid out of bounds indexing
                        gBar(l+1,k+1,v+1) = uciPart2(n2+1);
                        gBar1(l+1,k+1,v+1) = -2; % Map -2 for UCI part 2
                    end
                    n2 = n2+1;
                end
            end
        else
            % If symbol index does not belong to union of sets j, UCI part
            % 2 is accounted
            for k = 0:nSymbolUCI-1
                for v = 0:qm-1
                    if n2 < g2 % To avoid out of bounds indexing
                        gBar(l+1,k+1,v+1) = uciPart2(n2+1);
                        gBar1(l+1,k+1,v+1) = -2; % Map -2 for UCI part 2
                    end
                    n2 = n2+1;
                end
            end
        end
    end

    % Return multiplexed output and structural information
    out = reshape(permute(gBar(1:nPUCCHSymUCI,1:nSymbolUCI,1:qm),[3 2 1]),[],1);
    tmp = reshape(permute(gBar1(1:nPUCCHSymUCI,1:nSymbolUCI,1:qm),[3 2 1]),[],1);
    info.UCI1Indices = uint32(find(tmp == -1));
    info.UCI2Indices = uint32(find(tmp == -2));

end

function [nset,s1,s2,s3] = getSymbolSets(nPUCCHSym,dmrsSym)
%getSymbolSets provides the sets of symbols according to TS 38.212 Table
%   6.3.1.6-1, with the two inputs, number of symbols allocated for PUCCH
%   and the PUCCH DM-RS symbol indices. The valid combinations are:
%
%       PUCCH duration               PUCCH DM-RS symbol indices
%       --------------               -------------------------
%             4                            [1] or [0 2]
%             5                               [0 3]
%             6                               [1 4]
%             7                               [1 4]
%             8                               [1 5]
%             9                               [1 6]
%            10                          [2 7] or [1 3 6 8]
%            11                          [2 7] or [1 3 6 9]
%            12                          [2 8] or [1 4 7 10]
%            13                          [2 9] or [1 4 7 11]
%            14                         [3 10] or [1 5 8 12]
%
%   The PUCCH DM-RS symbol indices above are relative to the start of PUCCH
%   symbol.

    % Initialize the outputs
    nset = 0;
    s1 = zeros(1,0);
    s2 = zeros(1,0);
    s3 = zeros(1,0);
    coder.varsize('s1','s2','s3', [1 8], [0 1]);

    % Get the outputs based on valid combinations
    switch nPUCCHSym
        case 4
            if dmrsSym == 1
                nset = 2;
                s1 = [0 2];
                s2 = 3;
            elseif isequal(dmrsSym, [0 2])
                nset = 1;
                s1 = [1 3];
            end
        case 5
            if isequal(dmrsSym, [0 3])
                nset = 1;
                s1 = [1 2 4];
            end
        case 6
            if isequal(dmrsSym, [1 4])
                nset = 1;
                s1 = [0 2 3 5];
            end
        case 7
            if isequal(dmrsSym, [1 4])
                nset = 2;
                s1 = [0 2 3 5];
                s2 = 6;
            end
        case 8
            if isequal(dmrsSym, [1 5])
                nset = 2;
                s1 = [0 2 4 6];
                s2 = [3 7];
            end
        case 9
            if isequal(dmrsSym, [1 6])
                nset = 2;
                s1 = [0 2 5 7];
                s2 = [3 4 8];
            end
        case 10
            if isequal(dmrsSym, [2 7])
                nset = 2;
                s1 = [1 3 6 8];
                s2 = [0 4 5 9];
            elseif isequal(dmrsSym, [1 3 6 8])
                nset = 1;
                s1 = [0 2 4 5 7 9];
            end
        case 11
            if isequal(dmrsSym, [2 7])
                nset = 3;
                s1 = [1 3 6 8];
                s2 = [0 4 5 9];
                s3 = 10;
            elseif isequal(dmrsSym, [1 3 6 9])
                nset = 1;
                s1 = [0 2 4 5 7 8 10];
            end
        case 12
            if isequal(dmrsSym, [2 8])
                nset = 3;
                s1 = [1 3 7 9];
                s2 = [0 4 6 10];
                s3 = [5 11];
            elseif isequal(dmrsSym, [1 4 7 10])
                nset = 1;
                s1 = [0 2 3 5 6 8 9 11];
            end
        case 13
            if isequal(dmrsSym, [2 9])
                nset = 3;
                s1 = [1 3 8 10];
                s2 = [0 4 7 11];
                s3 = [5 6 12];
            elseif isequal(dmrsSym, [1 4 7 11])
                nset = 2;
                s1 = [0 2 3 5 6 8 10 12];
                s2 = 9;
            end
        otherwise % nPUCCHSym equal to 14
            if isequal(dmrsSym, [3 10])
                nset = 3;
                s1 = [2 4 9 11];
                s2 = [1 5 8 12];
                s3 = [0 6 7 13];
            elseif isequal(dmrsSym, [1 5 8 12])
                nset = 2;
                s1 = [0 2 4 6 7 9 11 13];
                s2 = [3 10];
            end
    end

end
