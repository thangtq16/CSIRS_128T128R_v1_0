function [info,rinfo] = getULSCHInfo(pusch,tcr,tbs,oack,ocsi1,ocsi2)
%getULSCHInfo Returns the structural information of UCI on PUSCH
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [INFO,RESINFO] = getULSCHInfo(PUSCH,TCR,TBS,OACK,OCSI1,OCSI2) returns
%   the structural information INFO and RESINFO related to the coding
%   aspects and resourcing aspects on the physical uplink shared channel,
%   given the inputs:
%   PUSCH - Physical uplink shared channel configuration object
%   TCR   - Target code rate (0 < TCR < 1)
%   TBS   - Transport block size
%   OACK  - HARQ-ACK payload length
%   OCSI1 - CSI part 1 payload length
%   OCSI2 - CSI part 2 payload length
%
%   INFO contains the following fields:
%   CRC     - CRC polynomial selection ('16' or '24A')
%   CRC2    - CRC polynomial selection for the second codeword ('16' or
%             '24A'). In one-codeword transmission, this field is always
%             ''. This output field only applies to the syntax INFO =
%             nrULSCHInfo(PUSCH,TCR,TBS,OACK,OCSI1,OCSI2).
%   L       - Number of CRC bits (16 or 24)
%   BGN     - LDPC base graph selection (1 or 2)
%   C       - Number of code blocks
%   Lcb     - Number of parity bits per code block (0 or 24)
%   F       - Number of <NULL> filler bits per code block
%   Zc      - Lifting size selection
%   K       - Number of bits per code block after CBS
%   N       - Number of bits per code block after LDPC coding
%   GULSCH  - Number of coded and rate matched UL-SCH data bits
%   GACK    - Number of coded and rate matched HARQ-ACK bits
%   GCSI1   - Number of coded and rate matched CSI part 1 bits
%   GCSI2   - Number of coded and rate matched CSI part 2 bits
%   GACKRvd - Number of reserved bits for HARQ-ACK
%   QdACK   - Number of coded HARQ-ACK symbols per layer (Q'_ACK)
%   QdCSI1  - Number of coded CSI part 1 symbols per layer (Q'_CSI1)
%   QdCSI2  - Number of coded CSI part 2 symbols per layer (Q'_CSI2)
%
%   RESINFO contains the following fields:
%   MULSCH         - Number of resource elements available for data
%                    transmission in each OFDM symbol
%   MUCI           - Number of resource elements available for UCI
%                    transmission in each OFDM symbol
%   PUSCHSymbolSet - 1-based OFDM symbol locations carrying data relative
%                    to the first OFDM symbol of PUSCH allocation
%                    (excluding DM-RS)
%   DMRSSymbolSet  - 1-based OFDM symbol locations carrying DM-RS relative
%                    to the first OFDM symbol of PUSCH allocation
%   PTRSSymbolSet  - 1-based OFDM symbol locations carrying PT-RS relative
%                    to the first OFDM symbol of PUSCH allocation
%   SymbolSet      - 1-based OFDM symbol locations allocated for PUSCH
%                    relative to the first OFDM symbol of PUSCH allocation
%   PRBSet         - Physical resource blocks allocated for PUSCH
%   FrequencyHopping - Frequency hopping

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    % Scalar expansion, if necessary
    if pusch.NumLayers <= 4
        ncw = 1;
    else
        ncw = 2;
    end
    % TBS
    if isscalar(tbs)
        tbsVec = [tbs tbs];
    else
        tbsVec = tbs;
    end
    % TCR
    if isscalar(tcr)
        tcrVec = [tcr tcr];
    else
        tcrVec = tcr;
    end

    % Initialize structures for calculations and output
    cbInfoCellInit = cell(1,ncw);
    cbInfoCell = coder.nullcopy(cbInfoCellInit);
    infoStruct = struct('GULSCH',0,'GACK',0,'GCSI1',0,'GCSI2',0,'GACKRvd',0, ...
                        'QdACK',0,'QdCSI1',0,'QdCSI2',0);
    uciInfo   = coder.nullcopy(infoStruct);
    noUCIInfo = coder.nullcopy(infoStruct);
    initChar = '';
    coder.varsize('initChar',[1,3],[0 1]);
    initNum = zeros(1,ncw);
    info = struct('CRC',initChar,'L',initNum,'BGN',initNum, ...
                  'C',initNum,'Lcb',initNum,'F',initNum,'Zc',initNum, ...
                  'K',initNum,'N',initNum,'GULSCH',initNum,'GACK',initNum, ...
                  'GCSI1',initNum,'GCSI2',initNum,'GACKRvd',initNum, ...
                  'QdACK',initNum,'QdCSI1',initNum,'QdCSI2',initNum);
    fieldList1 = {'CRC','L','BGN','C','Lcb','F','Zc','K','N'};
    fieldList2 = fieldnames(infoStruct);

    % Calculate resource information
    rinfo  = nr5g.internal.pusch.resourcesInfo(nrCarrierConfig,pusch);

    % Calculate UCI codeword information
    [QUCI,NumLayersUCI,QmUCI,~,NumLayersNoUCI,QmNoUCI,~] = nr5g.internal.pusch.getUCIMultiplexInfo(pusch,tcrVec);

    % Loop over codeword(s)
    for n = 1:ncw

        % Calculate information unrelated to UCI
        tbsN = tbsVec(n);
        tcrN = tcrVec(n);
        cbInfoCell{n} = nr5g.internal.getSCHInfo(tbsN,tcrN);

        % Calculated information related to UCI
        CN   = cbInfoCell{n}.C;
        KN   = cbInfoCell{n}.K;
        if n == QUCI+1
            uciInfo = calculateUCIInfo(pusch,rinfo,NumLayersUCI,QmUCI,tbsN,tcrN,CN,KN,oack,ocsi1,ocsi2);
        else
            noUCIInfo = calculateUCIInfo(pusch,rinfo,NumLayersNoUCI,QmNoUCI,tbsN,tcrN,CN,KN,0,0,0);
        end

        % Combine information
        for idx = 1:numel(fieldList1)
            fieldName = fieldList1{idx};
            if strcmpi(fieldName,'CRC')
                if n == 1
                    % Output CRC of the first codeword only
                    info.CRC = cbInfoCell{1}.CRC;
                end
            else
                info.(fieldName)(n) = cbInfoCell{n}.(fieldName);
            end
        end
        for idx = 1:numel(fieldList2)
            fieldName = fieldList2{idx};
            if n == QUCI+1
                info.(fieldName)(n) = uciInfo.(fieldName);
            else
                info.(fieldName)(n) = noUCIInfo.(fieldName);
            end
        end

    end

end

function info = calculateUCIInfo(pusch,rinfo,NumLayersUCI,QmUCI,tbsUCI,tcrUCI,CUCI,KUCI,oack,ocsi1,ocsi2)
%Calculate the UCI information

    % Get the information of resource elements that are available for data
    % in each OFDM symbol, along with OFDM symbol locations used for UCI
    % transmission and DM-RS. The carrier configuration is only used for
    % code generation purposes
    datasymbols = rinfo.PUSCHSymbolSet;
    dmrssymbols = rinfo.DMRSSymbolSet;

    % Initialize some variables
    nlqm = NumLayersUCI*QmUCI;    % Product of number of layers and modulation order
    alpha = double(pusch.UCIScaling);
    rqm = double(tcrUCI)*QmUCI; % Product of target code rate and modulation order

    % Calculate bit capacity of PUSCH (excluding DM-RS and PT-RS)
    E = sum(rinfo.MULSCH);
    G = nlqm*E;

    % Get the first OFDM symbol (l_0) that does not carry DM-RS, after
    % the first DM-RS symbol(s) in the PUSCH transmission
    if ~isempty(dmrssymbols) && any(dmrssymbols(1)<datasymbols)
        l0 = datasymbols(find((dmrssymbols(1)<datasymbols),1)); % 1-based
    else
        % When there are no DM-RS symbol locations in the allocation,
        % the first OFDM symbol considered is the starting symbol of
        % PUSCH allocation
        if ~isempty(datasymbols)
            l0 = datasymbols(1);
        else
            l0 = zeros(0,1);
        end
    end

    % Get the total number of resource elements available for UCI
    % transmission on PUSCH
    s1 = sum(rinfo.MUCI);

    % Get the number of resource elements available for UCI transmission on
    % PUSCH from l_0
    if ~isempty(l0)
        s2 = sum(rinfo.MUCI(l0(1):end));
    else
        s2 = 0;
    end

    % Get the number of coded bits of UL-SCH based on the code block groups
    % transmission. Here, all the code blocks are assumed to be transmitted
    a = ones(CUCI,1);
    numCodedBits = sum(a*KUCI);

    % Get the number of reserved ACK bits, based on the number of
    % HARQ-ACK bits
    if oack <= 2
        % Get the reserved bits when oACK <= 2, according to TS 38.212,
        % Section 6.2.7, Step 1
        oACKrvd = 2;
    else
        oACKrvd = 0;
    end

    % Get the number of coded HARQ-ACK symbols and bits, TS 38.212,
    % Section 6.3.2.4.1.1
    [qDashACK,EuciACK] = rateMatchInfoACK(...
        tbsUCI,oack,pusch.BetaOffsetACK,alpha,numCodedBits,s1,s2,rqm,nlqm);
    [qDashACKrvd,EuciACKRvd] = rateMatchInfoACK(...
        tbsUCI,oACKrvd,pusch.BetaOffsetACK,alpha,numCodedBits,s1,s2,rqm,nlqm);

    % Get the number of coded CSI part 1 symbols and bits, TS 38.212,
    % Section 6.3.2.4.1.2
    if ocsi1
        if oACKrvd
            qDashACKCSI1 = qDashACKrvd;
        else
            qDashACKCSI1 = qDashACK;
        end
        firstTerm = getFirstTermOfFormula(...
            tbsUCI,ocsi1,pusch.BetaOffsetCSI1,s1,numCodedBits,rqm);
        if tbsUCI
            qDashCSI1 = min(firstTerm,ceil(alpha*s1)-qDashACKCSI1);
        else
            if ocsi2
                qDashCSI1 = min(firstTerm,s1-qDashACKCSI1);
            else
                qDashCSI1 = s1-qDashACKCSI1;
            end
        end
        EuciCSI = nlqm*qDashCSI1;
    else
        qDashCSI1 = 0;
        EuciCSI = 0;
    end

    % Get the number of coded CSI part 2 symbols and bits, TS 38.212,
    % Section 6.3.2.4.1.3
    if ocsi2
        qDashACKCSI2 = qDashACK;
        if oack <= 2
            qDashACKCSI2 = 0;
        end
        if tbsUCI
            firstTerm = getFirstTermOfFormula(...
                tbsUCI,ocsi2,pusch.BetaOffsetCSI2,s1,numCodedBits,rqm);
            qDashCSI2 = min(firstTerm,ceil(alpha*s1)-qDashACKCSI2-qDashCSI1);
        else
            qDashCSI2 = s1-qDashACKCSI2-qDashCSI1;
        end
        EuciCSI2 = nlqm*qDashCSI2;
    else
        qDashCSI2 =0;
        EuciCSI2 = 0;
    end

    % Get the bit capacity of UL-SCH
    if tbsUCI
        gULSCH = G - EuciCSI - EuciCSI2 - EuciACK*(oACKrvd==0);
    else
        gULSCH = 0;
    end

    % Combine information
    info.GULSCH  = gULSCH;     % Bit capacity of UL-SCH
    info.GACK    = EuciACK;    % Bit capacity of HARQ-ACK
    info.GCSI1   = EuciCSI;    % Bit capacity of CSI part 1
    info.GCSI2   = EuciCSI2;   % Bit capacity of CSI part 2
    info.GACKRvd = EuciACKRvd; % Bit capacity of reserved HARQ-ACK
    info.QdACK   = qDashACK;   % Symbol capacity of HARQ-ACK
    info.QdCSI1  = qDashCSI1;  % Symbol capacity of CSI part 1
    info.QdCSI2  = qDashCSI2;  % Symbol capacity of CSI part 2

end

function [Qd,E] = rateMatchInfoACK(ulschFlag,oack,beta,alpha,sumKr,s1,s2,rqm,nlqm)
%rateMatchInfoACK Rate matching information of HARQ-ACK on PUSCH

    % Symbol and bit capacity of HARQ-ACK
    if oack
        firstTerm = getFirstTermOfFormula(ulschFlag,oack,beta,s1,sumKr,rqm);
        secondTerm = ceil(alpha*s2);
        Qd = min(firstTerm,secondTerm);
        E = nlqm*Qd;
    else
        Qd = 0;
        E = 0;
    end

end

function val = getFirstTermOfFormula(ulschFlag,ouci,beta,s1,sumKr,rqm)
%getFirstTermOfFormula First term of UCI rate match calculations

    % Value of first term in the formula
    if ulschFlag
        % In the presence of UL-SCH
        val = ceil((double(ouci)+getCRC(ouci))*double(beta)*s1/sumKr);
    else
        % In the absence of UL-SCH
        val = ceil((double(ouci)+getCRC(ouci))*double(beta)/rqm);
    end
end

function L = getCRC(oUCI)
% CRC bits for UCI information for input length oUCI, according to TS
% 38.212, Section 6.3.1.2.1
    if oUCI > 19
        L = 11;
    elseif oUCI > 11
        L = 6;
    else
        L = 0;
    end
end