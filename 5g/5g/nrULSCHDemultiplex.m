function [culsch,cack,ccsi1,ccsi2] = nrULSCHDemultiplex(pusch,tcr,tbs,oack,ocsi1,ocsi2,cws)
%nrULSCHDemultiplex Uplink shared channel data and control demultiplexing
%   [CULSCH,CACK,CCSI1,CCSI2] = nrULSCHDemultiplex(PUSCH,TCR,TBS,OACK,OCSI1,OCSI2,CWS)
%   returns the demultiplexed encoded data and encoded uplink control
%   information (UCI), post the demultiplexing of received codeword(s) to
%   undo the processing described in TS 38.212 Section 6.2.7.
%
%   PUSCH is the physical uplink shared channel configuration object, as
%   described in <a href="matlab:help('nrPUSCHConfig')">nrPUSCHConfig</a> with properties:
%
%   Modulation         - Modulation scheme(s) of codeword(s)
%                        ('QPSK' (default), 'pi/2-BPSK', '16QAM', '64QAM', '256QAM')
%   NumLayers          - Number of transmission layers (1...8) (default 1)
%   MappingType        - Mapping type of physical uplink shared channel
%                        ('A' (default), 'B')
%   SymbolAllocation   - Symbol allocation of physical uplink shared
%                        channel (default [0 14]). This property is a
%                        two-element vector. First element represents the
%                        start of OFDM symbol in a slot. Second element
%                        represents the number of contiguous OFDM symbols
%   PRBSet             - PRBs allocated for physical uplink shared channel
%                        within a BWP (0-based) (default 0:51)
%   TransformPrecoding - Flag to enable transform precoding
%                        (0 (default), 1). 0 indicates that transform
%                        precoding is disabled, and the waveform type is
%                        CP-OFDM. 1 indicates that transform precoding is
%                        enabled, and waveform type is DFT-s-OFDM
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   BetaOffsetACK      - Beta offset of HARQ-ACK (default 20)
%   BetaOffsetCSI1     - Beta offset of CSI part 1 (default 6.25)
%   BetaOffsetCSI2     - Beta offset of CSI part 2 (default 6.25)
%   UCIScaling         - UCI scaling factor (default 1)
%   RNTI               - Radio network temporary identifier (0...65535)
%                        (default 1)
%   DMRS               - PUSCH-specific DM-RS configuration object, as
%                        described in <a href="matlab:help('nrPUSCHDMRSConfig')">nrPUSCHDMRSConfig</a> with properties:
%       DMRSConfigurationType   - DM-RS configuration type (1 (default), 2).
%                                 When transform precoding is enabled, the
%                                 value must be 1
%       DMRSTypeAPosition       - Position of first DM-RS OFDM symbol in a
%                                 slot (2 (default), 3)
%       DMRSLength              - Number of consecutive DM-RS OFDM symbols
%                                 (1 (default), 2). When intra-slot
%                                 frequency hopping is enabled, the value
%                                 must be 1. Value of 1 indicates
%                                 single-symbol DM-RS. Value of 2 indicates
%                                 double-symbol DM-RS
%       DMRSAdditionalPosition  - Maximum number of DM-RS additional
%                                 positions (0...3) (default 0). When
%                                 intra-slot frequency hopping is enabled,
%                                 the value must be either 0 or 1
%       DMRSPortSet             - DM-RS antenna port set (0...11)
%                                 (default []). The default value implies
%                                 that the values are in the range from 0
%                                 to NumLayers-1
%       CustomSymbolSet         - Custom DM-RS symbol locations (0-based)
%                                 (default []). This property is used to
%                                 override the standard defined DM-RS
%                                 symbol locations. Each entry corresponds
%                                 to a single-symbol DM-RS
%       NumCDMGroupsWithoutData - Number of CDM groups without data
%                                 (1...3) (default 2). When transform
%                                 precoding is enabled, the value must be 2
%   EnablePTRS         - Enable or disable the PT-RS configuration
%                        (0 (default), 1)
%   PTRS               - PUSCH-specific PT-RS configuration object, as
%                        described in <a href="matlab:help('nrPUSCHPTRSConfig')">nrPUSCHPTRSConfig</a> with properties:
%       TimeDensity            - PT-RS time density (1 (default), 2, 4)
%    These properties are applicable, when transform precoding is set to 0:
%       FrequencyDensity       - PT-RS frequency density (2 (default), 4)
%       REOffset               - Resource element offset
%                                ('00' (default), '01', '10', '11')
%       PTRSPortSet            - PT-RS antenna port set (default []). The
%                                default value of empty ([]) implies the
%                                value is equal to the lowest DM-RS antenna
%                                port configured
%    These properties are applicable, when transform precoding is set to 1:
%       NumPTRSSamples         - Number of PT-RS samples (2 (default), 4)
%       NumPTRSGroups          - Number of PT-RS groups (2 (default), 4, 8)
%
%   TCR is the target code rate(s) for the codeword(s) in the UL-SCH
%   transmission, specified as a scalar between 0 and 1 for single codeword
%   transmission or a two-element vector for two-codeword transmission.
%
%   TBS is the transport block size(s) for the codeword(s) in the UL-SCH
%   transmission, specified as a nonnegative integer for single codeword
%   transmission or a two-element vector for two-codeword transmission.
%
%   OACK is the payload length of the HARQ-ACK bits, specified as a
%   nonnegative integer. A value of 0 indicates no HARQ-ACK transmission.
%
%   OCSI1 is the payload length of the CSI part 1 bits, specified as a
%   nonnegative integer. A value of 0 indicates no CSI part 1 transmission.
%
%   OCSI2 is the payload length of the CSI part 2 bits, specified as a
%   nonnegative integer. A value of 0 indicates no CSI part 2 transmission.
%
%   CWS is the received log-likelihood ratio (LLR) soft bits, specified as
%   a real-valued column vector, a cell array of one column vector for
%   single codeword transmission, or a cell array of two column vectors for
%   two-codeword transmission. The soft bits can also be specified as [],
%   indicating its absence. The length of codeword(s) must be equal to the
%   bit capacity of the uplink physical shared channel (PUSCH). When
%   processing two-codeword transmission, the codeword on which UCI is
%   multiplexed is determined internally, and the codeword without UCI is
%   not changed.
%
%   This function does not support PUSCH interlacing.
%
%   CULSCH is the coded uplink shared channel (UL-SCH) LLR soft bits. When
%   CWS is specified as a column vector, CULSCH is a column vector. When
%   CWS is specified as a cell array, CULSCH is a cell array.
%
%   CACK, CCSI1, CCSI2 are the coded LLR soft bits of HARQ-ACK, CSI part 1,
%   and CSI part 2 respectively, returned as real-valued column vectors.
%
%   Example:
%   % Perform UCI multiplexing and demultiplexing on PUSCH in the presence
%   % of UL-SCH. Check the demultiplexed outputs.
%
%   % Set the PUSCH parameters
%   nlayers = 1;
%   pusch = nrPUSCHConfig;
%   pusch.NumLayers = nlayers;
%   pusch.FrequencyHopping = 'neither';
%   pusch.BetaOffsetACK = 20;
%   pusch.BetaOffsetCSI1 = 6.25;
%   pusch.BetaOffsetCSI2 = 6.25;
%   pusch.UCIScaling = 1;
%
%   % Set the target code rate, payload lengths of UL-SCH data, HARQ-ACK,
%   % CSI part 1, and CSI part 2
%   tcr = 0.5;
%   oack = 8;
%   tbs = 3848;
%   ocsi1 = 88;
%   ocsi2 = 100;
%
%   % Get the rate matched lengths of data, HARQ-ACK, CSI part 1, and CSI
%   % part 2
%   rmInfo = nrULSCHInfo(pusch,tcr,tbs,oack,ocsi1,ocsi2);
%
%   % Create random payload of data, HARQ-ACK, CSI part 1, and CSI part 2
%   data = randi([0 1],tbs,1);
%   ack  = randi([0 1],oack,1);
%   csi1 = randi([0 1],ocsi1,1);
%   csi2 = randi([0 1],ocsi2,1);
%
%   % Perform encoding of data, ack, csi1, and csi2
%   encUL = nrULSCH;
%   setTransportBlock(encUL,data);
%   culsch = encUL(pusch.Modulation,nlayers,rmInfo.GULSCH,0);
%   cack  = nrUCIEncode(ack,rmInfo.GACK,pusch.Modulation);
%   ccsi1 = nrUCIEncode(csi1,rmInfo.GCSI1,pusch.Modulation);
%   ccsi2 = nrUCIEncode(csi2,rmInfo.GCSI2,pusch.Modulation);
%
%   % Get the codeword
%   cw = nrULSCHMultiplex(pusch,tcr,tbs,culsch,cack,ccsi1,ccsi2);
%
%   % Perform UCI and UL-SCH demultiplexing from codeword cw
%   [rxculsch,rxcack,rxccsi1,rxccsi2] = ...
%   nrULSCHDemultiplex(pusch,tcr,tbs,oack,ocsi1,ocsi2,1-2*double(cw));
%
%   % Check the demultiplexed types
%   isequal(rxculsch<0,culsch)
%   isequal(rxcack<0,cack)
%   isequal(rxccsi1<0,ccsi1)
%   isequal(rxccsi2<0,ccsi2)
%
%   See also nrULSCHMultiplex, nrULSCHInfo, nrPUSCHDecode,
%   nrRateRecoverLDPC, nrRateRecoverPolar, nrUCIDecode, nrULSCHDecoder.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    narginchk(7,7);

    % Input validations and vector/cell expansions
    [tcrVec,tbsVec,cwsCell] = validateInputs(pusch,tcr,tbs,oack,ocsi1,ocsi2,cws);

    % Get the codeword number of the codeword to be multiplexed with UCI,
    % and the number of layers, modulation order of this codeword
    [QUCI,NumLayersUCI,QmUCI,ModUCI,~,~,~] = nr5g.internal.pusch.getUCIMultiplexInfo(pusch,tcrVec);

    % Get the codeword with UCI
    cwUCI = cwsCell{QUCI+1};

    % Assign empty outputs, if codeword is empty
    cwLen = length(cwUCI);
    typecw = class(cwUCI);
    if cwLen
        % Get the rate-match and resource information
        [rmInfo,resInfo] = nr5g.internal.pusch.getULSCHInfo(...
            pusch,tcrVec,tbsVec,oack,ocsi1,ocsi2);
        resInfo.Modulation = ModUCI;
        resInfo.NumLayers = NumLayersUCI;

        % Get the bit capacities of UL-SCH and UCI
        GULSCH = rmInfo.GULSCH(QUCI+1);
        GACK = rmInfo.GACK(QUCI+1);
        GCSI1 = rmInfo.GCSI1(QUCI+1);
        GCSI2 = rmInfo.GCSI2(QUCI+1);
        GACKRvd = rmInfo.GACKRvd(QUCI+1);

        % Get the bit capacity of physical uplink shared channel
        G = sum(resInfo.MULSCH)*double(NumLayersUCI)*QmUCI;

        % Check the length of the codeword
        coder.internal.errorIf(cwLen ~= G,'nr5g:nrULSCHDemultiplex:InvalidCWLen',cwLen,G);

        % Get the locations of UL-SCH and UCI type(s) in the codeword
        [~,info] = nr5g.internal.pusch.dataAndControlMapping(...
                       resInfo,GULSCH,GACK,GCSI1,GCSI2,GACKRvd);

        % Get the locations that are accessed within the codeword
        ulschInd = info.ULSCHIndices;
        csi1Ind = info.CSI1Indices;
        csi2Ind = info.CSI2Indices;
        ackInd = info.ACKIndices;

        % Demultiplex codeword
        % Get coded UL-SCH
        if GULSCH ~= length(ulschInd) && GACKRvd
            % When there is puncturing of UL-SCH bits with ACK bits, figure
            % out the locations punctured by HARQ-ACK and get the coded
            % UL-SCH bits accordingly
            culschTemp = zeros(GULSCH,1,typecw);
            sortULSCHInd = sort([ulschInd;info.ULSCHACKIndices]);
            ulschLogicalInd = ismember(sortULSCHInd,ulschInd);
            culschTemp(ulschLogicalInd) = cwUCI(ulschInd);
        else
            culschTemp = cwUCI(ulschInd);
        end

        % Get coded HARQ-ACK
        cack = cwUCI(ackInd);

        % Get coded CSI part 1
        ccsi1 = cwUCI(csi1Ind);

        % Get coded CSI part 2
        if GCSI2 ~= length(csi2Ind) && GACKRvd
            % When there is puncturing of CSI part 2 bits with ACK bits,
            % figure out the locations punctured by HARQ-ACK and get the
            % coded CSI part 2 bits accordingly
            ccsi2 = zeros(GCSI2,1,typecw);
            sortCSI2Ind = sort([csi2Ind;info.CSI2ACKIndices]);
            csi2LogicalInd = ismember(sortCSI2Ind,csi2Ind);
            ccsi2(csi2LogicalInd) = cwUCI(csi2Ind);
        else
            ccsi2 = cwUCI(csi2Ind);
        end
    else
        % Return empty outputs for empty codeword
        culschTemp = zeros(0,1,typecw);
        cack = zeros(0,1,typecw);
        ccsi1 = zeros(0,1,typecw);
        ccsi2 = zeros(0,1,typecw);
    end

    % Output handling for culsch
    if ~isscalar(cwsCell) % 2-CW
        if QUCI == 0 % UCI on cw q=0
            culsch = {culschTemp,cwsCell{2}};
        else % UCI on cw q=1
            culsch = {cwsCell{1},culschTemp};
        end
    else % 1-CW
        if iscell(cws) % cell input
            culsch = {culschTemp};
        else % vector input
            culsch = culschTemp;
        end
    end

end

function [tcrVec,tbsVec,cwsCell] = validateInputs(pusch,tcr,tbs,oack,ocsi1,ocsi2,cws)
%Validate input and expand to vector/cell if necessary

    % Validate inputs
    fcnName = 'nrULSCHDemultiplex';

    % Common validations
    coder.internal.errorIf(~(isa(pusch,'nrPUSCHConfig') && isscalar(pusch)),...
        'nr5g:nrPUSCH:InvalidPUSCHInput');
    coder.internal.errorIf(pusch.Interlacing,'nr5g:nrULSCHMultiplex:InterlacingNotSupported');
    validateConfig(pusch);
    coder.internal.errorIf(~any(numel(tcr)==[1 2]),'nr5g:nrULSCHInfo:InvalidNumTCR');
    validateattributes(tcr,{'numeric'},{'real','>',0,'<',1},fcnName,'TCR');
    coder.internal.errorIf(~any(numel(tbs)==[1 2]),'nr5g:nrULSCHInfo:InvalidNumTBS');
    validateattributes(tbs,{'numeric'},{'integer','nonnegative'},fcnName,'TBS');
    validateattributes(oack,{'numeric'},{'scalar','integer','nonnegative'},fcnName,'OACK');
    validateattributes(ocsi1,{'numeric'},{'scalar','integer','nonnegative'},fcnName,'OCSI1');
    validateattributes(ocsi2,{'numeric'},{'scalar','integer','nonnegative'},fcnName,'OCSI2');

    % Validations dependent on number of codewords
    is2CW = pusch.NumLayers>4;
    if is2CW && iscell(cws)
        % 2 cw cases
        coder.internal.errorIf(numel(cws)~=2,'nr5g:nrULSCHMultiplex:InvalidCellInputTwoCW',numel(cws));
        for i = 1:2
            if ~isempty(cws{i})
                validateattributes(cws{i},{'double','single'},{'column','real'},fcnName,'CWS');
            else
                validateattributes(cws{i},{'double','single'},{'2d'},fcnName,'CWS');
            end
        end
    else
        % 1 cw cases and extract from cell if necessary
        coder.internal.errorIf(is2CW,'nr5g:nrULSCHMultiplex:InvalidCULSCHTwoCW');
        if iscell(cws)
            coder.internal.errorIf(numel(cws)~=1 && ~isempty(cws{2}),'nr5g:nrULSCHMultiplex:InvalidCellInputOneCW',numel(cws));
            cws1 = cws{1};
        else
            cws1 = cws;
        end
        if ~isempty(cws1)
            validateattributes(cws1,{'double','single'},{'column','real'},fcnName,'CWS');
        else
            validateattributes(cws1,{'double','single'},{'2d'},fcnName,'CWS');
        end
    end

    % Expand into vector/cell
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
    % CWS
    if iscell(cws)
        cwsCell = cws;
    else
        cwsCell = {cws};
    end

end
