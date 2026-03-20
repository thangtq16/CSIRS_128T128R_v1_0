function [cws,info] = nrULSCHMultiplex(pusch,tcr,tbs,culsch,cack,ccsi1,ccsi2)
%nrULSCHMultiplex Uplink shared channel data and control multiplexing
%   [CWS,INFO] = nrULSCHMultiplex(PUSCH,TBS,TCR,CULSCH,CACK,CCSI1,CCSI2)
%   returns the codeword(s) CWS by performing the uplink shared channel
%   (UL-SCH) multiplexing on both CULSCH containing encoded UL-SCH data,
%   and encoded uplink control information (UCI) in CACK, CCSI1, and CCSI2,
%   as defined in TS 38.212 Section 6.2.7. The length of the multiplexed
%   output codeword in CWS is equal to the bit capacity of physical uplink
%   shared channel.
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
%   CULSH is the coded UL-SCH data bits, specified as a binary-valued
%   column vector or a cell array of one column vector for single codeword
%   transmission, or a cell array of two column vectors for two-codeword
%   transmission. When processing two-codeword transmission, UCI is only
%   multiplexed on the codeword with the highest IMCS, or the first
%   codeword if 2 codewords have the same IMCS, as defined in TS 38.212
%   Section 6.2.7.
%
%   CACK is the coded HARQ-ACK bits, specified as a real-valued column
%   vector, or empty ([]) to disable the transmission of HARQ-ACK.
%
%   CCSI1 is the coded CSI part 1 bits, specified as a real-valued column
%   vector, or empty ([]) to disable the transmission of CSI part 1.
%
%   CCSI2 is the coded CSI part 2 bits, specified as a real-valued column
%   vector, or empty ([]) to disable the transmission of CSI part 2.
%
%   The length of CULSCH, CACK, CCSI1, and CCSI2 must be a multiple of
%   the modulation order times the number of transmission layers. When
%   processing two-codeword transmission, the length of CACK, CCSI1 and CCSI2
%   must be a multiple of the modulation order times the number of
%   transmission layers of the codeword on which UCI is multiplexed, and
%   the length of each codeword in CULSCH must be a multiple of their
%   respective modulation order times the number of transmission layers.
%
%   This function does not support PUSCH interlacing.
%
%   CWS is a vector if CULSCH is specified as a vector, a cell array if
%   CULSCH is specified as a cell array. When processing two-codeword
%   transmission, the codeword on which UCI is multiplexed is selected
%   internally, and the codeword without UCI multiplexing is not changed.
%   Note that the output CWS contains zeros, when not enough coded UL-SCH,
%   or coded UCI (HARQ-ACK, CSI part 1, CSI part 2) is present to achieve
%   the bit capacity of physical uplink shared channel. The number
%   of bits reserved for HARQ-ACK transmission, GACKRVD, is calculated
%   internally and compared against the lengths of coded inputs, to
%   determine the processing of HARQ-ACK for rate-matching or puncturing.
%
%   INFO provides the information about the 1-based locations of each
%   type in the codeword on which UCI is multiplexed and the codeword
%   number of this codeword. INFO contains the fields:
%   ULSCHIndices - Locations of coded UL-SCH bits in the codeword
%   CSI1Indices  - Locations of coded CSI part 1 bits in the codeword
%   CSI2Indices  - Locations of coded CSI part 2 bits in the codeword
%   ACKIndices   - Locations of coded HARQ-ACK bits in the codeword
%   UCIXIndices  - Locations of 'x' UCI placeholders in the codeword
%   UCIYIndices  - Locations of 'y' UCI placeholders in the codeword
%   QUCI         - Codeword number of the codeword on which UCI is
%                  multiplexed (0 or 1). For single codeword transmission,
%                  this is always 0.
%
%   Example 1:
%   % Get the multiplexed output of UCI without ULSCH for subsequent
%   % transmission on PUSCH.
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
%   tbs = 0;
%   oack = 8;
%   ocsi1 = 88;
%   ocsi2 = 720;
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
%   % Get the codeword from the coded bits of UL-SCH, and coded UCI type(s)
%   [cw,info] = nrULSCHMultiplex(pusch,tcr,tbs,culsch,cack,ccsi1,ccsi2);
%
%   Example 2:
%   % Use predefined values for coded UL-SCH, coded UCI type (HARQ-ACK, CSI
%   % part 1, CSI part 2) to observe how different types multiplex to form
%   % the codeword.
%
%   % Set the PUSCH parameters
%   pusch = nrPUSCHConfig;
%   pusch.PRBSet = 0:20;
%
%   % Set the target code rate, payload lengths of UL-SCH data, HARQ-ACK,
%   % CSI part 1, and CSI part 2
%   tcr = 0.5;
%   tbs = 100;
%   oack = 3;
%   ocsi1 = 10;
%   ocsi2 = 10;
%
%   % Get the rate matched lengths of data, HARQ-ACK, CSI part 1, and CSI
%   % part 2
%   rmInfo = nrULSCHInfo(pusch,tcr,tbs,oack,ocsi1,ocsi2);
%
%   % Create coded inputs with predefined values
%   culsch = ones(rmInfo.GULSCH(1),1);
%   cack = 2*ones(rmInfo.GACK(1),1);
%   ccsi1 = 3*ones(rmInfo.GCSI1(1),1);
%   ccsi2 = 4*ones(rmInfo.GCSI2(1),1);
%
%   % Get the codeword
%   cw = nrULSCHMultiplex(pusch,tcr,tbs,culsch,cack,ccsi1,ccsi2);
%
%   % Plot the codeword and observe that codeword starts with the
%   % elements of CSI part 1, followed by HARQ-ACK, CSI part 2, and, then
%   % mix of UL-SCH and CSI part 2
%   plot(cw)
%   xlabel('Codeword Indices')
%   ylabel('Codeword Values')
%   title('Multiplexing Operation')
%
%   See also nrULSCHDemultiplex, nrULSCHInfo, nrPUSCH, nrULSCH,
%   nrUCIEncode.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    narginchk(7,7);

    % Input validations and vector/cell expansions
    [tcrVec,tbsVec,culschCell] = validateInputs(pusch,tcr,tbs,culsch,cack,ccsi1,ccsi2);

    % Get the codeword number of the codeword to be multiplexed with UCI,
    % and the number of layers, modulation order of this codeword
    [QUCI,NumLayersUCI,QmUCI,ModUCI,~,~,~] = nr5g.internal.pusch.getUCIMultiplexInfo(pusch,tcrVec);

    % Get the UL-SCH codeword for UCI multiplexing
    culschUCI = culschCell{QUCI+1};

    % Get the number of coded UL-SCH bits and coded UCI type(s) bits
    GULSCH = numel(culschUCI);
    GACK  = numel(cack);
    GCSI1 = numel(ccsi1);
    GCSI2 = numel(ccsi2);

    % Check if the lengths of inputs are in multiple of product of
    % modulation order and number of transmission layers
    nlqm = double(NumLayersUCI)*QmUCI;
    coder.internal.errorIf(fix(GACK/nlqm) ~= (GACK/nlqm),...
        'nr5g:nrULSCHMultiplex:InvalidInputLen','CACK',GACK,nlqm);
    coder.internal.errorIf(fix(GCSI1/nlqm) ~= (GCSI1/nlqm),...
        'nr5g:nrULSCHMultiplex:InvalidInputLen','CCSI1',GCSI1,nlqm);
    coder.internal.errorIf(fix(GCSI2/nlqm) ~= (GCSI2/nlqm),...
        'nr5g:nrULSCHMultiplex:InvalidInputLen','CCSI2',GCSI2,nlqm);
    coder.internal.errorIf((tbsVec(QUCI+1) == 0) && (GULSCH > 0),...
        'nr5g:nrULSCHMultiplex:InvalidCULSCHZeroTBS',GULSCH);

    % Get the rate-match and resource information
    [rmInfo,resInfo] = nr5g.internal.pusch.getULSCHInfo(pusch,tcrVec,tbsVec,2,0,0); % oack: 2, ocsi1: 0, ocsi2: 0
    resInfo.Modulation = ModUCI;
    resInfo.NumLayers = NumLayersUCI;

    % Determine if the ACK payload length (oack) is less than or equal to 2
    GACKRvd = rmInfo.GACKRvd(QUCI+1);
    oACKLessThanEquals2Flag = 0;
    if ~GACK || ((GACKRvd >= GACK) && ((sum(resInfo.MULSCH)*nlqm-GULSCH-GCSI1-GCSI2) <= 0))
        oACKLessThanEquals2Flag = 1;
    end
    if GCSI1 && ~GCSI2 && ~GULSCH && (GACKRvd >= GACK)
        oACKLessThanEquals2Flag = 1;
    end
    if GACK && ~GCSI1 && ~GCSI2 && (GACKRvd >= GACK) && ((sum(resInfo.MULSCH)*nlqm-GULSCH) <= 0)
        oACKLessThanEquals2Flag = 1;
    end
    if GACK && ~GCSI1 && ~GCSI2 && ~GULSCH && (GACKRvd >= GACK)
        oACKLessThanEquals2Flag = 1;
    end
    if ~GULSCH && GCSI1 && GCSI2 && (GACKRvd >= GACK) && (sum(resInfo.MUCI)*nlqm-GCSI1-GCSI2-GACK < 0)
        oACKLessThanEquals2Flag = 1;
    end
    if ~GULSCH && ~GCSI1 && GCSI2 && (GACKRvd >= GACK) && (sum(resInfo.MUCI)*nlqm-GCSI2-GACK < 0)
        oACKLessThanEquals2Flag = 1;
    end

    % Update the number of reserved HARQ-ACK bits based on the flag of ACK
    % payload length
    if ~oACKLessThanEquals2Flag
        GACKRvd = 0; % Set the value to zero when oack is greater than 2
    end

    % Get the codeword mapping with pre-defined UL-SCH and UCI type(s).
    % Also, get the information about the locations of each type in the
    % codeword
    [cwTemp,indInfo] = nr5g.internal.pusch.dataAndControlMapping(...
        resInfo,GULSCH,GACK,GCSI1,GCSI2,GACKRvd);

    % Select output type based on input types
    % Double only if all inputs are of type double, else int8
    int8Flag = any([isa(culschUCI,'int8') isa(cack,'int8') isa(ccsi1,'int8') isa(ccsi2,'int8')]);

    % Get the actual codeword
    if ~isempty(cwTemp)
        if int8Flag
            cw = zeros(size(cwTemp),'int8');
        else
            cw = zeros(size(cwTemp));
        end

        % Validate the length of CULSCH
        cwULSCHLen = length(indInfo.ULSCHIndices)+...
            length(indInfo.ULSCHACKIndices)*oACKLessThanEquals2Flag;
        coder.internal.errorIf(cwULSCHLen > GULSCH,...
            'nr5g:nrULSCHMultiplex:InvalidGULSCH',GULSCH,cwULSCHLen);

        % In case of HARQ-ACK payload less than 2, HARQ-ACK punctures
        % either UL-SCH or CSI part 2. Get the locations of UL-SCH and CSI
        % part 2, to index the coded bits, that are used for actual
        % transmission
        if oACKLessThanEquals2Flag
            if ~isempty(indInfo.ULSCHIndices)
                if ~isempty(indInfo.ULSCHACKIndices)
                    % Find the logical indices to access CULSCH that are
                    % part of actual codeword
                    sortULSCHInd = sort([indInfo.ULSCHIndices;indInfo.ULSCHACKIndices]);
                    ulschLogicalInd = ismember(sortULSCHInd,indInfo.ULSCHIndices);
                else
                    ulschLogicalInd = true(length(indInfo.ULSCHIndices),1);
                end
            else
                ulschLogicalInd = true(0,1);
            end
            if ~isempty(indInfo.CSI2Indices)
                if ~isempty(indInfo.CSI2ACKIndices)
                    % Find the logical indices to access CCSI2 that are
                    % part of actual codeword
                    sortCSI2Ind = sort([indInfo.CSI2Indices;indInfo.CSI2ACKIndices]);
                    csi2LogicalInd = ismember(sortCSI2Ind,indInfo.CSI2Indices);
                else
                    csi2LogicalInd = true(length(indInfo.CSI2Indices),1);
                end
            else
                csi2LogicalInd = true(0,1);
            end
        else
            ulschLogicalInd = true(length(indInfo.ULSCHIndices),1);
            csi2LogicalInd = true(length(indInfo.CSI2Indices),1);
        end

        % Map the actual coded information into the codeword
        cw(indInfo.ULSCHIndices) = culschUCI(ulschLogicalInd);
        cw(indInfo.ACKIndices) = cack(1:length(indInfo.ACKIndices));
        cw(indInfo.CSI1Indices) = ccsi1(1:length(indInfo.CSI1Indices));
        cw(indInfo.CSI2Indices) = ccsi2(csi2LogicalInd);
    else
        if int8Flag
            cw = zeros(0,1,'int8');
        else
            cw = zeros(0,1);
        end
    end

    % Output format handling
    if ~iscell(culsch)
        cws = cw;
    else
        if isscalar(culschCell)
            % single codeword provided
            cws = {cw};
        else
            % 2 codewords provided - must be 2-CW transmission
            if QUCI == 0
                cws = {cw,culschCell{2}}; % UCI on q=0
            else
                cws = {culschCell{1},cw}; % UCI on q=1
            end
        end
    end

    % Get the required information
    info = struct;
    info.ULSCHIndices = indInfo.ULSCHIndices;
    info.ACKIndices   = indInfo.ACKIndices;
    info.CSI1Indices  = indInfo.CSI1Indices;
    info.CSI2Indices  = indInfo.CSI2Indices;
    info.UCIXIndices  = uint32(find(cw == -1));
    info.UCIYIndices  = uint32(find(cw == -2));
    info.QUCI         = QUCI;

end

function [tcrVec,tbsVec,culschCell] = validateInputs(pusch,tcr,tbs,culsch,cack,ccsi1,ccsi2)
%Validate input and expand to vector/cell if necessary

    fcnName = 'nrULSCHMultiplex';

    % Common validations
    coder.internal.errorIf(~(isa(pusch,'nrPUSCHConfig') && isscalar(pusch)),...
        'nr5g:nrPUSCH:InvalidPUSCHInput');
    coder.internal.errorIf(pusch.Interlacing,'nr5g:nrULSCHMultiplex:InterlacingNotSupported');
    validateConfig(pusch);
    coder.internal.errorIf(~any(numel(tcr) == [1 2]),'nr5g:nrULSCHInfo:InvalidNumTCR');
    validateattributes(tcr,{'numeric'},{'real','>',0,'<',1},fcnName,'TCR');
    coder.internal.errorIf(~any(numel(tbs) == [1 2]),'nr5g:nrULSCHInfo:InvalidNumTBS');
    validateattributes(tbs,{'numeric'},{'integer','nonnegative'},fcnName,'TBS');
    validateInputWithEmpty(cack,{'double','int8'},{'column','real'},fcnName,'CACK');
    validateInputWithEmpty(ccsi1,{'double','int8'},{'column','real'},fcnName,'CCSI1');
    validateInputWithEmpty(ccsi2,{'double','int8'},{'column','real'},fcnName,'CCSI2');

    % Validations dependent on number of codewords
    is2CW = pusch.NumLayers>4;
    if is2CW && iscell(culsch)
        % 2 cw cases
        coder.internal.errorIf(numel(culsch)~=2,'nr5g:nrULSCHMultiplex:InvalidCellInputTwoCW',numel(culsch));
        for i = 1:2
            validateInputWithEmpty(culsch{i},{'double','int8'},{'column','real'},fcnName,'CULSCH');
        end
    else
        % 1 cw cases and extract from cell if necessary
        coder.internal.errorIf(is2CW,'nr5g:nrULSCHMultiplex:InvalidCULSCHTwoCW');
        if iscell(culsch)
            coder.internal.errorIf(numel(culsch)~=1 && ~isempty(culsch{2}),'nr5g:nrULSCHMultiplex:InvalidCellInputOneCW',numel(culsch));
            culsch1 = culsch{1};
        else
            culsch1 = culsch;
        end
        validateInputWithEmpty(culsch1,{'double','int8'},{'column','real'},fcnName,'CULSCH');
    end

    % Expand to vector/cell
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
    % CULSCH
    if iscell(culsch)
        culschCell = culsch;
    else
        culschCell = {culsch};
    end

end

function validateInputWithEmpty(in,classes,attributes,fcnName,varname)
%Validates input with possible empty data

    if ~isempty(in)
        % Check for type and attributes
        validateattributes(in,classes,attributes,fcnName,varname);
    else
        % Check for type when input is empty
        validateattributes(in,classes,{'2d'},fcnName,varname);
    end

end
