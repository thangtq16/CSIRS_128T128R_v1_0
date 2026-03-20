function [uciBits,symbols,detMet] = nrPUCCHDecode(carrier,pucch,ouci,sym,varargin)
%nrPUCCHDecode Physical uplink control channel decoding
%   [UCIBITS,SYMBOLS,DETMET] = nrPUCCHDecode(...) performs physical uplink
%   control channel format-specific decoding and returns the cell array of
%   uplink control information (UCI) bits, UCIBITS. The function also
%   returns a vector of received constellation symbols, SYMBOLS, and the
%   detection metric, DETMET. When the number of UCI bits is less than 12,
%   the function performs discontinuous transmission (DTX) detection by
%   finding the normalized correlation coefficient of all the possible
%   reference sequences and then compares the maximum value against a
%   threshold. After the DTX detection, the function:
%   * Returns the hard UCI bits of type int8 for format 0, resulting from
%     the UCI bits leading to maximum normalized correlation coefficient.
%   * Returns the hard UCI bits of type int8 for format 1, resulting from
%     matched filtering and symbol demodulation.
%   * Returns soft UCI bits for formats 2, 3, and 4, resulting from the
%     inverse operation of physical uplink control channel processing, as
%     defined in TS 38.211 Sections 6.3.2.5 and 6.3.2.6.
%
%   [UCIBITS,SYMBOLS,DETMET] = nrPUCCHDecode(CARRIER,PUCCH,OUCI,SYM)
%   performs physical uplink control channel decoding for the specified
%   carrier configuration CARRIER, physical uplink control channel
%   configuration PUCCH, and received symbols SYM. CARRIER is a scalar
%   nrCarrierConfig object. For physical uplink control channel formats 0,
%   1, 2, 3, and 4, PUCCH is a scalar nrPUCCH0Config, nrPUCCH1Config,
%   nrPUCCH2Config, nrPUCCH3Config, and nrPUCCH4Config, respectively. OUCI
%   is a scalar or two-element vector representing the number of UCI bits.
%   For format 0, the first element of OUCI represents the number of
%   HARQ-ACK bits, and second element represents the number of SR bits. For
%   format 1, OUCI is a scalar representing the number of hybrid automatic
%   repeat request acknowledgment (HARQ-ACK) or scheduling request (SR)
%   bits. For all other formats, OUCI represents the number of UCI bits in
%   both the UCI parts. For formats 0 and 1, SYM is a matrix with number of
%   columns equal to the number of receive antennas. For all other formats,
%   SYM is a column vector. To detect the transmission of SR on format 1
%   without any HARQ-ACK bits, set OUCI to 1 and ensure UCIBITS contain
%   zero. This syntax assumes detection threshold of:
%   * 0.49 and 0.42, for format 0 with 1 and 2 OFDM symbols, respectively.
%   * 0.22 for format 1.
%   * 0.45 for formats 2, 3, and 4.
%
%   CARRIER is a carrier configuration object, as described in <a
%   href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a>.
%   Only these object properties are relevant for this function:
%
%   NCellID             - Physical layer cell identity (0...1007) (default 1)
%   SubcarrierSpacing   - Subcarrier spacing (SCS) in kHz
%                         (15 (default), 30, 60, 120, 240, 480, 960)
%   CyclicPrefix        - Cyclic prefix ('normal' (default), 'extended')
%   NSizeGrid           - Number of resource blocks in carrier resource
%                         grid (1...275) (default 52)
%   NStartGrid          - Start of carrier resource grid relative to common
%                         resource block 0 (CRB 0) (0...2199) (default 0)
%   NSlot               - Slot number (default 0)
%   IntraCellGuardBands - Intracell guard bands (default [])
%
%   For format 0, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH0Config')">nrPUCCH0Config</a>. Only these
%   object properties are relevant for this function:
%
%   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
%                        (default [13 1])
%   PRBSet             - PRBs allocated for PUCCH in the BWP (default 0)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%   GroupHopping       - Group hopping configuration
%                        ('neither' (default), 'enable', 'disable')
%   HoppingID          - Hopping identity (0...1023) (default [])
%   InitialCyclicShift - Initial cyclic shift (0...11) (default 0)
%
%   For format 1, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH1Config')">nrPUCCH1Config</a>. Only these
%   object properties are relevant for this function:
%
%   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
%                        (default [0 14])
%   PRBSet             - PRBs allocated for PUCCH in the BWP (default 0)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%   GroupHopping       - Group hopping configuration
%                        ('neither' (default), 'enable', 'disable')
%   HoppingID          - Hopping identity (0...1023) (default [])
%   InitialCyclicShift - Initial cyclic shift (0...11) (default 0)
%   OCCI               - Orthogonal cover code index (0...6) (default 0)
%
%   For format 2, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH2Config')">nrPUCCH2Config</a>. Only these
%   object properties are relevant for this function:
%
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%   SpreadingFactor    - Spreading factor (2 (default), 4)
%   OCCI               - Orthogonal cover code index (0...3) (default 0)
%   NID                - Data scrambling identity (0...1023) (default [])
%   RNTI               - Radio network temporary identifier (0...65535)
%                        (default 1)
%
%   For format 3, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH3Config')">nrPUCCH3Config</a>. Only these
%   object properties are relevant for this function:
%
%   Modulation         - Modulation scheme ('QPSK' (default), 'pi/2-BPSK')
%   PRBSet             - PRBs allocated for PUCCH in the BWP (default 0)
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%   SpreadingFactor    - Spreading factor (1, 2 (default), 4)
%   OCCI               - Orthogonal cover code index (0...3) (default 0)
%   NID                - Data scrambling identity (0...1023) (default [])
%   RNTI               - Radio network temporary identifier (0...65535)
%                        (default 1)
%
%   For format 4, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH4Config')">nrPUCCH4Config</a>. Only these
%   object properties are relevant for this function:
%
%   Modulation         - Modulation scheme ('QPSK' (default), 'pi/2-BPSK')
%   PRBSet             - PRBs allocated for PUCCH in the BWP (default 0)
%   SpreadingFactor    - Spreading factor (2 (default), 4)
%   OCCI               - Orthogonal cover code index (0...6) (default 0)
%   NID                - Data scrambling identity (0...1023) (default [])
%   RNTI               - Radio network temporary identifier (0...65535)
%                        (default 1)
%
%   [UCIBITS,SYMBOLS,DETMET] = nrPUCCHDecode(...,NVAR) allows the variance
%   of additive white Gaussian noise (AWGN) on the received symbols to be
%   specified by the parameter NVAR, a nonnegative real scalar. The default
%   value is 1e-10. When the value is less than 1e-10, the function uses
%   the value of 1e-10.
%
%   [UCIBITS,SYMBOLS,DETMET] = nrPUCCHDecode(...,NAME,VALUE) specifies an
%   additional option as a NAME,VALUE pair to allow control over the
%   detection threshold:
%
%   'DetectionThreshold' - Specifies the detection threshold as a real
%                          number in the range [0, 1]. When this input is
%                          not present or set to [], the function selects a
%                          default value. For format 0, the default value
%                          is 0.49 for 1 OFDM symbol and 0.42 for 2 OFDM
%                          symbols. For format 1, the default value is
%                          0.22. For other formats, the default value is
%                          0.45.
%
%   For PUCCH formats 0 to 3 and operation with shared spectrum channel
%   access for FR1, set Interlacing = true and specify the allocated
%   frequency resources using the RBSetIndex and InterlaceIndex properties
%   of the PUCCH configuration. The PRBSet and FrequencyHopping properties
%   are ignored. For PUCCH formats 2 and 3, you can specify the
%   SpreadingFactor and OCCI for single-interlace configurations.
%
%   Example 1:
%   % Decode the symbols that transmitted positive SR using PUCCH format 0.
%
%   ack = zeros(0,1);
%   sr = 1;
%
%   % Set carrier parameters
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15;
%   carrier.CyclicPrefix = 'normal';
%   carrier.NSlot = 63;
%
%   % Set PUCCH format 0 parameters
%   pucch = nrPUCCH0Config;
%   pucch.SymbolAllocation = [11 2];
%   pucch.HoppingID = 512;
%   pucch.GroupHopping = 'enable';
%   pucch.InitialCyclicShift = 5;
%   pucch.FrequencyHopping = 'intraSlot';
%
%   % Get PUCCH format 0 symbols
%   sym = nrPUCCH(carrier,pucch,{ack sr});
%
%   % Decode PUCCH format 0 symbols
%   uci = nrPUCCHDecode(carrier,pucch,[numel(ack) numel(sr)],sym);
%   isequal(uci{1},ack)
%   isequal(uci{2},sr)
%
%   Example 2:
%   % Decode the PUCCH format 1 modulated symbols for a 60 kHz SCS carrier
%   % and with a detection threshold of 0.5 for 1-bit UCI.
%
%   % Set carrier parameters
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 60;
%   carrier.CyclicPrefix = 'extended';
%   carrier.NSlot = 7;
%
%   % Set PUCCH format 1 parameters
%   pucch1 = nrPUCCH1Config;
%   pucch1.SymbolAllocation = [3 9];
%   pucch1.FrequencyHopping = 'intraSlot';
%   pucch1.GroupHopping = 'enable';
%   pucch1.HoppingID = 512;
%   pucch1.InitialCyclicShift = 9;
%   pucch1.OCCI = 1;
%
%   % Get PUCCH format 1 symbols
%   uci = 1;
%   sym = nrPUCCH(carrier,pucch1,uci);
%
%   % Decode PUCCH format 1 symbols
%   rxUCI = nrPUCCHDecode(carrier,pucch1,numel(uci),sym,'DetectionThreshold',0.5);
%   isequal(rxUCI{1},uci)
%
%   Example 3:
%   % Generate and decode the PUCCH format 2 symbols with cell identity as
%   % 148 and radio network temporary identifier as 160.
%
%   % Set carrier parameters
%   carrier = nrCarrierConfig;
%   carrier.NCellID = 148;
%
%   % Set PUCCH format 2 parameters
%   pucch2 = nrPUCCH2Config;
%   pucch2.NID = [];
%   pucch2.RNTI = 160;
%
%   % Get random UCI bits
%   ouci = 20;
%   uci = randi([0 1],ouci,1);
%
%   % Encode UCI and get PUCCH format 2 symbols
%   uciCW = nrUCIEncode(uci,100);
%   sym = nrPUCCH(carrier,pucch2,uciCW);
%
%   % Decode the PUCCH format 2 symbols
%   rxUCI = nrPUCCHDecode(carrier,pucch2,ouci,sym);
%   isequal(uciCW,double(rxUCI{1} < 0))
%
%   See also nrPUCCH, nrPUCCHIndices, nrUCIDecode, nrPUCCH0Config,
%   nrPUCCH1Config, nrPUCCH2Config, nrPUCCH3Config, nrPUCCH4Config.

% Copyright 2021-2023 The MathWorks, Inc.

%#codegen

    narginchk(4,7);

    % Parse and validate inputs
    [formatPUCCH,numUCIElements,nVar,thres,fcnName] = ...
        parseAndValidateInputs(carrier,pucch,ouci,sym,varargin{:});

    % Determine the number of RB allocated
    Mrb = numel(nr5g.internal.allocatedPhysicalResourceBlocks(carrier,pucch));

    % PUCCH decoding
    emptyFlag = Mrb == 0 || isempty(pucch.SymbolAllocation) || ...
        (pucch.SymbolAllocation(2) == 0) || isempty(sym);
    dtType = class(sym);
    outDtTypeF01 = 'int8';
    detMet = zeros(1,1,dtType);
    if emptyFlag && (formatPUCCH ~= 0)
        % When there is no PUCCH resource allocation or no input symbols,
        % return empty UCI bits, empty UCI symbols, and 0 for detection
        % metric
        if formatPUCCH == 1
            uciBits = {zeros(0,1,outDtTypeF01)};
        else
            uciBits = {zeros(0,1,dtType)};
        end
        symbols = zeros(0,1,dtType);
    else
        % Decode the PUCCH symbols, depending on the PUCCH format
        switch formatPUCCH
            case 0
                % Perform OUCI validation
                if numUCIElements > 1
                    validateattributes(ouci(2),{'numeric'},...
                        {'<=',1,'integer'},fcnName,'second element of OUCI');
                    oack = ouci(1);
                    osr = double(ouci(2));
                else
                    oack = ouci;
                    osr = 0;
                end
                % Perform PUCCH format 0 decoding
                if emptyFlag
                    rxSR = zeros(osr,1,outDtTypeF01);
                    uciBits = {zeros(0,1,outDtTypeF01) rxSR};
                else
                    [uciBits,detMet] = nr5g.internal.pucch.decodeFormat0(...
                        carrier,pucch,[oack osr],sym,thres);
                end
                % Assign the cell array output depending on the number of
                % elements in OUCI
                if numUCIElements == 1
                    uciBits = {uciBits{1}};
                end
                % Assign empty symbols output
                symbols = zeros(0,1,dtType);
            case 1
                % Perform PUCCH format 1 decoding
                [uciBits,symbols,detMet] = nr5g.internal.pucch.decodeFormat1(...
                    carrier,pucch,ouci(1),sym,nVar,thres);
            case 2
                % Perform PUCCH format 2 decoding
                [uciBits,symbols,detMet] = nr5g.internal.pucch.decodeFormat2(...
                    carrier,pucch,ouci(1),sym,nVar,thres);
            case 3
                % Perform PUCCH format 3 decoding
                [uciBits,symbols,detMet] = nr5g.internal.pucch.decodeFormat3(...
                    carrier,pucch,ouci(:),sym,nVar,thres);
            otherwise
                % Perform PUCCH format 4 decoding
                [uciBits,symbols,detMet] = nr5g.internal.pucch.decodeFormat4(...
                    carrier,pucch,ouci(:),sym,nVar,thres);
        end
    end

end

% Parse and validate inputs
function [formatPUCCH,numUCIElements,nVar,thres,fcnName] = ...
    parseAndValidateInputs(carrier,pucch,ouci,sym,varargin)

    % Validate input configuration objects
    fcnName = 'nrPUCCHDecode';
    formatPUCCH = nr5g.internal.pucch.validateInputObjects(carrier,pucch);

    % Validate OUCI
    numUCIElements = numel(ouci);
    validateattributes(ouci,{'numeric'},...
        {'real','vector','nonnegative','integer','nonempty'},fcnName,'OUCI');
    if numUCIElements > 1
        validateattributes(ouci,{'numeric'},{'numel',2 - (formatPUCCH == 1)},fcnName,'OUCI');
    end
    if formatPUCCH > 1
        % For PUCCH formats 2, 3, and 4, first element of OUCI must be 0 or
        % greater than 2
        if any(ouci(1) == [1 2])
            validateattributes(ouci(1),{'numeric'},...
                {'>=',3,'scalar'},fcnName,'first element of OUCI');
        end
    else
        % For PUCCH formats 0 and 1, first element of OUCI must be 0, 1, or 2
        validateattributes(ouci(1),{'numeric'},...
            {'<=',2,'scalar'},fcnName,'first element of OUCI');
    end

    % Validate SYM
    if ~isempty(sym)
        validateattributes(sym,{'double','single'},{'2d','finite'},fcnName,'SYM');
        if formatPUCCH > 1
            % For PUCCH formats 2, 3, and 4, SYM must be a column vector
            validateattributes(sym,{'double','single'},{'column','finite'},fcnName,'SYM');
        end
    else
        validateattributes(sym,{'double','single'},{'2d'},fcnName,'SYM');
    end

    % Parse optional inputs
    if any(nargin == [5 7])
        nVar = varargin{1};
        validateattributes(nVar,{'double','single'},{'scalar','real',...
            'nonnegative','nonnan','finite'},fcnName,'NVAR');
        if nargin == 7
            opts = nr5g.internal.parseOptions(fcnName,...
                {'DetectionThreshold'},varargin{2:end});
            thresTemp = double(opts.DetectionThreshold);
        else
            thresTemp = [];
        end
    else
        % nargin equals 4 or 6
        nVar = 1e-10;
        if nargin == 6
            opts = nr5g.internal.parseOptions(fcnName,...
                {'DetectionThreshold'},varargin{:});
            thresTemp = double(opts.DetectionThreshold);
        else
            thresTemp = [];
        end
    end

    % Get the value of detection threshold
    if isempty(thresTemp) && ~isempty(pucch.SymbolAllocation)
        % When the provided threshold is empty, use the pre-defined
        % values as threshold for each PUCCH format
        switch formatPUCCH
            case 0
                thres = 0.49 - 0.07*(pucch.SymbolAllocation(2)==2);
            case 1
                thres = 0.22;
            otherwise % 2, 3, or 4
                thres = 0.45;
        end
    else
        thres = thresTemp;
    end

end
