function sym = nrPUCCH(carrier,pucch,uciBits,varargin)
%nrPUCCH Physical uplink control channel
%   SYM = nrPUCCH(CARRIER,PUCCH,UCIBITS) returns the physical uplink
%   control channel symbols, SYM, as defined in TS 38.211 Sections 6.3.2.3
%   to 6.3.2.6, for given carrier configuration CARRIER, physical uplink
%   control channel configuration PUCCH, and uplink control information
%   (UCI) bits UCIBITS. CARRIER is a scalar nrCarrierConfig object. For
%   physical uplink control channel formats 0, 1, 2, 3, and 4, PUCCH is a
%   scalar nrPUCCH0Config, nrPUCCH1Config, nrPUCCH2Config, nrPUCCH3Config,
%   and nrPUCCH4Config, respectively. UCIBITS is a column vector with
%   binary values or a cell array with at most two cells. When UCIBITS is a
%   cell array, each cell must be a column vector. For format 0, when
%   UCIBITS is a column vector or a cell array with one cell, UCIBITS is
%   assumed to be hybrid automatic repeat request acknowledgment (HARQ-ACK)
%   bits. For format 0, when UCIBITS is cell array with two cells, the
%   first cell is assumed as HARQ-ACK bits and the second cell is assumed
%   as scheduling request (SR) bit. For all formats other than format 0,
%   UCIBITS is either a column vector or a cell array with one cell. For
%   format 1, UCIBITS is either HARQ-ACK or SR payload bits. In case of
%   format 1 with only positive SR, use value 0 for UCIBITS. For formats 2,
%   3, and 4, UCIBITS is the codeword containing encoded UCI bits.
%
%   Note that for PUCCH formats 0 and 1, when GroupHopping property of
%   PUCCH configuration is set to 'disable', sequence hopping is enabled
%   which might result in selecting a sequence number that is not
%   appropriate for short base sequences.
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
%   SYM = nrPUCCH(CARRIER,PUCCH,UCIBITS,NAME,VALUE) specifies an additional
%   option as a NAME,VALUE pair to allow control over the format of the
%   symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   For PUCCH formats 0 to 3 and operation with shared spectrum channel
%   access for FR1, set Interlacing = true and specify the allocated
%   frequency resources using the RBSetIndex and InterlaceIndex properties
%   of the PUCCH configuration. The PRBSet and FrequencyHopping properties
%   are ignored. For PUCCH formats 2 and 3, you can specify the
%   SpreadingFactor and OCCI for single-interlace configurations.
%
%   % Example 1:
%   % Generate the PUCCH format 0 symbols for transmitting positive SR,
%   % in a PUCCH occupying last two OFDM symbols of a slot. The initial
%   % cyclic shift is 5, and both intra-slot frequency hopping and group
%   % hopping is enabled. Consider a carrier with 15 kHz SCS having cell
%   % identity as 512 and slot number as 3.
%
%   % Set carrier parameters
%   carrier = nrCarrierConfig;
%   carrier.NCellID = 512;
%   carrier.SubcarrierSpacing = 15;
%   carrier.CyclicPrefix = 'normal';
%   carrier.NSlot = 3;
%
%   % Set PUCCH format 0 parameters
%   pucch0 = nrPUCCH0Config;
%   pucch0.SymbolAllocation = [12 2];
%   pucch0.FrequencyHopping = 'intraSlot';
%   pucch0.GroupHopping = 'enable';
%   pucch0.HoppingID = [];
%   pucch0.InitialCyclicShift = 5;
%
%   % Set HARQ-ACK and SR bits
%   sr = 1;
%   ack = zeros(0,1);
%   uciBits = {ack, sr};
%
%   % Get PUCCH format 0 symbols
%   sym = nrPUCCH(carrier,pucch0,uciBits);
%
%   Example 2:
%   % Generate the PUCCH format 1 modulated symbols for 1-bit UCI, when the
%   % starting symbol in a slot is 3 and the number of PUCCH symbols is 9.
%   % The orthogonal cover code index is 1, hopping identity is 512, and
%   % initial cyclic shift is 9. Consider both intra-slot frequency hopping
%   % and group hopping is enabled. Use a 60 kHz SCS carrier with extended
%   % cyclic prefix having slot number as 7.
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
%   Example 3:
%   % Generate PUCCH format 2 symbols with cell identity as 148 and radio
%   % network temporary identifier as 160.
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
%   % Get PUCCH format 2 symbols for a random codeword
%   uciCW = randi([0 1],100,1);
%   sym = nrPUCCH(carrier,pucch2,uciCW);
%
%   See also nrPUCCH0, nrPUCCH1, nrPUCCH2, nrPUCCH3, nrPUCCH4, nrPUCCHDMRS,
%   nrPUCCHIndices, nrPUCCHDecode, nrPUCCH0Config, nrPUCCH1Config,
%   nrPUCCH2Config, nrPUCCH3Config, nrPUCCH4Config, nrCarrierConfig,
%   nrIntraCellGuardBandsConfig.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    narginchk(3,5);

    % Validate inputs
    fcnName = 'nrPUCCH';
    [formatPUCCH,interlacing,freqHopping] = nr5g.internal.pucch.validateInputObjects(carrier,pucch);
    uciBitsCell = validateUCIBits(uciBits,formatPUCCH,fcnName);

    % Get the intra-slot frequency hopping configuration
    if strcmpi(freqHopping,'intraSlot')
        intraSlotfreqHopping = 'enabled';
    else
        intraSlotfreqHopping = 'disabled';
    end

    % Get the scrambling identity
    nid = scramblingIdentity(carrier,pucch,formatPUCCH);

    % Relative slot number
    nslot = mod(double(carrier.NSlot),carrier.SlotsPerFrame);

    % If interlacing is active, determine the resource block indices in the
    % interlace. Otherwise, use PRBSet to determine the RB allocation.
    if interlacing
        [nIRB,~,Mrb] = nr5g.internal.interlacing.interlacedResourceBlockSet(carrier,pucch);
    else
        % Number of resource blocks allocated for PUCCH without interlacing
        nIRB = [];
        Mrb = numel(unique(double(pucch.PRBSet(:))));
    end

    % Get the symbols, depending on PUCCH format
    if isempty(pucch.SymbolAllocation) || (pucch.SymbolAllocation(2) == 0) ...
            || Mrb == 0 || isempty(uciBits)
        seq = zeros(0,1);
    else

        % Spreading factor (formats 2 to 4) and OCC index (formats 1 to 4)
        [sf,occi] = nr5g.internal.pucch.occConfiguration(pucch,formatPUCCH);

        switch formatPUCCH
            case 0
                [ack,sr] = parseACKSRBits(uciBitsCell,formatPUCCH);
                % PUCCH format 0 symbols
                seq = nr5g.internal.pucch.hPUCCH0(logical(ack(:)),logical(sr(:)),pucch.SymbolAllocation,...
                        carrier.CyclicPrefix,nslot,nid,pucch.GroupHopping,...
                        pucch.InitialCyclicShift,intraSlotfreqHopping,Mrb,nIRB);
            case 1
                [ack,sr] = parseACKSRBits(uciBitsCell,formatPUCCH);
                % PUCCH format 1 symbols
                seq = nr5g.internal.pucch.hPUCCH1(logical(ack(:)),sr,pucch.SymbolAllocation,...
                        carrier.CyclicPrefix,nslot,nid,pucch.GroupHopping,...
                        pucch.InitialCyclicShift,intraSlotfreqHopping,occi,Mrb,nIRB);
            case 2
                % PUCCH format 2 symbols
                seq = nr5g.internal.pucch.hPUCCH2(uciBitsCell{1},nid,pucch.RNTI,nIRB,sf,occi);
            case 3
                % PUCCH format 3 symbols
                seq = nr5g.internal.pucch.hPUCCH3(uciBitsCell{1},pucch.Modulation,nid,pucch.RNTI,Mrb,sf,occi);
            otherwise
                % PUCCH format 4 symbols
                seq = nrPUCCH4(uciBitsCell{1},pucch.Modulation,nid,pucch.RNTI,sf,occi,Mrb);
        end
    end

    % Apply options
    if nargin > 3
        opts = nr5g.internal.parseOptions(fcnName,...
            {'OutputDataType'},varargin{:});
        sym = cast(seq,opts.OutputDataType);
    else
        sym = seq;
    end

end

function nid = scramblingIdentity(carrier,pucch,formatPUCCH)
    if any(formatPUCCH == [0 1])
        if isempty(pucch.HoppingID)
            nid = double(carrier.NCellID);
        else
            nid = double(pucch.HoppingID(1));
        end
    else
        % PUCCH formats 2, 3, and 4
        if isempty(pucch.NID)
            nid = double(carrier.NCellID);
        else
            nid = double(pucch.NID(1));
        end
    end
end

function uciBitsCell = validateUCIBits(uciBits,formatPUCCH,fcnName)
%ValidateUCIBits Validate input UCIBITS and return them in a cell array

    % Validate input UCIBITS
    validateattributes(uciBits,{'cell','numeric','logical'},{'2d'},...
        fcnName,'UCIBITS');
    maxNumElements = 1;
    if formatPUCCH == 0
        maxNumElements = 2;
    end
    if iscell(uciBits)
        numElements = numel(uciBits);
        coder.internal.errorIf(numElements > maxNumElements,...
            'nr5g:nrPUCCH:InvalidUCILength',numElements,maxNumElements);
        if numElements > 0
            validateInputWithEmpty(uciBits{1},{'double','int8','logical'},...
                {'real','column','nonnan'},fcnName,'UCIBITS{1}');
            if any(formatPUCCH == [0 1])
                lenUCI1 = length(uciBits{1});
                coder.internal.errorIf(lenUCI1 > 2,...
                    'nr5g:nrPUCCH:InvalidUCILengthCell',lenUCI1,2);
            end
        end
        if numElements > 1
            validateInputWithEmpty(uciBits{2},{'double','int8','logical'},...
                {'real','scalar','nonnan'},fcnName,'UCIBITS{2}');
        end
    else
        validateInputWithEmpty(uciBits,{'double','int8','logical'},...
            {'real','column','nonnan'},fcnName,'UCIBITS');
        if any(formatPUCCH == [0 1])
            lenUCI = length(uciBits);
            coder.internal.errorIf(lenUCI > 2,...
                'nr5g:nrPUCCH:InvalidUCILength',lenUCI,2);
        end
    end

    % Convert numeric type input to cell input
    if iscell(uciBits)
        uciBitsCell = uciBits;
    else
        uciBitsCell = {uciBits};
    end

end

function [ack,sr] = parseACKSRBits(uciBitsCell,formatPUCCH)

    if formatPUCCH == 0
        % Get the ACK and SR bits, depending on uciBitsCell
        if numel(uciBitsCell) == 1
            % Only one cell, treat it as ACK bits
            ack = uciBitsCell{1};
            sr = zeros(0,1);
        else
            % First cell is ACK bits, second cell is SR bit
            ack = uciBitsCell{1};
            sr = double(uciBitsCell{2});
        end
    else % format 1
        % UCIBITS is either ACK or SR. Pass UCIBITS in ACK and
        % empty in SR to nrPUCCH1 function, as the function treats
        % SR as a flag and is of length 1. ACK bits in nrPUCCH1
        % function does direct processing on the bits.
        ack = uciBitsCell{1};
        sr = zeros(0,1);
    end
    
end

function validateInputWithEmpty(in,classes,attributes,fcnName,varname)
%Validates input with empty handling

    if ~isempty(in)
        % Check for type and attributes
        validateattributes(in,classes,attributes,fcnName,varname);
    else
        % Check for type when input is empty
        validateattributes(in,classes,{'2d'},fcnName,varname);
    end

end