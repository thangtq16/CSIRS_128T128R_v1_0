function sym = nrPUCCHDMRS(carrier,pucch,varargin)
%nrPUCCHDMRS Physical uplink control channel demodulation reference signal
%   SYM = nrPUCCHDMRS(CARRIER,PUCCH) returns the complex symbols SYM
%   containing demodulation reference signal (DM-RS) symbols of physical
%   uplink control channel, as defined in TS 38.211 Section 6.4.1.3, for
%   all physical uplink control channel formats. CARRIER is a scalar
%   nrCarrierConfig object. For physical uplink control channel formats 0,
%   1, 2, 3, and 4, PUCCH is a scalar nrPUCCH0Config, nrPUCCH1Config,
%   nrPUCCH2Config, nrPUCCH3Config, and nrPUCCH4Config, respectively. The
%   output SYM is empty for physical uplink control channel format 0.
%
%   Note that for PUCCH formats 1, 3, and 4, when GroupHopping property of
%   PUCCH configuration is set to 'disable', sequence hopping is enabled
%   which might result in selecting a sequence number that is not
%   appropriate for short base sequences.
%
%   CARRIER is a carrier configuration object, as described in <a
%   href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a>.
%   Only these object properties are relevant for this function:
%
%   NCellID             - Physical layer cell identity (0...1007) (default 1)
%   SubcarrierSpacing   - Subcarrier spacing in kHz
%                         (15 (default), 30, 60, 120, 240, 480, 960)
%   CyclicPrefix        - Cyclic prefix ('normal' (default), 'extended')
%   NSizeGrid           - Number of resource blocks in carrier resource
%                         grid (1...275) (default 52)
%   NStartGrid          - Start of carrier resource grid relative to common
%                         resource block 0 (CRB 0) (0...2199) (default 0)
%   NSlot               - Slot number (default 0)
%   IntraCellGuardBands - Intracell guard bands (default [])
%
%   For format 1, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH1Config')">nrPUCCH1Config</a>. Only these
%   object properties are relevant for this function:
%
%   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
%                        (default [0 14])
%   PRBSet             - PRBs allocated for PUCCH within the BWP
%                        (default 0)
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
%   NStartBWP          - Starting PRB index of BWP relative to common
%                        resource block 0 (CRB 0) (0...2473) (default [])
%   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
%                        (default [13 1])
%   PRBSet             - PRBs allocated for PUCCH within the BWP
%                        (default 0)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   SecondHopStartPRB  - Starting PRB of second hop relative to the
%                        BWP (0...274) (default 1)
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%   SpreadingFactor    - Spreading factor (2 (default), 4)
%   OCCI               - Orthogonal cover code index (0...3) (default 0)
%   NID0               - DM-RS scrambling identity (0...65535) (default [])
%
%   For format 3, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH3Config')">nrPUCCH3Config</a>. Only these
%   object properties are relevant for this function:
%
%   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
%                        (default [0 14])
%   PRBSet             - PRBs allocated for PUCCH within the BWP
%                        (default 0)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   Interlacing        - Enable interlacing (default false)
%   RBSetIndex         - Resource block set index (default 0)
%   InterlaceIndex     - Interlace indices (0...9) (default 0)
%   GroupHopping       - Group hopping configuration
%                        ('neither' (default), 'enable', 'disable')
%   HoppingID          - Hopping identity (0...1023) (default [])
%   SpreadingFactor    - Spreading factor (1, 2 (default), 4)
%   OCCI               - Orthogonal cover code index (0...3) (default 0)
%   AdditionalDMRS     - Additional DM-RS configuration flag (0 (default), 1)
%   DMRSUplinkTransformPrecodingR16 - Enable low PAPR DM-RS sequence for
%                        DFT-s-OFDM (0 (default), 1). To enable this
%                        property, set the Modulation property to 'pi/2-BPSK'.
%   NID0               - Scrambling identity for demodulation reference
%                        signal (DM-RS) (0...65535) (default [])
%
%   For format 4, PUCCH is the physical uplink control channel
%   configuration object, as described in <a
%   href="matlab:help('nrPUCCH4Config')">nrPUCCH4Config</a>. Only these
%   object properties are relevant for this function:
%
%   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
%                        (default [0 14])
%   PRBSet             - PRBs allocated for PUCCH within the BWP
%                        (default 0)
%   FrequencyHopping   - Frequency hopping configuration
%                        ('neither' (default), 'intraSlot', 'interSlot')
%   GroupHopping       - Group hopping configuration
%                        ('neither' (default), 'enable', 'disable')
%   HoppingID          - Hopping identity (0...1023) (default [])
%   SpreadingFactor    - Spreading factor (2 (default), 4)
%   OCCI               - Orthogonal cover code index (0...6) (default 0)
%   AdditionalDMRS     - Additional DM-RS configuration flag (0 (default), 1)
%   DMRSUplinkTransformPrecodingR16 - Enable low PAPR DM-RS sequence for
%                        DFT-s-OFDM (0 (default), 1). To enable this
%                        property, set the Modulation property to 'pi/2-BPSK'.
%   NID0               - Scrambling identity for demodulation reference
%                        signal (DM-RS) (0...65535) (default [])
%
%   SYM = nrPUCCHDMRS(CARRIER,PUCCH,NAME,VALUE) specifies additional
%   options as NAME,VALUE pairs to allow control over the data type of the
%   output symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   For PUCCH formats 0 to 3 and operation with shared spectrum channel
%   access for FR1, set Interlacing = true and specify the allocated
%   frequency resources using the properties RBSetIndex and InterlaceIndex
%   of the PUCCH configuration. The PRBSet and FrequencyHopping properties
%   are ignored. For PUCCH formats 2 and 3, you can specify the
%   SpreadingFactor and OCCI for single-interlace configurations.
%
%   Example 1:
%   % Generate the DM-RS symbols of a physical uplink control channel with
%   % format 1 occupying first resource block in the bandwidth part. The
%   % starting OFDM symbol and number of OFDM symbols allocated for PUCCH
%   % is 3 and 9, respectively. The bandwidth part occupies the complete
%   % 10 MHz bandwidth of a 15 kHz subcarrier spacing carrier.
%
%   % Set carrier parameters
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15;
%   carrier.CyclicPrefix = 'normal';
%   carrier.NSizeGrid = 52;
%
%   % Set PUCCH format 1 parameters
%   pucch1 = nrPUCCH1Config;
%   pucch1.NSizeBWP = [];
%   pucch1.NStartBWP = [];
%   pucch1.PRBSet = 0;
%   pucch1.SymbolAllocation = [3 9];
%
%   % Get PUCCH format 1 DM-RS symbols
%   sym = nrPUCCHDMRS(carrier,pucch1);
%
%   Example 2:
%   % Generate the DM-RS symbols of a physical uplink control channel with
%   % format 3 occupying first 12 resource blocks in the bandwidth part.
%   % The starting OFDM symbol and number of OFDM symbols allocated for
%   % PUCCH is 3 and 9, respectively. Set additional DM-RS to 1. The
%   % bandwidth part occupies the complete 10 MHz bandwidth of a 15 kHz
%   % subcarrier spacing carrier.
%
%   % Set carrier parameters
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15;
%   carrier.CyclicPrefix = 'normal';
%   carrier.NSizeGrid = 52;
%
%   % Set PUCCH format 3 parameters
%   pucch3 = nrPUCCH3Config;
%   pucch3.NSizeBWP = [];
%   pucch3.NStartBWP = [];
%   pucch3.PRBSet = 0:11;
%   pucch3.SymbolAllocation = [3 9];
%   pucch3.AdditionalDMRS = 1;
%
%   % Get PUCCH format 3 DM-RS symbols
%   sym = nrPUCCHDMRS(carrier,pucch3);
%
%   See also nrPUCCHDMRSIndices, nrPUCCH, nrPUCCH0, nrPUCCH1, nrPUCCH2,
%   nrPUCCH3, nrPUCCH4, nrPUCCH0Config, nrPUCCH1Config, nrPUCCH2Config,
%   nrPUCCH3Config, nrPUCCH4Config, nrCarrierConfig,
%   nrIntraCellGuardBandsConfig.

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    narginchk(2,4);

    % Validate inputs
    formatPUCCH = nr5g.internal.pucch.validateInputObjects(carrier,pucch);

    % Determine the number of RB allocated
    Mrb = numel(nr5g.internal.allocatedPhysicalResourceBlocks(carrier,pucch));

    % Get DM-RS symbols
    if Mrb == 0 || isempty(pucch.SymbolAllocation) || ...
            (pucch.SymbolAllocation(2) == 0) || (formatPUCCH == 0)
        dmrs = complex(zeros(0,1));
    else
        % Validate allocation
        nr5g.internal.pucch.validateAllocation(carrier,pucch);

        % Get the DM-RS symbols, depending on PUCCH format
        switch formatPUCCH
            case 1
                % PUCCH format 1, TS 38.211 Section 6.4.1.3.1.1
                dmrs = nr5g.internal.pucch.dmrsFormat1(carrier,pucch);
            case 2
                % PUCCH format 2, TS 38.211 Section 6.4.1.3.2.1
                dmrs = nr5g.internal.pucch.dmrsFormat2(carrier,pucch);
            otherwise
                % PUCCH formats 3 and 4, TS 38.211 Section 6.4.1.3.3.1
                dmrs = nr5g.internal.pucch.dmrsFormats34(carrier,pucch);
        end
    end

    % Apply options
    if nargin > 2
        fcnName = 'nrPUCCHDMRS';
        opts = nr5g.internal.parseOptions(fcnName,...
            {'OutputDataType'},varargin{:});
        sym = cast(dmrs(:),opts.OutputDataType);
    else
        sym = dmrs(:);
    end

end
