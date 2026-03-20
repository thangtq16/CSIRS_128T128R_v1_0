classdef nrPUCCH4Config < nr5g.internal.BWPSizeStart & nr5g.internal.pucch.Format4ConfigBase
    %nrPUCCH4Config NR PUCCH format 4 configuration
    %   PUCCH = nrPUCCH4Config creates a physical uplink control channel
    %   (PUCCH) configuration object, which contains all the parameters of
    %   PUCCH format 4. This object provides the properties related to TS
    %   38.211 Sections 6.3.2.1, 6.3.2.2, 6.3.2.6, and 6.4.1.3.3. The
    %   object also configures the bandwidth part (BWP) containing the
    %   PUCCH and the number of resource blocks (RBs) occupied by the PUCCH
    %   within the BWP. The default nrPUCCH4Config object configures a
    %   PUCCH format 4 allocated in the first RB of the BWP and spanning
    %   over 14 OFDM symbols in a slot.
    %
    %   PUCCH = nrPUCCH4Config(Name,Value) creates a physical uplink
    %   control channel configuration object, PUCCH, with the specified
    %   property Name set to the specified Value. You can specify
    %   additional name-value pair arguments in any order as
    %   (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPUCCH4Config properties:
    %
    %   NSizeBWP           - Size of BWP in terms of number of physical
    %                        resource blocks (PRBs) (1...275) (default [])
    %   NStartBWP          - Starting PRB index of BWP relative to common
    %                        resource block 0 (CRB 0) (0...2473) (default [])
    %   Modulation         - Modulation scheme ('QPSK' (default), 'pi/2-BPSK')
    %   SymbolAllocation   - OFDM symbol allocation of PUCCH within a slot
    %                        (default [0 14])
    %   PRBSet             - PRBs allocated for PUCCH within the BWP
    %                        (default 0)
    %   FrequencyHopping   - Frequency hopping configuration
    %                        ('neither' (default), 'intraSlot', 'interSlot')
    %   SecondHopStartPRB  - Starting PRB of second hop relative to the
    %                        BWP (0...274) (default 1)
    %   GroupHopping       - Group hopping configuration
    %                        ('neither' (default), 'enable', 'disable')
    %   HoppingID          - Hopping identity (0...1023) (default [])
    %   SpreadingFactor    - Spreading factor (4 or 2 (default))
    %   OCCI               - Orthogonal cover code index (0...6) (default 0)
    %   NID                - Data scrambling identity (0...1023) (default [])
    %   RNTI               - Radio network temporary identifier (0...65535)
    %                        (default 1)
    %   NID0               - Scrambling identity for demodulation reference
    %                        signal (DM-RS) (0...65535) (default [])
    %   DMRSUplinkTransformPrecodingR16 - Enable low PAPR DM-RS sequence for
    %                        DFT-s-OFDM (0 (default), 1). To enable this
    %                        property, set the Modulation property to 'pi/2-BPSK'.
    %   AdditionalDMRS     - Additional demodulation reference signal (DM-RS)
    %                        configuration flag (0 (default), 1)
    %
    %   Example:
    %   % Display the default properties of PUCCH format 4.
    %
    %   pucch = nrPUCCH4Config
    %
    %   See also nrPUCCH, nrPUCCHDMRS, nrPUCCHDMRSIndices,
    %   nrPUCCHIndices, nrPUCCH0Config, nrPUCCH1Config, nrPUCCH2Config,
    %   nrPUCCH3Config.

    %   Copyright 2020-2023 The MathWorks, Inc.

    %#codegen

    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'NSizeBWP','NStartBWP','Modulation','SymbolAllocation',...
            'PRBSet','FrequencyHopping','SecondHopStartPRB','GroupHopping',...
            'HoppingID','SpreadingFactor','OCCI','NID','RNTI','NID0',...
            'DMRSUplinkTransformPrecodingR16','AdditionalDMRS'};
    end

    methods
        function obj = nrPUCCH4Config(varargin)
            obj = obj@nr5g.internal.pucch.Format4ConfigBase(varargin{:});
            if ~isempty(coder.target) % Allow to be var-size in codegen
                % Get the value of additional var-sized properties from the
                % name-value pairs
                obj.NStartBWP = nr5g.internal.parseProp('NStartBWP',[],varargin{:});
                obj.NSizeBWP = nr5g.internal.parseProp('NSizeBWP',[],varargin{:});
            end
        end
        
        % Validate configuration
        function validateConfig(obj)
            % Call Format4ConfigBase validator
            validateConfig@nr5g.internal.pucch.Format4ConfigBase(obj);
        end
    end

end
