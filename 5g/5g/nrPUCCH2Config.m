classdef nrPUCCH2Config < nr5g.internal.BWPSizeStart & nr5g.internal.pucch.Format2ConfigBase
    %nrPUCCH2Config NR PUCCH format 2 configuration
    %   PUCCH = nrPUCCH2Config creates a physical uplink control channel
    %   (PUCCH) configuration object, which contains all the parameters of
    %   PUCCH format 2. This object provides the properties related to TS
    %   38.211 Sections 6.3.2.1, 6.3.2.5, and 6.4.1.3.2. The object also
    %   configures the bandwidth part (BWP) containing the PUCCH and the
    %   number of resource blocks (RBs) occupied by the PUCCH within the
    %   BWP. The default nrPUCCH2Config object configures a PUCCH format 2
    %   allocated in the first RB of the BWP and the last OFDM symbol in
    %   the slot of 14 OFDM symbols.
    %
    %   PUCCH = nrPUCCH2Config(Name,Value) creates a physical uplink
    %   control channel configuration object, PUCCH, with the specified
    %   property Name set to the specified Value. You can specify
    %   additional name-value pair arguments in any order as
    %   (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPUCCH2Config properties:
    %
    %   NSizeBWP           - Size of BWP in terms of number of physical
    %                        resource blocks (PRBs) (1...275) (default [])
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
    %   NID                - Data scrambling identity (0...1023) (default [])
    %   RNTI               - Radio network temporary identifier (0...65535)
    %                        (default 1)
    %   NID0               - Scrambling identity for demodulation reference
    %                        signal (DM-RS) (0...65535) (default [])
    %
    %   Example:
    %   % Display the default properties of PUCCH format 2.
    %
    %   pucch = nrPUCCH2Config
    %
    %   See also nrPUCCH, nrPUCCHDMRS, nrPUCCHDMRSIndices,
    %   nrPUCCHIndices, nrPUCCH0Config, nrPUCCH1Config, nrPUCCH3Config,
    %   nrPUCCH4Config.

    %   Copyright 2020-2023 The MathWorks, Inc.

    %#codegen

    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'NSizeBWP','NStartBWP','SymbolAllocation','PRBSet',...
            'FrequencyHopping','SecondHopStartPRB','Interlacing','RBSetIndex','InterlaceIndex',...
            'SpreadingFactor','OCCI','NID','RNTI','NID0'};
    end

    methods
        function obj = nrPUCCH2Config(varargin)
            obj = obj@nr5g.internal.pucch.Format2ConfigBase(varargin{:});
            if ~isempty(coder.target) % Allow to be var-size in codegen
                % Get the value of additional var-sized properties from the
                % name-value pairs
                obj.NStartBWP = nr5g.internal.parseProp('NStartBWP',[],varargin{:});
                obj.NSizeBWP = nr5g.internal.parseProp('NSizeBWP',[],varargin{:});
            end
        end
        
        % Validate configuration
        function validateConfig(obj)
            % Call base class method
            validateConfig@nr5g.internal.pucch.Format2ConfigBase(obj);
        end
    end

end
