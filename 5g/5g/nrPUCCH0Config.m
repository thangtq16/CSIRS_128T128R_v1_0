classdef nrPUCCH0Config < nr5g.internal.BWPSizeStart & nr5g.internal.pucch.Format0ConfigBase
    %nrPUCCH0Config NR PUCCH format 0 configuration
    %   PUCCH = nrPUCCH0Config creates a physical uplink control channel
    %   (PUCCH) configuration object, which contains all the parameters of
    %   PUCCH format 0. This object provides the properties related to TS
    %   38.211 Sections 6.3.2.1 to 6.3.2.3. The object also configures the
    %   bandwidth part (BWP) containing the PUCCH and the number of
    %   resource blocks (RBs) occupied by the PUCCH within the BWP. The
    %   default nrPUCCH0Config object configures a PUCCH format 0 allocated
    %   in the first RB of the BWP and the last OFDM symbol in the slot of
    %   14 OFDM symbols.
    %
    %   PUCCH = nrPUCCH0Config(Name,Value) creates a physical uplink
    %   control channel configuration object, PUCCH, with the specified
    %   property Name set to the specified Value. You can specify
    %   additional name-value pair arguments in any order as
    %   (Name1,Value1,...,NameN,ValueN).
    %
    %   nrPUCCH0Config properties:
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
    %   GroupHopping       - Group hopping configuration
    %                        ('neither' (default), 'enable', 'disable')
    %   HoppingID          - Hopping identity (0...1023) (default [])
    %   InitialCyclicShift - Initial cyclic shift (0...11) (default 0)
    %
    %   Example:
    %   % Display the default properties of PUCCH format 0.
    %
    %   pucch = nrPUCCH0Config
    %
    %   See also nrPUCCH, nrPUCCHIndices, nrPUCCH1Config, nrPUCCH2Config,
    %   nrPUCCH3Config, nrPUCCH4Config.

    %   Copyright 2020-2023 The MathWorks, Inc.

    %#codegen

    % Hidden properties
    properties (Hidden)
        % Custom property list to change the order of display properties
        CustomPropList = {'NSizeBWP','NStartBWP','SymbolAllocation','PRBSet',...
            'FrequencyHopping','SecondHopStartPRB','Interlacing','RBSetIndex',...
            'InterlaceIndex','GroupHopping','HoppingID','InitialCyclicShift'};
    end

    methods
        function obj = nrPUCCH0Config(varargin)
            obj = obj@nr5g.internal.pucch.Format0ConfigBase(varargin{:});
            if ~isempty(coder.target) % Allow to be var-size in codegen
                % Get the value of additional var-sized properties from the
                % name-value pairs
                obj.NStartBWP = nr5g.internal.parseProp('NStartBWP',[],varargin{:});
                obj.NSizeBWP = nr5g.internal.parseProp('NSizeBWP',[],varargin{:});
            end
        end
        
        % Validate configuration
        function validateConfig(obj)
            % Call Format0ConfigBase validator
            validateConfig@nr5g.internal.pucch.Format0ConfigBase(obj);
        end
    end

    methods (Access = protected)
        % Controls the conditional display of properties
        function flag = isInactiveProperty(obj, prop)
            % Call base class method
            flag = isInactiveProperty@nr5g.internal.pucch.Format0ConfigBase(obj, prop);
        end
    end

end
