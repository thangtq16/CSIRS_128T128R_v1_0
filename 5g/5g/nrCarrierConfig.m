classdef nrCarrierConfig < nr5g.internal.nrNumerologyConfig
    %nrCarrierConfig NR carrier configuration
    %   CARRIER = nrCarrierConfig creates a carrier configuration object
    %   for a specific OFDM numerology, as described in TS 38.211 Sections
    %   4.2, 4.3, and 4.4. This object contains parameters defining the
    %   carrier subcarrier spacing (SCS), bandwidth, and offset from point A.
    %   Point A is the center of subcarrier 0 of common resource block 0
    %   (CRB 0). For 60 kHz SCS, you can specify either normal or extended
    %   cyclic prefix. Given the carrier numerology, the object includes
    %   read-only properties describing the carrier resource grid
    %   time-domain dimensions. By default, the object defines a 10 MHz
    %   carrier with a 15 kHz subcarrier spacing (52 resource blocks). You
    %   can use the object in slot-oriented processing by specifying the
    %   current slot and frame numbers.
    %
    %   CARRIER = nrCarrierConfig(Name,Value) creates a carrier
    %   configuration object with the specified property Name set to the
    %   specified Value. You can specify additional name-value pair
    %   arguments in any order as (Name1,Value1,...,NameN,ValueN).
    %
    %   nrCarrierConfig properties:
    %
    %   NCellID             - Physical layer cell identity (0...1007)
    %                         (default 1)
    %   SubcarrierSpacing   - Subcarrier spacing in kHz
    %                         (15 (default), 30, 60, 120, 240, 480, 960)
    %   CyclicPrefix        - Cyclic prefix ('normal' (default), 'extended')
    %   NSizeGrid           - Number of resource blocks in carrier resource
    %                         grid (1...275) (default 52)
    %   NStartGrid          - Start of carrier resource grid relative to CRB 0
    %                         (0...2199) (default 0)
    %   NSlot               - Slot number (default 0)
    %   NFrame              - System frame number (default 0)
    %   IntraCellGuardBands - Intracell guard bands (default [])
    %
    %   nrCarrierConfig properties (read-only):
    %
    %   SymbolsPerSlot   - Number of OFDM symbols in a slot
    %   SlotsPerSubframe - Number of slots in a 1 ms subframe
    %   SlotsPerFrame    - Number of slots in a 10 ms frame
    %
    %   Note that NSlot and NFrame can be set to values beyond the number
    %   of slots per frame and beyond 1024, respectively. This allows them
    %   to be set directly by transmission loop counters in a MATLAB
    %   simulation. Calling code should ensure that these property values
    %   are modulo the respective ranges if required.
    %
    %   Example 1:
    %   % Create a default object specifying a 10 MHz carrier at 15 kHz
    %   % subcarrier spacing.
    %
    %   carrier = nrCarrierConfig
    %
    %   Example 2:
    %   % Create an object which specifies a 100 MHz carrier at 30 kHz
    %   % subcarrier spacing.
    %
    %   carrier = nrCarrierConfig;
    %   carrier.SubcarrierSpacing = 30;
    %   carrier.NSizeGrid = 273
    %
    %   Example 3:
    %   % Define a configuration object for a carrier with OFDM numerology
    %   % of 60 kHz subcarrier spacing and extended cyclic prefix. Set the
    %   % subcarrier spacing first then set the cyclic prefix type.
    %
    %   carrier = nrCarrierConfig;
    %   carrier.SubcarrierSpacing = 60;
    %   carrier.CyclicPrefix = 'extended'
    %
    %   See also nrResourceGrid, nrOFDMModulate, nrOFDMInfo,
    %   nrOFDMDemodulate.

    %   Copyright 2019-2023 The MathWorks, Inc.

    %#codegen

    properties
        %NCellID Physical layer cell identity
        %   Specify the physical layer cell identity. The value must be in
        %   the range 0...1007. The default value is 1.
        NCellID (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(NCellID, 1007)} = 1;
        
        %NSizeGrid Number of resource blocks in carrier resource grid
        %   Specify the size of carrier resource grid in number of resource
        %   blocks. The value must be an integer in the range 1...275. The
        %   default value is 52 which corresponds to the maximum number of
        %   resource blocks for a 10 MHz carrier with a 15 kHz subcarrier
        %   spacing.
        NSizeGrid (1,1) {mustBeNumeric, mustBeInteger, mustBePositive} = 52;

        %NStartGrid Start of carrier resource grid relative to CRB 0
        %   Specify the starting resource block of the carrier resource
        %   grid relative to common resource block 0 (CRB 0). The value
        %   must be an integer in the range 0...2199. The default value is 0.
        NStartGrid (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative} = 0;
        
        %NSlot Slot number
        %   Specify the slot number as a scalar nonnegative integer. The
        %   default value is 0.
        NSlot (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeFinite} = 0;

        %NFrame System frame number
        %   Specify the system frame number as a scalar nonnegative
        %   integer. The default value is 0.
        NFrame (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeFinite} = 0;

        %IntraCellGuardBands Intracell guard bands
        %   Specify the intracell guard bands for operation with shared
        %   spectrum channel access for FR1 as an NGB-by-2 numeric matrix
        %   or cell array of <a href="matlab:help('nrIntraCellGuardBandsConfig')">nrIntraCellGuardBandsConfig</a> objects. When 
        %   specified as a matrix, each row defines a guard band. The first
        %   column specifies the start of the guard band relative to common
        %   resource block 0 (CRB 0) and the second column defines the size
        %   of the guard band in resource blocks. When specified as cell
        %   array of nrIntraCellGuardBandsConfig objects, only the guard
        %   bands configuration with the same subcarrier spacing as the
        %   SubcarrierSpacing property of the carrier is applicable. This
        %   property corresponds to the higher-layer parameter
        %   IntraCellGuardBandsPerSCS in TS 38.331. The default value is
        %   empty, which indicates that guard bands are not configured and
        %   all resource blocks are available.
        IntraCellGuardBands {mustBeA(IntraCellGuardBands,{'numeric','cell'})} = zeros(0,2);

    end

    properties (SetAccess = private)
        %SymbolsPerSlot Number of OFDM symbols in a slot
        %   The value is either 14 or 12 depending on the value of
        %   CyclicPrefix, 'normal' or 'extended', respectively.
        SymbolsPerSlot;

        %SlotsPerSubframe Number of slots in a 1 ms subframe
        %   The value is one of {1, 2, 4, 8, 16, 32, 64} depending on
        %   SubcarrierSpacing values {15, 30, 60, 120, 240, 480, 960}, 
        %   respectively.
        SlotsPerSubframe;

        %SlotsPerFrame Number of slots in a 10 ms frame
        %   The value is one of {10, 20, 40, 80, 160, 320, 640} depending 
        %   on SubcarrierSpacing values {15, 30, 60, 120, 240, 480, 960}, 
        %   respectively.
        SlotsPerFrame;
    end
    
    methods

        function obj = nrCarrierConfig(varargin)
            %nrCarrierConfig Create nrCarrierConfig object
            obj@nr5g.internal.nrNumerologyConfig(varargin{:});        
        end

        function obj = set.IntraCellGuardBands(obj,val)
            obj.IntraCellGuardBands = validateGuardBands(obj,val);
        end

        function obj = set.NSizeGrid(obj,val)
            obj.NSizeGrid = validateNSizeGrid(obj,val);            
        end

        function obj = set.NStartGrid(obj,val)
            obj.NStartGrid = validateNStartGrid(obj,val);
        end
        
        % Read-only properties
        function val = get.SymbolsPerSlot(obj)
            % The number of OFDM symbols in a slot depends on the cyclic prefix
            if strcmpi(obj.CyclicPrefix,'normal')
                val = 14;
            else
                val = 12;
            end
        end

        function val = get.SlotsPerSubframe(obj)
            % The number of slots in a subframe depends on the subcarrier spacing
            val = double(obj.SubcarrierSpacing)/15;
        end

        function val = get.SlotsPerFrame(obj)
            % The number of slots in a frame depends on the subcarrier spacing
            val = 10*(double(obj.SubcarrierSpacing)/15);
        end

    end
    
    methods (Access = protected)

      function groups = getPropertyGroups(obj)
        groups = getPropertyGroups@nr5g.internal.nrNumerologyConfig(obj);
        customOrder = {'NCellID', 'SubcarrierSpacing', 'CyclicPrefix', 'NSizeGrid', 'NStartGrid', 'NSlot', 'NFrame','IntraCellGuardBands'};
        groups(1).PropertyList = orderfields(groups(1).PropertyList, customOrder);
      end

      function temp = validateGuardBands(obj,gb)
          prop = 'IntraCellGuardBands';
          if isnumeric(gb) % Numeric matrix
              % To allow codegen for varying length in a single function script
              temp = gb;
              coder.varsize('temp',[Inf 2],[1 1]);
              if ~isempty(temp)
                  validateattributes(temp,{'numeric'},{'ncols',2,'nonnegative','integer'},[class(obj) '.' prop],prop);

                  % Guard bands must not overlap
                  [ogb,str] = nr5g.internal.interlacing.overlappingGuardBands(gb);
                  coder.internal.errorIf(~isempty(ogb),'nr5g:nrIntraCellGuardBandsConfig:OverlappingGuardBands',str);
              end
              
          else % cell array of nrIntraCellGuardBandsConfig
              validateCellObjProp(obj, prop, 'nrIntraCellGuardBandsConfig', gb)
              scs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(gb,'SubcarrierSpacing');
              coder.internal.errorIf(numel(unique(scs(:))) ~= numel(gb),'nr5g:nrIntraCellGuardBandsConfig:MultipleSCS');
              temp = gb;
          end
      end

      function validateCellObjProp(obj, prop, classes, val)
          % Classes is a cell array of character vectors
          for idx = 1:numel(val)
              validateattributes(val{idx},classes,{'scalar'},[class(obj) '.' prop],prop);
          end
      end

        function val = validateNSizeGrid(~,val)
            mustBeLessThanOrEqual(val,275);
        end

        function val = validateNStartGrid(~,val)
            mustBeLessThanOrEqual(val,2199);
        end

    end
end
