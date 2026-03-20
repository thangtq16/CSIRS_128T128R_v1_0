classdef nrIntraCellGuardBandsConfig < comm.internal.ConfigBase
    %nrIntraCellGuardBandsConfig Intracell guard bands configuration
    %   GB = nrIntraCellGuardBands creates a configuration object for a set
    %   of intracell guard bands, as defined by the higher-layer parameter
    %   IntraCellGuardBandsPerSCS in TS 38.331. Intracell guard bands are
    %   signaled for operation with shared spectrum channel access for FR1.
    %   Intracell guard bands separate RB sets, as described in TS 38.214
    %   Section 7.
    %
    %   GB = nrIntraCellGuardBandsConfig(Name=Value) creates a 5G intracell
    %   guard bands configuration object with the specified property Name
    %   set to the specified Value. You can specify additional name-value
    %   arguments in any order as (Name1=Value1,...,NameN=ValueN).
    %
    %   nrIntraCellGuardBandsConfig properties:
    %
    %   GuardBandSize       - Start and size of the guard bands in RB
    %   SubcarrierSpacing   - Subcarrier spacing in kHz (15,30)
    %
    %   Example 1:
    %
    %   % Define the nominal intracell guard bands for a 100 MHz carrier 
    %   % and 30 kHz subcarrier spacing, as defined in TS 38.101-1 Table 5.3.3.2.
    %   gb = nrIntraCellGuardBandsConfig;
    %   gb.GuardBandSize = [50 6; 106 6; 161 6; 217 6];
    %   gb.SubcarrierSpacing = 30;
    %
    %   See also nrCarrierConfig, nrULCarrierConfig, nrPUSCHConfig,
    %   nrPUCCH0Config, nrPUCCH1Config, nrPUCCH2Config, nrPUCCH3Config.

    % Copyright 2023-2024 The MathWorks, Inc.

    %#codegen

    properties

        %GuardBandSize Start and length of intracell guard band set
        %   Specify the size of the intracell guard bands as an NGB-by-2
        %   matrix [startCRB1 nrofCRB1; startCRB2 nrofCRB2; ...]. Each row
        %   configures the start and length of each guard band in common
        %   resource blocks (CRB) from the lowest CRB of the carrier. The
        %   default value is empty, which specifies no intracell guard
        %   bands and a single RB set spanning the entire bandwidth part.
        GuardBandSize (:,2) {mustBeNumeric, mustBeInteger, mustBeFinite, mustBeNonnegative} = zeros(0,2);

        %SubcarrierSpacing Subcarrier spacing in kHz
        %   Specify the subcarrier spacing of the carrier in kHz. The value
        %   must be one of {15, 30}. The default value is 15.
       SubcarrierSpacing (1,1) {mustBeMember(SubcarrierSpacing, [15 30])} = 15;

    end

    methods
        % Constructor
        function obj = nrIntraCellGuardBandsConfig(varargin)
            obj@comm.internal.ConfigBase(varargin{:});
        end

        function obj = set.GuardBandSize(obj,val)
            prop = 'GuardBandSize';

            % To allow codegen for varying length in a single function script
            gb = double(val);
            coder.varsize('gb',[Inf 2],[1 1]);
            if size(gb,1)>1
                [ogb,str] = nr5g.internal.interlacing.overlappingGuardBands(gb);
                coder.internal.errorIf(~isempty(ogb),'nr5g:nrIntraCellGuardBandsConfig:OverlappingGuardBands',str);
            end
            obj.(prop) = gb;
        end
    end

end
