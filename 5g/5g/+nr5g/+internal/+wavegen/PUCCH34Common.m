classdef PUCCH34Common < nr5g.internal.wavegen.PUCCHConfigBase & ...
                         nr5g.internal.wavegen.CodingCommon & nr5g.internal.wavegen.DMRSPowerCommon
    %PUCCH34Common Common wavegen configuration object for PUCCH formats 3 and 4
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   PUCCH34Common properties (configurable):
    %
    %   Coding         - Flag to enable channel coding (default true)
    %   TargetCodeRate - Target code rate (default 0.15)
    %   NumUCIBits     - Number of UCI part 1 bits (0...1706) (default 1)
    %   DataSourceUCI  - Source of UCI part 1 contents (PN or custom) (default 'PN9-ITU')
    %   NumUCI2Bits    - Number of UCI part 2 bits (0...1706) (default 0)
    %   DataSourceUCI2 - Source of UCI part 2 contents (PN or custom) (default 'PN9-ITU')
    %   DMRSPower      - Power scaling of the DM-RS in dB (default 0)

    %   Copyright 2021-2023 The MathWorks, Inc.

    %#codegen

    % Constant, hidden properties
    properties (Constant,Hidden)
        DataSourceUCI2_Values = nr5g.internal.wavegen.DataSourceCommon.DataSource_Options;
    end
    
    % Public, tunable properties
    properties
        %TargetCodeRate Code rate used to calculate transport block size
        % Specify TargetCodeRate as a real scalar with values in (0, 1).
        % This property applies when Coding is true and there is
        % multiplexing of UCI part 1 and UCI part 2. The default is 0.15.
        TargetCodeRate (1,1) {mustBeNumeric, mustBeReal, mustBePositive, mustBeLessThan(TargetCodeRate, 1)} = 0.15;
        
        %NumUCI2Bits Number of UCI part 2 bits
        % Specify the number of UCI part 2 bits as a scalar nonnegative
        % integer up to 1706. For no UCI part 2 transmission, set the value
        % to 0. The default is 0.
        NumUCI2Bits (1,1) {mustBeNumeric, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(NumUCI2Bits, 1706)} = 0;
        
        %DataSourceUCI2 Source of UCI part 2 contents
        % Specify DataSourceUCI2 as one of {'PN9-ITU', 'PN9', 'PN11',
        % 'PN15', 'PN23'}, as a cell array containing one of the
        % abovementioned options and a numeric scalar that is the random
        % seed (for example, {'PN9',7}), or as a binary vector. If the seed
        % is not specified, then all shift registers are initialized with
        % an active state. This property applies when NumUCI2Bits > 0. The
        % default is 'PN9-ITU'.
        DataSourceUCI2 = 'PN9-ITU';
    end

    methods
        % Self-validate and set properties
        function obj = set.DataSourceUCI2(obj,val)
            prop = 'DataSourceUCI2';
            temp = getDataSource(obj, prop, val);
            obj.(prop) = temp;
        end
    end
    
    methods(Access = public)
        % Validate configuration
        function validateConfig(obj,format)
            UCIBits = obj.NumUCIBits + obj.NumUCI2Bits;
            coder.internal.errorIf(UCIBits > 0 && UCIBits <= 2,'nr5g:nrWaveformGenerator:InvalidUCIBits234',format,UCIBits);
            coder.internal.errorIf(UCIBits > 1706,'nr5g:nrWaveformGenerator:InvalidPUCCHPayloadSize',format,UCIBits);
        end
    end
    
    methods (Access=protected)
        % Controls the conditional display of properties
        function inactive = isInactiveProperty(obj, prop)
            inactive = isInactiveProperty@nr5g.internal.wavegen.PUCCHConfigBase(obj, prop);
            
            % DMRSUplinkTransformPrecodingR16 only if Modulation is
            % pi/2-BPSK
            if strcmp(prop,'DMRSUplinkTransformPrecodingR16')
                inactive = ~strcmp(obj.Modulation,'pi/2-BPSK');
            end

            % NID0 only if Modulation is pi/2-BPSK and
            % DMRSUplinkTransformPrecodingR16 is true
            if strcmp(prop,'NID0')
                inactive = ~(obj.DMRSUplinkTransformPrecodingR16 && strcmp(obj.Modulation,'pi/2-BPSK'));
            end

            % NumUCIBits only if Coding is true
            if strcmp(prop,'NumUCIBits')
                inactive = ~obj.Coding;
            end
            
            % DataSourceUCI only if Coding is false or NumUCIBits > 0
            if strcmp(prop,'DataSourceUCI')
                inactive = obj.Coding && ~obj.NumUCIBits;
            end
            
            % TargetCodeRate only if Coding is true, NumUCIBits > 0, and
            % NumUCI2Bits > 0
            if strcmp(prop,'TargetCodeRate')
                inactive = ~(obj.Coding && obj.NumUCIBits>0 && obj.NumUCI2Bits>0);
            end
            
            % NumUCI2Bits only if Coding is true and NumUCIBits > 0
            if strcmp(prop,'NumUCI2Bits')
                inactive = ~(obj.Coding && obj.NumUCIBits>0);
            end
            
            % DataSourceUCI2 only if Coding is true, NumUCIBits > 0, and
            % NumUCI2Bits > 0
            if strcmp(prop,'DataSourceUCI2')
                inactive = ~(obj.Coding && obj.NumUCIBits>0 && obj.NumUCI2Bits>0);
            end
        end
    end
end
