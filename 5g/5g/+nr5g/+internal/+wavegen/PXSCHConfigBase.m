classdef PXSCHConfigBase < nr5g.internal.wavegen.PXYCHConfigBase & ...
                           nr5g.internal.wavegen.DataSourceCommon & ...
                           nr5g.internal.wavegen.DMRSPowerCommon
    %PXSCHConfigBase Class offering properties common between
    %nrWavegenPDSCHConfig and nrWavegenPUSCHConfig
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   PXSCHConfigBase properties (configurable):
    %
    %   Coding         - Flag to enable transport channel coding (default true)
    %   TargetCodeRate - Target code rate (0...1) (default 526/1024)
    %   XOverhead      - Rate matching overhead (default 0)
    %   RVSequence     - Redundancy version sequence (default [0 2 3 1])
    %   DataSource     - Source of transport block contents (default 'PN9-ITU')

    %   Copyright 2020-2024 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %Coding Flag to enable transport channel coding
        % Specify Coding as a scalar logical. Setting Coding to true
        % enables transport channel coding. The default is true.
        Coding (1,1) logical = true;

        %TargetCodeRate Code rate used to calculate transport block size
        % Specify TargetCodeRate as a scalar or two element vector with
        % values in (0, 1). The second code rate value applies only for a
        % second codeword. This property applies when Coding is true. The
        % default is 526/1024.
        TargetCodeRate = 526/1024;

        %XOverhead Rate matching overhead
        % Specify XOverhead as one of {0, 6, 12, 18}. This property applies
        % when Coding is true. The default is 0.
        XOverhead (1,1) {mustBeMember(XOverhead, [0 6 12 18])} = 0;

        %LimitedBufferRateMatching Enable limited buffer for rate matching
        % Enable limited buffer for rate matching. This property only
        % applies when Coding is true. For UL, the default is false; for
        % DL, the default is true.
        LimitedBufferRateMatching (1,1) logical;

        %MaxNumLayers Maximum number of layers
        % Maximum number of layers configured for or supported by the UE.
        % This property only applies when LimitedBufferRateMatching and
        % Coding is true. The default is 8.
        MaxNumLayers (1,1) {mustBeMember(MaxNumLayers,[1 2 3 4 5 6 7 8])} = 8;

        %MCSTable higher-layer parameter mcs-Table
        % Higher layer parameter 'mcs-Table' configured by the appropriate
        % L3 RRC IE. This property only applies when Coding and
        % LimitedBufferRateMatching are true. The default is 'qam256'.
        MCSTable = 'qam256';

        %RVSequence Redundancy version sequence
        % Specify RVSequence as a scalar, a vector or a two-element cell
        % array containing nonnegative integers. For the case of a
        % two-element cell array, the second value applies only for a
        % second codeword. This property applies when Coding is true. The
        % default is [0 2 3 1].
        RVSequence = [0 2 3 1];

        %DataSource Source of transport block contents
        % Specify DataSource as one of {'PN9-ITU', 'PN9', 'PN11', 'PN15',
        % 'PN23'}, as a cell array containing one of the abovementioned
        % options and a numeric scalar that is the random seed (for example,
        % {'PN9',7}), or as a binary vector. If the seed is not specified,
        % then all shift registers are initialized with an active state.
        % The default is 'PN9-ITU'.
        DataSource = 'PN9-ITU';
    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        DataSource_Values = nr5g.internal.wavegen.DataSourceCommon.DataSource_Options;
    end

    properties (Abstract,Constant,Hidden)
        MCSTable_Values
    end

    % Set properties
    methods
        function obj = set.DataSource(obj,val)
            prop = 'DataSource';
            temp = getDataSource(obj, prop, val);
            obj.(prop) = temp;
        end

        function obj = set.TargetCodeRate(obj,val)
            prop = 'TargetCodeRate';

            coder.internal.errorIf(~any(numel(val) == [1 2]), ...
                'nr5g:nrWaveformGenerator:InvalidSize',prop);
            validateattributes(val, {'numeric'}, ...
                {'real','>',0,'<',1}, ...
                [class(obj) '.' prop], prop);

            obj.(prop) = val;
        end

        function obj = set.MCSTable(obj,val)
            prop = 'MCSTable';
            validateattributes(val,{'char','string'},{'nonempty'},[class(obj) '.' prop],prop);
            mcsTable = validatestring(val,obj.MCSTable_Values,[class(obj) '.' prop],prop);
            obj.(prop) = mcsTable;
        end

        function obj = set.RVSequence(obj,val)
            prop = 'RVSequence';

            if iscell(val)
                validateattributes(val,{'cell'},{'numel', 2},[class(obj) '.' prop],prop);
                temp1 = val{1};
                coder.varsize('temp1',[inf inf],[1 1]);
                validateattributes(temp1,{'numeric'},{'nonempty','vector','integer','nonnegative'},[class(obj) '.' prop],prop);

                temp2 = val{2};
                coder.varsize('temp2',[inf inf],[1 1]);
                validateattributes(temp2,{'numeric'},{'nonempty','vector','integer','nonnegative'},[class(obj) '.' prop],prop);
                temp = {temp1, temp2};
            else
                temp = val;
                coder.varsize('temp',[inf inf],[1 1]);
                validateattributes(temp,{'numeric'},{'nonempty','vector','integer','nonnegative'},[class(obj) '.' prop],prop);
            end

            obj.(prop) = temp;
        end
    end

    methods (Access=public, Hidden=true)

        function validateLBRMProperties(obj,pxschString,idx,warnAsError)
            % Check LBRM related properties and throw warnings when
            % necessary

            if nargin<4
                % By default, do not warn as error
                warnAsError = false;
            end

            % Check MaxNumLayers
            if warnAsError
                % Throw warning as error
                coder.internal.errorIf(obj.MaxNumLayers<obj.NumLayers, ...
                    'nr5g:nrWaveformGenerator:SmallMaxNumLayers',pxschString,idx,obj.NumLayers,obj.MaxNumLayers);
            else
                coder.internal.warningIf(obj.MaxNumLayers<obj.NumLayers, ...
                    'nr5g:nrWaveformGenerator:SmallMaxNumLayers',pxschString,idx,obj.NumLayers,obj.MaxNumLayers);
            end

            % Check MCSTable
            [~,qm_mcs,qm_mod] = nr5g.internal.wavegen.PXSCHConfigBase.getMaxQmModulation(obj.MCSTable,obj.Modulation);
            switch qm_mod
                case 1
                    modstr = 'pi/2-BPSK';
                case 2
                    modstr = 'QPSK';
                case 4
                    modstr = '16QAM';
                case 6
                    modstr = '64QAM';
                case 8
                    modstr = '256QAM';
                otherwise % 10
                    modstr = '1024QAM';
            end
            if warnAsError
                % Throw warning as error
                coder.internal.errorIf(qm_mcs<qm_mod, ...
                    'nr5g:nrWaveformGenerator:SmallMaxQm',pxschString,idx,modstr,obj.MCSTable);
            else
                coder.internal.warningIf(qm_mcs<qm_mod, ...
                    'nr5g:nrWaveformGenerator:SmallMaxQm',pxschString,idx,modstr,obj.MCSTable);
            end

        end

    end
    
    methods (Access=protected)
        % Controls the conditional display of properties
        function flag = isInactiveProperty(obj, prop)
            flag = false;
            
            % TargetCodeRate, XOverhead, and RVSequence only if Coding is 1
            if any(strcmp(prop,{'TargetCodeRate', 'XOverhead', 'RVSequence'}))
                flag = ~obj.Coding;
            end
            
            % PTRS and PTRSPower only if EnablePTRS is 1
            if any(strcmp(prop,{'PTRS', 'PTRSPower'}))
                flag = ~obj.EnablePTRS;
            end
        end
    end

    methods (Static, Hidden)

        function [modLBRM,qm_mcs,qm_mod] = getMaxQmModulation(mcsTable,modulation)
            % Get the modulation scheme corresponding to the maximum
            % modulation order given by MCSTable or Modulation, and the
            % respective modulation order given by MCSTable and Modulation

            % Get modulation order from MCSTable
            qm_mcs = nr5g.internal.getQm(strcat(extractAfter(string(mcsTable),'qam'),"QAM"));

            % Get highest modulation order from Modulation
            if iscell(modulation)
                qm_mod = max(cellfun(@nr5g.internal.getQm,modulation));
            else
                qm_mod = nr5g.internal.getQm(modulation);
            end

            % Get maximum modulation and its corresponding modulation
            % scheme
            maxQm = max(qm_mcs,qm_mod);
            switch maxQm
                case 6
                    modLBRM = '64QAM';
                case 8
                    modLBRM = '256QAM';
                otherwise % 10
                    modLBRM = '1024QAM';
            end

        end

    end
end
