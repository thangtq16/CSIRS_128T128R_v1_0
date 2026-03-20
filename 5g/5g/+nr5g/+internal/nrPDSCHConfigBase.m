classdef nrPDSCHConfigBase < nr5g.internal.pxsch.CommonConfig
  % This is an internal class that may change any time. It contains common
  % properties between nrWavegenPDSCHConfig and nrPDSCHConfig.

%   Copyright 2019-2023 The MathWorks, Inc.

    %#codegen

    % Public, writable properties
    properties
        %Modulation Modulation scheme(s) of codeword(s)
        %   Specify the modulation scheme for the codeword(s). It must be
        %   specified as one of {'QPSK', '16QAM', '64QAM', '256QAM', '1024QAM'}.
        %   Modulation scheme for single codeword can be specified as a
        %   character array or string scalar. Two codewords can be
        %   configured with single modulation scheme, or cell array or
        %   string array of modulation schemes where each value corresponds
        %   to a separate codeword. The default value is 'QPSK'.
        Modulation = 'QPSK';
        
        %ReservedPRB Reserved PRBs and OFDM symbols pattern
        %   Specify the reserved PRBs and OFDM symbols pattern
        %   corresponding to CORESET and SS burst, as a cell array of
        %   object(s), of class <a href="matlab:help('nrPDSCHReservedConfig')">nrPDSCHReservedConfig</a> with the properties:
        %   PRBSet    - Reserved PRB indices (0-based) defined as a vector.
        %               The default value is []
        %   SymbolSet - OFDM symbols associated with reserved PRBs spanning
        %               over one or more slots. The default value is []
        %   Period    - Total number of slots in the pattern period. The
        %               default value is []
        %   The default value is {nrPDSCHReservedConfig}.
        ReservedPRB = {nrPDSCHReservedConfig};

        %PRBSetType Type of indices used in the PRBSet property
        %   Specify the type of the resource block allocation indices
        %   defined in the PRBSet property. When set to 'PRB', the indices
        %   are after the VRB-to-PRB mapping. When set to 'VRB', the
        %   indices are before the VRB-to-PRB mapping. The default value is
        %   'VRB'.
        PRBSetType = 'VRB';

        %VRBToPRBInterleaving VRB-to-PRB interleaving
        %   Specify the flag for VRB-to-PRB interleaving as a logical
        %   scalar. The VRB-to-PRB mapping is interleaved if this property
        %   is true, and is non-interleaved otherwise. The default value is
        %   false.
        VRBToPRBInterleaving (1,1) logical = false;

        %VRBBundleSize Bundle size in terms of number of PRBs
        %   Specify the bundle size for VRB-to-PRB interleaving as a scalar
        %   positive integer. The value must be one of {2, 4}. The default
        %   value is 2.
        VRBBundleSize (1,1) = 2;

        %DMRS PDSCH-specific DM-RS configuration object
        %   Specify the DM-RS configuration object associated with PDSCH.
        %   The default value is a default <a href="matlab:help('nrPDSCHDMRSConfig')"
        %   >nrPDSCHDMRSConfig</a> object.
        DMRS = nrPDSCHDMRSConfig;

        %PTRS PDSCH-specific PT-RS configuration object
        %   Specify the PT-RS configuration object associated with PDSCH.
        %   The default value is a default <a href="matlab:help('nrPDSCHPTRSConfig')"
        %   >nrPDSCHPTRSConfig</a> object.
        PTRS = nrPDSCHPTRSConfig;
    end

    % Hidden properties
    properties (Hidden)
        %NumPorts Number of ports
        %   Number of ports. This is a hidden property and is equal to the
        %   number of layers. This is used in updating the hidden property
        %   NLayers of DMRS property.
        NumPorts = 1;
    end

    % Constant, hidden properties
    properties (Constant,Hidden)
        Modulation_Values = {'QPSK','16QAM','64QAM','256QAM','1024QAM'};
        PRBSetType_Values = {'VRB', 'PRB'};
    end

    methods
        % Constructor
        function obj = nrPDSCHConfigBase(varargin)
          % Get the value of ReservedPRB from the name-value pairs
            reservedPRB = nr5g.internal.parseProp('ReservedPRB',{nrPDSCHReservedConfig},varargin{:});
            dmrs = nr5g.internal.parseProp('DMRS',nrPDSCHDMRSConfig,varargin{:});
            % Get the value of PTRS from the name-value pairs
            ptrs = nr5g.internal.parseProp('PTRS',nrPDSCHPTRSConfig,varargin{:});

            % Support name-value pair arguments when constructing object
            obj@nr5g.internal.pxsch.CommonConfig(...
                'ReservedPRB',reservedPRB, ...
                'DMRS',dmrs, ...
                'PTRS',ptrs, ...
                varargin{:});
        end

        % Self-validate and set properties
        function obj = set.Modulation(obj,val)
            prop = 'Modulation';
            modulation = validateModulation(obj,val);
            % Initialize to empty for varying length in codegen
            if iscell(modulation)
                s = size(modulation);
                obj.(prop) = repmat({''},s(1),s(2));
            else
                obj.(prop) = '';
            end
            obj.(prop) = modulation;
        end

        function obj = set.ReservedPRB(obj,val)
            prop = 'ReservedPRB';
            validateattributes(val,{'cell'},{},...
                [class(obj) '.' prop],prop);
            for idx = 1:numel(val)
                validateattributes(val{idx},{'nrPDSCHReservedConfig'},{'scalar'},[class(obj) '.' prop],prop);
            end
            obj.(prop) = cell(1,numel(val)+1);
            obj.(prop) = val;
        end

        function obj = set.PRBSetType(obj,val)
            prop = 'PRBSetType';
            validateattributes(val,{'char','string'},...
                {'nonempty'},[class(obj) '.' prop], prop);
            temp = validatestring(val,obj.PRBSetType_Values,[class(obj) '.' prop],prop);
            obj.(prop) = temp(1:3);
        end

        function obj = set.VRBBundleSize(obj,val)
            obj.VRBBundleSize = validateVRBBundleSize(obj,val);
        end

        function obj = set.DMRS(obj,val)
            prop = 'DMRS';
            validateattributes(val,{'nrPDSCHDMRSConfig'},{'scalar'},[class(obj) '.' prop],prop);
            obj.(prop) = val;
            % Update the NLayers hidden property in DM-RS with the value of NumPorts
            obj.(prop).NLayers = obj.NumPorts;  %#ok<MCSUP>
        end

        function obj = set.PTRS(obj,val)
            prop = 'PTRS';
            validateattributes(val,{'nrPDSCHPTRSConfig'},{'scalar'},[class(obj) '.' prop],prop);
            obj.(prop) = val;
        end

    end

    methods(Access = public)
        function validateConfig(obj)

            % Validate DM-RS configuration itself
            validateConfig(obj.DMRS);

            % Check whether the number of layers is equal to the length of
            % DM-RS port set, when DM-RS port set is not empty
            if ~isempty(obj.DMRS.DMRSPortSet)
                flag = obj.NumLayers ~= numel(obj.DMRS.DMRSPortSet);
                coder.internal.errorIf(flag,'nr5g:nrPXSCHConfig:InvalidNumLayers',obj.NumLayers,numel(obj.DMRS.DMRSPortSet));
            end

            % Validate the combination of DM-RS port set and PT-RS port
            % set, when PT-RS is enabled
            if obj.EnablePTRS
                validatePTRSPortCompatible(obj.DMRS,obj.PTRS);
            end
        end
    end

    methods (Access = protected)

        function val = validateVRBBundleSize(~,val)
            mustBeMember(val,[2 4]);
        end

    end

end
