classdef BWPSizeStart
    %BWPSizeStart Common configuration object for physical channels and
    %signals
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   BWPSizeStart properties (configurable):
    %
    %   NSizeBWP         - Size of the bandwidth part (BWP) in terms of
    %                      number of physical resource blocks (PRBs)
    %                      (1...275) (default [])
    %   NStartBWP        - Starting PRB index of BWP relative to common
    %                      resource block 0 (CRB 0) (0...2473) (default [])
   
    %   Copyright 2020-2023 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %NSizeBWP Size of bandwidth part (BWP) in terms of number of
        %physical resource blocks (PRBs)
        %   Specify the size of BWP in terms of number of PRBs as a scalar
        %   positive integer. The value must be in the range 1...275. Use
        %   empty ([]) to allow this property to be equal to <a href="matlab:help('nrCarrierConfig/NSizeGrid')">NSizeGrid</a> of
        %   nrCarrierConfig. The default value is [].
        NSizeBWP = [];

        %NStartBWP Starting PRB index of BWP relative to CRB 0
        %   Specify the starting PRB index of BWP relative to CRB 0 as a
        %   scalar nonnegative integer. The value must be in the range
        %   0...2473. Use empty ([]) to allow this property to be equal to
        %   <a href="matlab:help('nrCarrierConfig/NStartGrid')">NStartGrid</a> of nrCarrierConfig. The default value is [].
        NStartBWP = [];
    end

    methods
       
        % Self-validate and set properties
        function obj = set.NSizeBWP(obj,val)
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateNSizeBWP(obj,temp);
            end
            obj.NSizeBWP = temp;
        end

        function obj = set.NStartBWP(obj,val)
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateNStartBWP(obj,temp);
            end
            obj.NStartBWP = temp;
        end

    end

    methods (Access = protected)

        function val = validateNSizeBWP(obj,val)
            prop = 'NSizeBWP';
            validateattributes(val,{'numeric'},{'scalar','integer','positive','<=',275},[class(obj) '.' prop],prop);
        end

        function val = validateNStartBWP(obj,val)
            prop = 'NStartBWP';
             validateattributes(val,{'numeric'},{'scalar','integer','nonnegative','<=',2473},[class(obj) '.' prop],prop);
        end

    end

end
