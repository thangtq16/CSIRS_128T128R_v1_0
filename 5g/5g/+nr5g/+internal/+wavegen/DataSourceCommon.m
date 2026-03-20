classdef DataSourceCommon
    %DataSourceCommon Object containing centralized DataSource accepted values and set method
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.

    %   Copyright 2021 The MathWorks, Inc.

    %#codegen

    % Constant, hidden properties
    properties (Constant,Hidden)
        DataSource_Options = {'PN9', 'PN9-ITU', 'PN11', 'PN15', 'PN23'}
    end

    methods (Access=protected)
        % Set and validate any input data source
        function out = getDataSource(obj, prop, val)
            
            if iscell(val)
                validateattributes(val,{'cell'},{'numel', 2},[class(obj) '.' prop],prop);
                temp2 = validatestring(val{1}, obj.DataSource_Options,[class(obj) '.' prop],prop);
                validateattributes(val{2},{'numeric'},{'scalar', 'integer', 'nonnegative'},[class(obj) '.' prop],prop);
                out = {temp2 val{2}};
            elseif ischar(val) || isstring(val)
                out = validatestring(val, obj.DataSource_Options,[class(obj) '.' prop],prop);
            else
                out = val;
                coder.varsize('temp',[inf inf],[1 1]);
                validateattributes(out,{'numeric'},{'vector', 'binary'},[class(obj) '.' prop],prop);
            end
        end
    end
end
