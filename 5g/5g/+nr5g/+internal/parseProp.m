function val = parseProp(prop,defVal,varargin)
%parseProp Parse property values from set of Name-Value pairs
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   VAL = nr5g.internal.parseProp(PROP,DEFVAL,Name,Value,...) outputs the
%   VAL for the specified property PROP if the Name-Value pairs specified
%   have PROP as one of the Name(s). The last matching Name-Value pair's
%   value is output. If not present, the default value, DEFVAL, is output
%   as the value.
%
%   See also nrPDCCHConfig, nrPDSCHConfig.

%   Copyright 2019 The MathWorks, Inc.

%#codegen

    propPresence = 0;
    for i = nargin-3:-2:1
        if strcmp(varargin{i},prop)
            val = varargin{i+1};
            propPresence = 1;
            break
        end
    end
    if ~propPresence
        val = defVal;
    end
end
