%getCarrierCfgObject Recreate carrier configuration object from SCSCarrier or BWP parameters
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen

function carrierCfg = getCarrierCfgObject(in, NCellID, varargin)

    guardBandsArgIndex = 0;
    if isa(in,'nrSCSCarrierConfig')
        nStart = in.NStartGrid;
        nSize = in.NSizeGrid;
        cp = varargin{1};
        if nargin > 3
            guardBandsArgIndex = 2;
        end
    else % nrWavegenBWPConfig
        nStart = in.NStartBWP;
        nSize = in.NSizeBWP;
        cp = in.CyclicPrefix;
        if nargin > 2
            guardBandsArgIndex = 1;
        end
    end

    carrierCfg = nrCarrierConfig;
    carrierCfg.SubcarrierSpacing = in.SubcarrierSpacing;
    carrierCfg.NCellID = NCellID;
    carrierCfg.NSizeGrid = nSize;
    carrierCfg.NStartGrid = nStart;
    carrierCfg.CyclicPrefix = cp;

    % Add intracell guard bands if provided
    if guardBandsArgIndex > 0
        carrierCfg.IntraCellGuardBands = varargin{guardBandsArgIndex};
    end
end