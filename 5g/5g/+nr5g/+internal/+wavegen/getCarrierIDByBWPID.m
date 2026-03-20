%getCarrierIDByBWPID Get SCS carrier ID from the linked BWP
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   Copyright 2021 The MathWorks, Inc.

%#codegen

function carrierID = getCarrierIDByBWPID(carriers, bwp, bwpID)
    scs = NaN;        % Some inits for codegen; nrXLCarrierConfig.validateConfig()
    carrierID = NaN;  % ensures a carrier exist for each BWP

    % Get subcarrier spacing of this BWP
    for idx = 1:numel(bwp)
        if bwp{idx}.BandwidthPartID == bwpID
            scs = bwp{idx}.SubcarrierSpacing;
            break;
        end
    end

    % Get carrier ID corresponding to the BWP's subcarrier spacing
    for idx = 1:numel(carriers)
        if carriers{idx}.SubcarrierSpacing == scs
            carrierID = idx;
        end
    end
end