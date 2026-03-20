%getFrequencyOffsetk0 SCS carrier frequency offset k0
%   K0 = getFrequencyOffsetk0(CARRIERS) returns the frequency offset K0
%   associated to the nrSCSCarrierConfig objects or carrier configuration
%   structures CARRIERS.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%

%   Copyright 2021 The MathWorks, Inc.

%#codegen

function k0 = getFrequencyOffsetk0(carriers)

    if iscell(carriers) && isa(carriers{1}, 'nrSCSCarrierConfig')
        NStartGrids = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers,'NStartGrid','double');
        NSizeGrids = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers,'NSizeGrid','double');
        SubcarrierSpacings = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(carriers,'SubcarrierSpacing','double');
    else % featured examples
        NStartGrids = [carriers.RBStart];
        NSizeGrids = [carriers.NRB];
        SubcarrierSpacings = [carriers.SubcarrierSpacing];
    end
    [~, idx] = max(SubcarrierSpacings);
    k0offset = (NStartGrids(idx) + NSizeGrids(idx)/2)*12*(SubcarrierSpacings(idx)/15);
    
    k0 = (NStartGrids + NSizeGrids/2)*12 - (k0offset./(SubcarrierSpacings/15));
    
end