function csirsObj = getCSIRSObject(waveCSIRS)
%getCSIRSObject Creates nrCSIRSConfig object
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   CSIRSOBJ = getCSIRSObject(WAVECSIRS) provides the CSI-RS configuration object
%   nrCSIRSConfig CSIRSOBJ, given the input nrWavegenCSIRSConfig object WAVECSIRS.

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    csirsObj = nrCSIRSConfig;
    props = {'CSIRSType', 'RowNumber', 'Density', 'SymbolLocations', ...
        'SubcarrierLocations', 'NumRB', 'RBOffset', 'NID', 'CSIRSPeriod'};
    for idx = 1:length(props)
        csirsObj.(props{idx}) = waveCSIRS.(props{idx});
    end
end