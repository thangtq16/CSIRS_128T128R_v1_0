function out = validatePRACHAndCarrier(carrier, prach, fcnName)
%validatePRACHAndCarrier Validate CARRIER and PRACH input objects.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%   OUT = validatePRACHAndCarrier(CARRIER, PRACH, FCNNAME) checks that the
%   inputs CARRIER and PRACH are valid nrCarrierConfig and nrPRACHConfig
%   objects, respectively. The function also checks that carrier and prach
%   objects satisfy the constraints given in TS 38.211 Table 6.3.3.2-1.
%   These are related to:
%       * SubcarrierSpacing
%       * Number of carrier RBs allocated for PRACH
%   The output OUT is a structure with fields corresponding to the
%   properties of the input PRACH.

%  Copyright 2019-2022 The MathWorks, Inc.

%#codegen

    % Validate carrier and PRACH input types
    validateattributes(carrier, {'nrCarrierConfig'}, {'scalar'}, fcnName, 'Carrier specific configuration object');
    validateattributes(prach, {'nrPRACHConfig'}, {'scalar'}, fcnName, 'PRACH configuration object');

    % Validate the PRACH configuration
    out = validateConfig(prach);

    % Get Table 6.3.3.2-1
    table = nr5g.internal.prach.getTable6332x(1);
    
    % Find the current combination of SubcarrierSpacing in the table
    comb = table.LRA == prach.LRA & ...
           table.PRACHSubcarrierSpacing == prach.SubcarrierSpacing & ...
           table.PUSCHSubcarrierSpacing == carrier.SubcarrierSpacing;
    
    % Validate the combination of SubcarrierSpacing
    errorFlag = ~any(comb);
    coder.internal.errorIf(errorFlag,'nr5g:nrPRACH:InvalidSCSForPRACHAndCarrier',sprintf('%g',prach.SubcarrierSpacing),sprintf('%g',double(carrier.SubcarrierSpacing)));
    
    % Validate the carrier grid size for the given PRACH
    errorFlag = any(carrier.NSizeGrid < table.NRBAllocation(comb));
    coder.internal.errorIf(errorFlag,'nr5g:nrPRACH:InvalidNRBForPRACHAndCarrier',sprintf('%g',min(table.NRBAllocation(comb))),sprintf('%g',double(carrier.NSizeGrid)));

end
