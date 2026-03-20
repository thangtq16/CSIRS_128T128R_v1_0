function validateParameters(prmName,prm,fcnName)
% nr5g.internal.validateParameters Validate common 5G parameters
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   Example: 
%
%   L = 3;
%   nr5g.internal.validateParameters('ListLength',L,'nrPolarDecode')

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

    switch prmName
        case 'ListLength'
            % Validate decoding list length (L) to be a power of two
            validateattributes(prm, {'numeric'}, ...
                {'scalar','integer','positive','finite'},fcnName,'L');
            coder.internal.errorIf( floor(log2(prm))~=log2(prm), ...
                'nr5g:nrPrmListLength:notPow2',prm);
            
        case 'RV'
            % Validate RV to be scalar integer between [0,3]
            validateattributes(prm,{'numeric'},...
                {'scalar','integer','>=',0,'<=',3},fcnName,'RV');

        case 'HARQID'
            % Validate HARQ ID to be scalar integer between [0,31]
            validateattributes(prm,{'numeric'}, ...
                    {'scalar','integer','>=',0,'<=',31},fcnName,'HARQID');

        case 'CBGTI'
            % Validate cbgti to be a binary column vector or 2-column
            % binary matrix
            validateattributes(prm,{'logical','numeric'}, ...
                {'binary','nonempty'},fcnName,'CBGTI');
            coder.internal.errorIf(~any(size(prm,2)==[1 2]), ...
                'nr5g:nrXLSCH:InvalidNumColCBGTI');

        % Add more case statements for other shared parameters
    end

end