function polyIndex = validateCRCinputs(in,poly,mask,fcnName)
% validateCRCinputs validates the inputs of CRC encoding and decoding
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%    polyIndex = nr5g.internal.validateCRCinputs(in,poly,mask,fcnName)
%    validates the inputs as specified below and returns an index associated
%    with the polynomial:
%    1) Input data block (in) should be a matrix (int8, double or
%    logical)
%    2) CRC polynomial (poly) should be a character row vector or a scalar
%    string from one of the following ('6','11','16','24A','24B','24C')
%    3) Mask should be a scalar nonnegative integer
%    4) Function name (fcnName) should be the name of calling function

%  Copyright 2018-2019 The MathWorks, Inc.

%#codegen

    if strcmpi(fcnName,'nrCRCEncode')
      input = 'BLK';
    else
      input = 'BLKCRC';
    end

    validateattributes(in,{'double','int8','logical'},{'2d','real','nonnan'},...
        fcnName,input);

    polyList = {'6','11','16','24A','24B','24C'};
    poly = validatestring(poly,polyList,fcnName,'POLY');
    polyIndex = strcmpi(poly,polyList);

    validateattributes(mask,{'numeric'},...
        {'scalar','integer','nonnegative'},fcnName,'MASK');
end