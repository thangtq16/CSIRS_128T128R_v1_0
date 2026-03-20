function [seq,cinit] = nrPUCCHPRBS(nid,rnti,n,varargin)
%nrPUCCHPRBS PUCCH pseudorandom scrambling sequence
%   [SEQ,CINIT] = nrPUCCHPRBS(NID,RNTI,N) returns vector SEQ containing the
%   first N outputs of the physical uplink control channel (PUCCH)
%   scrambling sequence when initialized with scrambling identity NID
%   (0...1023) and Radio Network Temporary Identifier RNTI (0...65535). The
%   CINIT value used to initialize the pseudorandom binary sequence (PRBS)
%   generator is also returned. The PUCCH scrambling sequence is defined in
%   TS 38.211 Section 6.3.2.5.1/6.3.2.6.1.
%
%   [SEQ,CINIT] = nrPUCCHPRBS(NID,RNTI,N,NAME,VALUE) specifies additional
%   options as NAME,VALUE pairs to allow control over the format of the
%   sequence:
%
%   'MappingType'    - 'binary' to map true to  1, false to 0 (default) 
%                      'signed' to map true to -1, false to 1
%
%   For 'binary', the output data type is logical. For 'signed', the output
%   data type can be configured as:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   Example:
%   % Generate a PUCCH scrambling sequence of length 300 for cell identity
%   % 17 and rnti 120.
%
%   ncellid = 17;
%   rnti = 120;
%   n = 300;
%
%   seq = nrPUCCHPRBS(ncellid,rnti,n);
%
%   See also nrPRBS, nrPUCCH2, nrPUCCH3, nrPUCCH4.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    narginchk(3,7);

    % Validate inputs
    fcnName = 'nrPUCCHPRBS';
    validateattributes(nid,{'numeric'},{'scalar',...
        'real','integer','>=',0,'<=',1023},fcnName,'NID');
    validateattributes(rnti,{'numeric'},{'scalar',...
        'real','integer','>=',0,'<=',65535},fcnName,'RNTI');
    validateattributes(n,{'numeric'}, {'scalar',...
        'real','nonnegative','integer'},fcnName,'n');

    % Calculate scrambler initialization
    cinit = (double(rnti) * 2^15) + double(nid);

    % Get scrambling sequence
    seq = nrPRBS(cinit,n,varargin{:});

end
