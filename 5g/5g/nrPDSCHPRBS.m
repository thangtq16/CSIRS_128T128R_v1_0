function [seq,cinit] = nrPDSCHPRBS(nid,rnti,q,n,varargin)
%nrPDSCHPRBS PDSCH pseudorandom scrambling sequence
%   [SEQ,CINIT] = nrPDSCHPRBS(NID,RNTI,Q,N) returns vector SEQ containing
%   the first N outputs of the physical downlink shared channel (PDSCH)
%   scrambling sequence when initialized with scrambling identity NID
%   (0...1023), Radio Network Temporary Identifier RNTI (0...65535), and
%   codeword number Q (0,1). The CINIT value used to initialize the
%   pseudorandom binary sequence (PRBS) generator is also returned. The
%   PDSCH scrambling sequence is defined in TS 38.211 Section 7.3.1.1.
%
%   [SEQ,CINIT] = nrPDSCHPRBS(NID,RNTI,Q,N,NAME,VALUE) specifies additional
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
%   % Generate the PDSCH scrambling sequence for cell identity 17.
%
%   ncellid = 17;
%   rnti = 120;
%   q = 0;
%   n = 300;
%
%   seq = nrPDSCHPRBS(ncellid,rnti,q,n);
%
%   See also nrPDSCH, nrPDSCHDecode, nrPRBS.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    narginchk(4,8);

    % Validate inputs
    fcnName = 'nrPDSCHPRBS';
    validateattributes(nid,{'numeric'},{'scalar', ...
        'integer','>=',0,'<=',1023},fcnName,'NID');
    validateattributes(rnti,{'numeric'},{'scalar', ...
        'integer','>=',0,'<=',65535},fcnName,'RNTI');
    validateattributes(q,{'numeric'}, {'scalar', ...
        'integer','>=',0,'<=',1},fcnName,'q');
    validateattributes(n,{'numeric'}, {'scalar', ...
        'nonnegative','integer'},fcnName,'n');

    cinit = (double(rnti) * 2^15) + (double(q) * 2^14) + double(nid);

    % Get scrambling sequence
    seq = nrPRBS(cinit,n,varargin{:});

end
