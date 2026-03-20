function [seq,cinit] = nrPUSCHPRBS(nid,rnti,varargin)
%nrPUSCHPRBS PUSCH pseudorandom scrambling sequence
%   [SEQ,CINIT] = nrPUSCHPRBS(NID,RNTI,N) returns vector SEQ containing the
%   first N outputs of the physical uplink shared channel (PUSCH)
%   scrambling sequence when initialized with scrambling identity NID
%   (0...1023) and Radio Network Temporary Identifier RNTI (0...65535). The
%   CINIT value used to initialize the pseudorandom binary sequence (PRBS)
%   generator is also returned. The PUSCH scrambling sequence is defined in
%   TS 38.211 Section 6.3.1.1.
%
%   [SEQ,CINIT] = nrPUSCHPRBS(NID,RNTI,NRAPID,N) initializes the scrambling
%   sequence also with the random access preamble index, NRAPID, needed for
%   msgA on PUSCH.
%
%   [SEQ,CINIT] = nrPUSCHPRBS(NID,RNTI,NRAPID,N,Q) initializes the
%   scrambling sequence with the codeword number Q (0 (default) or 1). In
%   this syntax, when Q is 1, NRAPID must be empty.
%
%   [SEQ,CINIT] = nrPUSCHPRBS(...,NAME,VALUE) specifies additional options
%   as NAME,VALUE pairs to allow control over the format of the sequence:
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
%   Example 1:
%   % Generate a PUSCH scrambling sequence of length 300 for cell identity
%   % 17 and RNTI 120.
%
%   ncellid = 17;
%   rnti = 120;
%   n = 300;
%
%   seq = nrPUSCHPRBS(ncellid,rnti,n);
%
%   Example 2:
%   % Generate a PUSCH scrambling sequence of length 300 for cell identity
%   % 17, RNTI 120, and NRAPID 63 for msgA on PUSCH.
%
%   ncellid = 17;
%   rnti = 120;
%   nrapid = 63;
%   n = 300;
%
%   seq = nrPUSCHPRBS(ncellid,rnti,nrapid,n);
%
%   See also nrPUSCHScramble, nrPUSCHDescramble, nrPRBS.

%   Copyright 2018-2023 The MathWorks, Inc.

%#codegen
    
    narginchk(3,9);
    fcnName = 'nrPUSCHPRBS';
    if nargin > 3 && isnumeric(varargin{2}) % nrPUSCHPRBS(nid,rnti,nrapid,n,...)
        nrapid = varargin{1};
        n = varargin{2};
        if nargin>4 && isnumeric(varargin{3}) % nrPUSCHPRBS(nid,rnti,nrapid,n,q,...)
            q = varargin{3};
            firstoptarg = 4;
            validateattributes(q,{'numeric'},{'scalar', ...
                'integer','>=',0,'<=',1},fcnName,'q');
            if ~isempty(nrapid)
                coder.internal.errorIf(q==1,'nr5g:nrPUSCHPRBS:InvalidNRAPIDFor2CW',nrapid(1));
            end
        else % nrPUSCHPRBS(nid,rnti,nrapid,n,...)
            q = 0;
            firstoptarg = 3;
        end
    else % nrPUSCHPRBS(nid,rnti,n,...)
        nrapid = [];
        n = varargin{1};
        q = 0;
        firstoptarg = 2;
    end
    
    % Validate inputs
    validateattributes(nid,{'numeric'},{'scalar', ...
        'integer','>=',0,'<=',1023},fcnName,'NID');
    validateattributes(rnti,{'numeric'},{'scalar', ...
        'integer','>=',0,'<=',65535},fcnName,'RNTI');
    if ~isempty(nrapid)
        validateattributes(nrapid,{'numeric'}, {'scalar', ...
            'integer','>=',0,'<=',63},fcnName,'NRAPID');
    end
    validateattributes(n,{'numeric'}, {'scalar', ...
        'nonnegative','integer'},fcnName,'n');
    
    % Calculate scrambler initialization
    if isempty(nrapid)
        cinit = (double(rnti) * 2^15) + (double(q) * 2^14) + double(nid);
    else
        cinit = (double(rnti) * 2^16) + (double(nrapid) * 2^10) + double(nid);
    end
    
    % Get scrambling sequence
    seq = nrPRBS(cinit,n,varargin{firstoptarg:end});
    
end