function [seq,ncellid] = nrPBCHPRBS(ncellid,v,n,varargin)
%nrPBCHPRBS PBCH pseudorandom scrambling sequence
%   [SEQ,CINIT] = nrPBCHPRBS(NCELLID,V,N) returns vector SEQ containing the
%   first N outputs of the physical broadcast channel (PBCH) scrambling
%   sequence when initialized with NCELLID, the physical layer cell
%   identity (0...1007) and V, the 2 or 3 LSBs (0...7) of the SS/PBCH block
%   index (used to select a portion of the scrambling sequence). The CINIT
%   value used to initialize the pseudorandom binary sequence (PRBS)
%   generator is also returned. The PBCH scrambling sequence is defined in
%   TS 38.211 Section 7.3.3.1.
%
%   [SEQ,CINIT] = nrPBCHPRBS(NCELLID,V,N,NAME,VALUE,...) specifies
%   additional options as NAME,VALUE pairs to allow control over the format
%   of the sequence:
%
%   'MappingType'    - 'binary' to map true to  1, false to 0 (default) 
%                      'signed' to map true to -1, false to 1
%
%   For 'binary', the output data type is logical. For 'signed':
%   'OutputDataType' - 'double' for double precision (default) 
%                      'single' for single precision
%
%   Example:
%   % Generate the PBCH scrambling sequence for the 43rd SS/PBCH block in a
%   % burst (ssbindex is 42).
%
%   ncellid = 17;
%   ssbindex = 42;
%   v = mod(ssbindex,8); % assuming L_max is 64
%   E = 864;             % PBCH bit capacity, TS 38.212 Section 7.1.5
%
%   seq = nrPBCHPRBS(ncellid,v,E);
%
%   See also nrPBCH, nrPBCHDecode, nrPBCHIndices, nrPRBS.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

    narginchk(3,7);

    % Validate inputs
    fcnName = 'nrPBCHPRBS';
    validateattributes(ncellid,{'numeric'}, ...
        {'scalar','integer','>=',0,'<=',1007},fcnName,'NCELLID');
    validateattributes(v,{'numeric'}, ...
        {'scalar','integer','>=',0,'<=',7},fcnName,'V');
    validateattributes(n,{'numeric'}, ...
        {'scalar','integer','nonnegative'},fcnName,'N');

    % Get scrambling sequence
    seq = nrPRBS(ncellid,[v*n n],varargin{:});

end
