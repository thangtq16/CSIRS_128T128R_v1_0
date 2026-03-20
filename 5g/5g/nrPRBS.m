function [seq,cinit] = nrPRBS(cinit,n,varargin)
%nrPRBS Pseudorandom binary sequence
%   [SEQ,CINIT] = nrPRBS(CINIT,N) returns vector SEQ containing the first N
%   elements of the pseudorandom binary sequence (PRBS) generator defined
%   in TS 38.211 Section 5.2.1, when initialized with 31-bit integer CINIT.
%   If input N contains a pair of elements [P M], then the output sequence
%   will be the M values of the PRBS starting at position P (0-based). For
%   uniformity with the channel specific PRBS functions, the CINIT value is
%   also returned at the output.
%
%   [SEQ,CINIT] = nrPRBS(CINIT,N,NAME,VALUE,...) specifies additional
%   options as NAME,VALUE pairs to allow control over the format of the
%   sequence:
%
%   'MappingType'    - 'binary' to map true to  1, false to 0 (default) 
%                      'signed' to map true to -1, false to 1
%
%   For 'binary', the output data type is logical. For 'signed':
%   'OutputDataType' - 'double' for double precision (default) 
%                      'single' for single precision
%
%   Example 1:
%   % Generate a 1000 bit length sequence with cinit equal to 9.
%   
%   cinit = 9;
%   prbs = nrPRBS(cinit,1000);
%
%   Example 2:
%   % Generate a sequence of length 10 with cinit equal to 9 and using
%   % signed mapping type.
% 
%   cinit = 9;
%   prbs = nrPRBS(cinit,10,'MappingType','signed');
%
%   See also nrPBCHPRBS, nrPDCCHPRBS, nrPDSCHPRBS.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

    narginchk(2,6);

    % Validate inputs
    fcnName = 'nrPRBS';
    validateattributes(cinit,{'numeric'}, ...
        {'scalar','nonnegative','integer'},fcnName,'CINIT');
    validateattributes(n,{'numeric'}, ...
        {'row','nonnegative','integer'},fcnName,'N');
    
    if isempty(coder.target)
        % Use pre-compiled generated code for simulation
        prbsInternal = nr5g.internal.cg_PRBS_uint32(uint32(cinit),uint32(sum(n)));
    else
        % Generate code from MATLAB code
        prbsInternal = nr5g.internal.PRBS(cinit,sum(n));
    end
    
    % If a subsequence was requested, extract the subsequence
    if ~isscalar(n)
        prbs = prbsInternal(end-n(2)+1:end);
    else
        prbs = prbsInternal;
    end
    
    % Apply options
    if nargin>2
        opts = nr5g.internal.parseOptions(fcnName,{'MappingType','OutputDataType'},varargin{:});
        if strcmpi(opts.MappingType,'signed')
            seq = cast(prbs,opts.OutputDataType);
            seq = 1 - 2*seq;
        else
            seq = prbs;
        end
    else
        seq = prbs;
    end

end
