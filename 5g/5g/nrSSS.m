function sym = nrSSS(ncellid,varargin)
%nrSSS Secondary synchronization signal
%   SYM = nrSSS(NCELLID) returns a real column vector SYM containing the
%   secondary synchronization signal (SSS) symbols as defined in TS 38.211
%   Section 7.4.2.3, given physical layer cell identity NCELLID (0...1007).
%
%   SYM = nrSSS(NCELLID,NAME,VALUE) specifies additional options as one or
%   more NAME,VALUE pairs to allow control over the format of the symbols:
%
%   'OutputDataType' - 'double' for double precision (default) 
%                      'single' for single precision
%
%   Example:
%   % Generate the 127 secondary synchronization signal symbols (BPSK) for 
%   % a given cell ID. The SSS is transmitted in symbol #2 (0-based) of a 
%   % SS/PBCH block.
%   
%   ncellid = 17;
%   sss = nrSSS(ncellid);
% 
%   See also nrSSSIndices, nrPSS, nrPBCHDMRS, nrPBCH.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

    narginchk(1,3);

    % Validate physical layer cell identity (0...1007)
    fcnName = 'nrSSS';
    validateattributes(ncellid,{'numeric'}, ...
        {'scalar','integer','>=',0,'<=',1007},fcnName,'NCELLID');
    
    % d_SSS0(m)
    d_SSS0 = [...
     -1   1   1   1   1   1   1  -1   1   1  -1   1   1  -1  -1   1  -1 ...
      1   1  -1  -1  -1  -1   1  -1  -1  -1   1   1   1   1  -1  -1  -1 ...
     -1  -1  -1  -1   1   1   1  -1  -1  -1   1  -1  -1   1   1   1  -1 ...
      1  -1   1   1  -1   1  -1  -1  -1  -1  -1   1  -1   1  -1   1  -1 ...
      1   1   1   1  -1   1  -1  -1   1  -1  -1  -1  -1   1   1  -1  -1 ...
     -1   1   1  -1   1  -1   1  -1  -1   1   1  -1  -1   1   1   1   1 ...
      1  -1  -1   1  -1  -1   1  -1   1  -1  -1  -1   1  -1   1   1   1 ...
     -1  -1   1   1  -1   1   1   1].';

    % d_SSS1(m)
    d_SSS1 = [...
     -1   1   1   1   1   1   1  -1   1   1   1   1   1  -1  -1   1   1 ...
      1   1  -1   1  -1   1   1   1  -1  -1  -1  -1   1   1  -1   1   1 ...
      1  -1   1  -1  -1   1   1  -1  -1  -1   1  -1   1  -1   1   1  -1 ...
     -1  -1  -1  -1   1  -1   1   1   1   1  -1  -1  -1   1   1   1  -1 ...
      1   1  -1   1   1  -1  -1   1  -1  -1   1  -1   1  -1  -1   1  -1 ...
     -1  -1  -1   1  -1  -1   1   1   1  -1  -1   1  -1   1   1  -1   1 ...
     -1  -1  -1   1  -1  -1  -1   1   1  -1  -1   1   1  -1   1  -1   1 ...
     -1   1  -1  -1  -1  -1  -1  -1].';

    n1 = fix(ncellid/3);
    n2 = mod(ncellid,3);
    m0 = 15*fix(n1/112) + 5*n2;
    m1 = mod(n1,112);
    
    seq0 = d_SSS0(1 + mod(m0 + (0:126),127));
    seq1 = d_SSS1(1 + mod(m1 + (0:126),127));
    
    seq = seq0.*seq1;

    % Apply options
    if nargin>1
        opts = nr5g.internal.parseOptions( ...
            fcnName,{'OutputDataType'},varargin{:});
        sym = cast(seq,opts.OutputDataType);
    else
        sym = seq;
    end
    
end
