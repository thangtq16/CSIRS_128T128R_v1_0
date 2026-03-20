function sym = nrPSS(ncellid,varargin) 
%nrPSS Primary synchronization signal
%   SYM = nrPSS(NCELLID) returns a real column vector SYM containing the
%   primary synchronization signal (PSS) symbols as defined in TS 38.211
%   Section 7.4.2.2, given physical layer cell identity NCELLID (0...1007).
%
%   SYM = nrPSS(NCELLID,NAME,VALUE) specifies an additional option as a
%   NAME,VALUE pair to allow control over the format of the symbols:
%
%   'OutputDataType' - 'double' for double precision (default) 
%                      'single' for single precision
% 
%   Example:
%   % Generate the 127 primary synchronization signal symbols (BPSK) for a 
%   % given cell ID. The PSS is transmitted in symbol #0 of a SS/PBCH 
%   % block.
%   
%   ncellid = 17;
%   pss = nrPSS(ncellid);
% 
%   See also nrPSSIndices, nrSSS, nrPBCHDMRS, nrPBCH.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

    narginchk(1,3);

    % Validate physical layer cell identity (0...1007)
    fcnName = 'nrPSS';
    validateattributes(ncellid,{'numeric'}, ...
        {'scalar','integer','>=',0,'<=',1007},fcnName,'NCELLID');

    % d_PSS(m)
    d_PSS = [...
      1  -1  -1   1  -1  -1  -1  -1   1   1  -1  -1  -1   1   1  -1   1 ...
     -1   1  -1  -1   1   1  -1  -1   1   1   1   1   1  -1  -1   1  -1 ...
     -1   1  -1   1  -1  -1  -1   1  -1   1   1   1  -1  -1   1   1  -1 ...
      1   1   1  -1   1   1   1   1   1   1  -1   1   1  -1   1   1  -1 ...
     -1   1  -1   1   1  -1  -1  -1  -1   1  -1  -1  -1   1   1   1   1 ...
     -1  -1  -1  -1  -1  -1  -1   1   1   1  -1  -1  -1   1  -1  -1   1 ...
      1   1  -1   1  -1   1   1  -1   1  -1  -1  -1  -1  -1   1  -1   1 ...
     -1   1  -1   1   1   1   1  -1].';

    n2 = mod(ncellid,3);

    seq = d_PSS(1 + mod(43*n2 + (0:126),127));

    % Apply options
    if nargin>1
        opts = nr5g.internal.parseOptions( ...
            fcnName,{'OutputDataType'},varargin{:});
        sym = cast(seq,opts.OutputDataType);
    else
        sym = seq;
    end

end
