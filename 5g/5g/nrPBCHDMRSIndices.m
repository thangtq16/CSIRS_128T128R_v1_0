function ind = nrPBCHDMRSIndices(ncellid,varargin)
%nrPBCHDMRSIndices PBCH DM-RS resource element indices
%   IND = nrPBCHDMRSIndices(NCELLID) returns a column vector of resource
%   element (RE) indices for the physical broadcast channel (PBCH)
%   demodulation reference signal (DM-RS) as defined in TS 38.211 Section
%   7.4.3.1. NCELLID is the physical layer cell identity (0...1007). By
%   default, the indices are returned in 1-based linear indexing form that
%   can directly index elements of a matrix representing the SS/PBCH block
%   (240-by-4). These indices are ordered as the PBCH DM-RS modulation
%   symbols should be mapped. Alternative indexing formats can also be
%   generated.
%
%   IND = nrPBCHDMRSIndices(NCELLID,NAME,VALUE,...) specifies additional
%   options as NAME,VALUE pairs to allow control over the format of the
%   indices:
%
%   'IndexStyle'     - 'index' for linear indices (default)
%                      'subscript' for [subcarrier, symbol, antenna] 
%                       subscript row form
%
%   'IndexBase'      - '1based' for 1-based indices (default) 
%                      '0based' for 0-based indices
%
%   Example:
%   % Generate the 144 resource element indices associated with the PBCH
%   % DM-RS symbol within a single SS/PBCH block.
%
%   ncellid = 17;
%   indices = nrPBCHDMRSIndices(ncellid);
% 
%   See also nrPBCHDMRS, nrPBCHIndices, nrPSSIndices, nrSSSIndices.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

    narginchk(1,5);

    % Validate physical layer cell identity (0...1007)
    fcnName = 'nrPBCHDMRSIndices';
    validateattributes(ncellid,{'numeric'}, ...
        {'scalar','integer','>=',0,'<=',1007},fcnName,'NCELLID');

    gridsize = [240 4];
    K = gridsize(1);

    v = mod(ncellid,4);
    ind = v + ...
                [ (1*K):4:(1*K + 236), ... %l = 1, k =   0 + v ... 236 + v
                  (2*K):4:(2*K + 44),  ... %l = 2, k =   0 + v ...  44 + v
            (2*K + 192):4:(2*K + 236), ... %l = 2, k = 192 + v ... 236 + v
                  (3*K):4:(3*K + 236) ].'; %l = 3, k =   0 + v ... 236 + v

    % Apply options
    if nargin==1
        ind = uint32(ind + 1);
    else
        opts = nr5g.internal.parseOptions(fcnName,{'IndexStyle','IndexBase'},varargin{:});
        ind = nr5g.internal.applyIndicesOptions(gridsize,opts,ind);
    end

end
