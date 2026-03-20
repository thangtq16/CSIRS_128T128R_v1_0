function ind = nrPSSIndices(varargin)
%nrPSSIndices PSS resource element indices
%   IND = nrPSSIndices() returns a column vector of resource element (RE)
%   indices for the primary synchronization signal (PSS) as defined in TS
%   38.211 Section 7.4.3.1. By default, the indices are returned in 1-based
%   linear indexing form that can directly index elements of a matrix
%   representing the SS/PBCH block (240-by-4). These indices are ordered as
%   the PSS modulation symbols should be mapped. Alternative indexing
%   formats can also be generated.
% 
%   IND = nrPSSIndices(NAME,VALUE,...) specifies additional options as
%   NAME,VALUE pairs to allow control over the format of the indices:
%
%   'IndexStyle'     - 'index' for linear indices (default)
%                      'subscript' for [subcarrier, symbol, antenna] 
%                       subscript row form
%
%   'IndexBase'      - '1based' for 1-based indices (default) 
%                      '0based' for 0-based indices
%    
%   Example:
%   % Generate the 127 resource element indices associated with the PSS
%   % within a single SS/PBCH block.
%   
%   indices = nrPSSIndices();
% 
%   See also nrPSS, nrSSSIndices, nrPBCHDMRSIndices, nrPBCHIndices.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

    narginchk(0,4);
    
    fcnName = 'nrPSSIndices';

    gridsize = [240 4];

    k = (56:182).';
    l = 0;
    ind = k + (l * gridsize(1));

    % Apply options
    if nargin==0
        ind = uint32(ind + 1);
    else
        opts = nr5g.internal.parseOptions(fcnName,{'IndexStyle','IndexBase'},varargin{:});
        ind = nr5g.internal.applyIndicesOptions(gridsize,opts,ind);
    end
    
end
