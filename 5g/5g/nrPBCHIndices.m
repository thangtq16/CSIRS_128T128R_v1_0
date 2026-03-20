function [ind,info] = nrPBCHIndices(ncellid,varargin)
%nrPBCHIndices PBCH resource element indices
%   [IND,INFO] = nrPBCHIndices(NCELLID) returns a column vector of resource
%   element (RE) indices for the physical broadcast channel (PBCH) as
%   defined in TS 38.211 Section 7.4.3.1. Structure INFO is also returned,
%   containing information related to the PBCH indices. NCELLID is the
%   physical layer cell identity (0...1007). By default, the indices are
%   returned in 1-based linear indexing form that can directly index
%   elements of a matrix representing the SS/PBCH block (240-by-4). These
%   indices are ordered as the PBCH modulation symbols should be mapped.
%   Alternative indexing formats can also be generated.
%
%   INFO is a structure containing the fields:
%   G  - A scalar specifying the number of coded and rate matched PBCH 
%        data bits.
%   Gd - A scalar specifying the number of coded and rate matched PBCH data
%        symbols, equal to the number of rows in the PBCH indices.
% 
%   [IND,INFO] = nrPBCHIndices(NCELLID,NAME,VALUE,...) specifies additional
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
%   % Generate the 432 resource element indices associated with the PBCH
%   % symbols within a single SS/PBCH block.
%
%   ncellid = 17;    
%   indices = nrPBCHIndices(ncellid);
%
%   See also nrPBCH, nrPBCHDMRSIndices, nrPSSIndices, nrSSSIndices.

%   Copyright 2018-2022 The MathWorks, Inc.

%#codegen

    narginchk(1,5);

    % Validate physical layer cell identity (0...1007)
    fcnName = 'nrPBCHIndices';
    validateattributes(ncellid,{'numeric'}, ...
        {'scalar','integer','>=',0,'<=',1007},fcnName,'NCELLID');

    gridsize = [240 4];
    K = gridsize(1);

    % DMRS indices for shift of v equal to 0, gives starting indices of
    % groups of 4 indices where 3 are for PBCH, 1 is for PBCH DMRS
    dmrsInd0 = [  (1*K):4:(1*K + 236), ... %l = 1, k =   0,   4 ... 236
                  (2*K):4:(2*K + 44),  ... %l = 2, k =   0,   4 ...  44
            (2*K + 192):4:(2*K + 236), ... %l = 2, k = 192, 196 ... 236
                  (3*K):4:(3*K + 236) ];   %l = 3, k =   0,   4 ... 236

    % 3 indices for PBCH within groups of 4, depending on DMRS shift
    notDmrs = [ 1 2 3; ... % v = 0
                0 2 3; ... % v = 1
                0 1 3; ... % v = 2
                0 1 2; ... % v = 3
                ].';

    % Create overall indices
    v = mod(ncellid,4);
    ind = dmrsInd0 + notDmrs(:,v+1);
    ind = ind(:);

    info = struct('G',numel(ind)*2,'Gd',numel(ind));

    % Apply options
    if nargin==1
        ind = uint32(ind + 1);
    else        
        opts = nr5g.internal.parseOptions(fcnName,{'IndexStyle','IndexBase'},varargin{:});
        ind = nr5g.internal.applyIndicesOptions(gridsize,opts,ind);
    end

end
