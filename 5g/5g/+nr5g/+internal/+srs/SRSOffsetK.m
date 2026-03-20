function kOffset = SRSOffsetK(KTC,NumSRSSymbols)
%SRSOffsetK SRS k_offset table
%   
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   KOFFSET = SRSOffsetK(KTC,NUMSRSSYMBOLS) returns the SRS offset KOFFSET
%   corresponding to the transmission comb KTC {2,4,8} and number of SRS
%   symbols NUMSRSSYMBOLS {1,2,4,8,12}. See TS 38.211 Table 6.4.1.4.3-2.
%   Both KTC and NUMSRSSYMBOLS can be vectors.
% 
%   KOFFSET = SRSOffsetK returns the offset table as a 3-by-6 cell array.
%
%   Example: 
%   kOffset = nr5g.internal.srs.SRSOffsetK(4,8)

% Copyright 2020-2023 The MathWorks, Inc.

%#codegen

   confTable = {...
                2,   0,     [0,1], [0,1,0,1],          NaN(1,8),                 NaN(1,12);
                4, NaN,     [0,2], [0,2,1,3], [0,2,1,3,0,2,1,3], [0,2,1,3,0,2,1,3,0,2,1,3];
                8, NaN,  NaN(1,2), [0,4,2,6], [0,4,2,6,1,5,3,7], [0,4,2,6,1,5,3,7,0,4,2,6]};
    
    if nargin == 2
        nSyms = [1 2 4 8 12];
        kOffset = confTable{log2(double(KTC)),[NaN nSyms] == NumSRSSymbols};
    else
        kOffset = confTable;
    end

end