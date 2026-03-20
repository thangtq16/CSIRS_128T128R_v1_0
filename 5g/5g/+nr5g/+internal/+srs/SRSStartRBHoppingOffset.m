function kHop = SRSStartRBHoppingOffset(kBarHop,PF)
%SRSStartRBHoppingOffset SRS Start RB hopping offset table
%   
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   kHop = SRSStartRBHoppingOffset(KBARHOP,PF) returns the SRS hopping
%   offset corresponding to the frequency scaling factor {1,2,4} and the
%   intermediate hopping offset kBarHop {0,1,2,3}. See TS 38.211 Table
%   6.4.1.4.3-3.
% 
%   kHop = SRSStartRBHoppingOffset returns the offset table as a 4-by-4
%   cell array.
%
%   Example: 
%   kHop = nr5g.internal.srs.SRSStartRBHoppingOffset(2,1)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

   confTable = { 0   0      0       0;
                 1   NaN    1       2;
                 2   NaN    NaN     1;
                 3   NaN    NaN     3};
    
    if nargin == 2
        kHop = [confTable{kBarHop+1,PF == [NaN 1 2 4]}];
    else
        kHop = confTable;
    end

end