function w = getWTDM(NSym,NPorts,ports8tdm)
%getWTDM SRS WTDM sequence defined in TS 38.211 Section 6.4.1.4.2
%   
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

% Copyright 2023 The MathWorks, Inc.

%#codegen

    w = ones(1,NSym,NPorts);

    if ports8tdm
        w(1,2:2:NSym,1+[0 1 4 5]) = 0;
        w(1,1:2:NSym,1+[2 3 6 7]) = 0;
    end

end