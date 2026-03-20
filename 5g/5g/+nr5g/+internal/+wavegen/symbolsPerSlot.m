%symbolsPerSlot Establish the number of symbols per slot from the cyclic prefix
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2021 The MathWorks, Inc.

%#codegen

function symbs = symbolsPerSlot(config)

    cp = config.CyclicPrefix;
    cpoptions = {'Normal','Extended'};
    symbs = sum(strcmpi(cp,cpoptions) .* [14 12]);

end