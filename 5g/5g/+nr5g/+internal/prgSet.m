%prgSet 1-based PRG indices for each RB in the carrier grid
%    
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2023 The MathWorks, Inc.

%#codegen

function prgset = prgSet(NRB,nstartgrid,NPRG)

    Pd_BWP = 2^ceil(log2((NRB + nstartgrid) / NPRG));
    prgset = repmat(1:NPRG,[Pd_BWP 1]);
    prgset = prgset(nstartgrid + (1:NRB).');

end
