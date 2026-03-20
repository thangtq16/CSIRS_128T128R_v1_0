function sym = getSymbols(prach,info,opts)
%getSymbols Get the PRACH symbols.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.

%  Copyright 2022 The MathWorks, Inc.

%#codegen

    % Set up an empty output vector
    sym = complex(zeros(0,1,opts.OutputDataType));
    
    % If PRACH is active in this subframe:
    if nr5g.internal.prach.isActive(prach)
        
        % Create Zadoff-Chu sequence, apply appropriate cyclic shift,
        % perform DFT.
        x_u = zadoffChuSeq(info.RootSequence(1), prach.LRA);
        C_v = info.CyclicShift;
        x_uv = circshift(x_u, [-C_v 0]);
        y_uv = fft(x_uv);
        
        % Generate the output symbol vector
        L = nr5g.internal.prach.getNumOFDMSymbols(prach);
        sym = cast(repmat(y_uv(1:prach.LRA,1), L(1), 1), opts.OutputDataType);
    end

end