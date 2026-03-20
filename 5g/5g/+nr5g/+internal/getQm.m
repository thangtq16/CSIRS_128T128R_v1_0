function qm = getQm(modScheme)
%getQm Return the modulation order
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   QM = nr5g.internal.getQm(MODULATION) returns the modulation order for
%   the modulation scheme specified by MODULATION. MODULATION must be one
%   of 'pi/2-BPSK','BPSK','QPSK','16QAM','64QAM','256QAM','1024QAM',
%   '4096QAM'.
%
%   See also nrSymbolModulate.

%   Copyright 2018-2024 The MathWorks, Inc.

%#codegen

% Get modulation order
    switch upper(modScheme)
        case {'PI/2-BPSK','BPSK'}
            qm = 1;
        case 'QPSK'
            qm = 2;
        case '16QAM'
            qm = 4;
        case '64QAM'
            qm = 6;
        case '256QAM'
            qm = 8;
        case '1024QAM'
            qm = 10;
        otherwise %'4096QAM'
            qm = 12;
    end
    
end
