function guardband = getGuardband(fr,cbw,scs)
% TS 38.104 Section 5.3.3 
% Minimum guardband and transmission bandwidth configuration

%   Copyright 2019-2023 The MathWorks, Inc.

%#codegen

    % FR1   
    % DL: Table 5.3.3-1: Minimum guardband [kHz] (FR1) (TS 38.104)
    % UL: Table 5.3.3-1: Minimum guardband [kHz] (FR1) (TS 38.101-1) 
    cbwFR1    = [    5     10    15     20     25     30     35     40     45     50     60     70     80     90    100];
    guardsFR1 = [242.5  312.5 382.5  452.5  522.5  592.5  572.5  552.5  712.5  692.5    NaN    NaN    NaN    NaN    NaN; ...  % 15 kHz
                 505.0  665.0 645.0  805.0  785.0  945.0  925.0  905.0 1065.0 1045.0  825.0  965.0  925.0  885.0  845.0; ...  % 30 kHz
                   NaN 1010.0 990.0 1330.0 1310.0 1290.0 1630.0 1610.0 1590.0 1570.0 1530.0 1490.0 1450.0 1410.0 1370.0];     % 60 kHz
    scsFR1 = [15 30 60].';

    % FR2
    % DL: Tables 5.3.3-2 / 5.3.3-2a: Minimum guardband [kHz] (FR2) (TS 38.104)
    % UL: Table 5.3.3-1: Minimum guardband [kHz] (FR2) (TS 38.101-2) 
    cbwFR2    = [  50  100  200  400   800  1600   2000];
    guardsFR2 = [1210 2450 4930  NaN   NaN   NaN    NaN;   % 60 kHz  (Table 5.3.3-2 FR2-1)
                 1900 2420 4900 9860   NaN   NaN    NaN;   % 120 kHz (Table 5.3.3-2 FR2-1 and Table 5.3.3-2a, FR2-2)
                  NaN  NaN  NaN 9680 42640 85520    NaN;   % 480 kHz (Table 5.3.3-2a, FR2-2)
                  NaN  NaN  NaN 9440 42400 85280 147040];  % 960 kHz (Table 5.3.3-2a, FR2-2)
    scsFR2 = [60 120 480 960].';
    
    % Return value in MHz
    if (strcmpi(fr,'FR1'))
        guardband = guardsFR1(scsFR1==scs,cbwFR1==cbw) / 1e3;
    else % FR2 (FR2-1, FR2-2)
        guardband = guardsFR2(scsFR2==scs,cbwFR2==cbw) / 1e3;
    end
    
end