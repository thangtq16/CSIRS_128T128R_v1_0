function MCSTable = getMCSTable(tableIdx)
%getMCSTable Get TS 38.214 Tables 5.1.3.1-1, 5.1.3.1-2, 5.1.3.1-3
% and 5.1.3.1-4.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%    MCSTable = nr5g.internal.getMCSTable(tableIdx) returns MCS Tables
%    as defined in TS 38.214 Tables 5.1.3.1-x, where x is between 1-4

%    Copyright 2023 The MathWorks, Inc.

%#codegen

MCSIndex = (0:31)';
Qm = [repmat(2,10,1);repmat(4,7,1);repmat(6,12,1);2;4;6];
TargetCodeRate=[120;157;193;251;308;379;449;526;602;679;340;378;434;490;553;616;658;438;466;517;567;616;666;719;772;822;873;910;948;NaN;NaN;NaN]/1024;
SpectralEfficiency= round(TargetCodeRate.*Qm*1e4)/1e4;
Modulation = [repmat({'QPSK'},10,1);repmat({'16QAM'},7,1);repmat({'64QAM'},12,1);{'QPSK'};{'16QAM'};{'64QAM'}];
mcstable51311 = table(MCSIndex,Modulation,Qm,TargetCodeRate,SpectralEfficiency,'VariableNames',{'MCSIndex', 'Modulation', 'Qm', 'TargetCodeRate', 'SpectralEfficiency'});

Qm = [repmat(2,5,1);repmat(4,6,1);repmat(6,9,1);repmat(8,8,1);2;4;6;8];
TargetCodeRate=[120;193;308;449;602;378;434;490;553;616;658;466;517;567;616;666;719;772;822;873;682.5;711;754;797;841;885;916.5;948;NaN;NaN;NaN;NaN]/1024;
SpectralEfficiency= round(TargetCodeRate.*Qm*1e4)/1e4;
Modulation = [repmat({'QPSK'},5,1);repmat({'16QAM'},6,1);repmat({'64QAM'},9,1);repmat({'256QAM'},8,1);{'QPSK'};{'16QAM'};{'64QAM'};{'256QAM'}];
mcstable51312 = table(MCSIndex,Modulation,Qm,TargetCodeRate,SpectralEfficiency,'VariableNames',{'MCSIndex', 'Modulation', 'Qm', 'TargetCodeRate', 'SpectralEfficiency'});

Qm = [repmat(2,15,1);repmat(4,6,1);repmat(6,8,1);2;4;6];
TargetCodeRate=[30;40;50;64;78;99;120;157;193;251;308;379;449;526;602;340;378;434;490;553;616;438;466;517;567;616;666;719;772;NaN;NaN;NaN]/1024;
SpectralEfficiency= round(TargetCodeRate.*Qm*1e4)/1e4;
Modulation = [repmat({'QPSK'},15,1);repmat({'16QAM'},6,1);repmat({'64QAM'},8,1);{'QPSK'};{'16QAM'};{'64QAM'}];
mcstable51313 = table(MCSIndex,Modulation,Qm,TargetCodeRate,SpectralEfficiency,'VariableNames',{'MCSIndex', 'Modulation', 'Qm', 'TargetCodeRate', 'SpectralEfficiency'});


Qm = [repmat(2,3,1);repmat(4,3,1);repmat(6,9,1);repmat(8,8,1);repmat(10,4,1);2;4;6;8;10];
TargetCodeRate=[120;193;449;378;490;616;466;517;567;616;666;719;772;822;873;682.5;711;754;797;841;885;916.5;948;805.5;853;900.5;948;NaN;NaN;NaN;NaN;NaN]/1024;
SpectralEfficiency= round(TargetCodeRate.*Qm*1e4)/1e4;
Modulation = [repmat({'QPSK'},3,1);repmat({'16QAM'},3,1);repmat({'64QAM'},9,1);repmat({'256QAM'},8,1);repmat({'1024QAM'},4,1);{'QPSK'};{'16QAM'};{'64QAM'};{'256QAM'};{'1024QAM'}];
mcstable51314 = table(MCSIndex,Modulation,Qm,TargetCodeRate,SpectralEfficiency,'VariableNames',{'MCSIndex', 'Modulation', 'Qm', 'TargetCodeRate', 'SpectralEfficiency'});

switch tableIdx
    case 1 % TS 38.214 Table 5.1.3.1-1
        MCSTable = mcstable51311;
    case 2 % TS 38.214 Table 5.1.3.1-2
        MCSTable = mcstable51312;
    case 3 % TS 38.214 Table 5.1.3.1-3
        MCSTable = mcstable51313;
    case 4 % TS 38.214 Table 5.1.3.1-4
        MCSTable = mcstable51314;
    otherwise % Return default table (TS 38.214 Table 5.1.3.1-1)
        MCSTable = mcstable51311;
end

end