function CQITable = getCQITable(tableIdx)
%getCQITable Get TS 38.214 Tables 5.2.2.1-2, 5.2.2.1-3, 5.2.2.1-4
% and 5.2.2.1-5.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%    CQITable = nr5g.internal.getCQITable(tableIdx) returns CQI Tables
%    as defined in TS 38.214 Tables 5.2.2.1-x, where x is between 2-5

%    Copyright 2023 The MathWorks, Inc.

%#codegen

CQIIndex = (0:15)';
Qm = [NaN;repmat(2,6,1);repmat(4,3,1);repmat(6,6,1)];
TargetCodeRate=[NaN;78;120;193;308;449;602;378;490;616;466;567;666;772;873;948]/1024;
SpectralEfficiency= round(TargetCodeRate.*Qm*1e4)/1e4;
Modulation = [{'Out of Range'};repmat({'QPSK'},6,1);repmat({'16QAM'},3,1);repmat({'64QAM'},6,1);];
cqitable52212 = table(CQIIndex,Modulation,Qm,TargetCodeRate,SpectralEfficiency,'VariableNames',{'CQIIndex', 'Modulation', 'Qm', 'TargetCodeRate', 'SpectralEfficiency'});

Qm = [NaN;repmat(2,3,1);repmat(4,3,1);repmat(6,5,1);repmat(8,4,1)];
TargetCodeRate=[NaN;78;193;449;378;490;616;466;567;666;772;873;711;797;885;948]/1024;
SpectralEfficiency= round(TargetCodeRate.*Qm*1e4)/1e4;
Modulation =[{'Out of Range'};repmat({'QPSK'},3,1);repmat({'16QAM'},3,1);repmat({'64QAM'},5,1);repmat({'256QAM'},4,1);];
cqitable52213 = table(CQIIndex,Modulation,Qm,TargetCodeRate,SpectralEfficiency,'VariableNames',{'CQIIndex', 'Modulation', 'Qm', 'TargetCodeRate', 'SpectralEfficiency'});

Qm = [NaN;repmat(2,8,1);repmat(4,3,1);repmat(6,4,1)];
TargetCodeRate=[NaN;30;50;78;120;193;308;449;602;378;490;616;466;567;666;772]/1024;
SpectralEfficiency= round(TargetCodeRate.*Qm*1e4)/1e4;
Modulation = [{'Out of Range'};repmat({'QPSK'},8,1);repmat({'16QAM'},3,1);repmat({'64QAM'},4,1);];
cqitable52214 = table(CQIIndex,Modulation,Qm,TargetCodeRate,SpectralEfficiency,'VariableNames',{'CQIIndex', 'Modulation', 'Qm', 'TargetCodeRate', 'SpectralEfficiency'});

Qm = [NaN;repmat(2,3,1);repmat(4,2,1);repmat(6,4,1);repmat(8,4,1);repmat(10,2,1)];
TargetCodeRate=[NaN;78;193;449;378;616;567;666;772;873;711;797;885;948;853;948]/1024;
SpectralEfficiency= round(TargetCodeRate.*Qm*1e4)/1e4;
Modulation = [{'Out of Range'};repmat({'QPSK'},3,1);repmat({'16QAM'},2,1);repmat({'64QAM'},4,1);repmat({'256QAM'},4,1);repmat({'1024QAM'},2,1)];
cqitable52215 = table(CQIIndex,Modulation,Qm,TargetCodeRate,SpectralEfficiency,'VariableNames',{'CQIIndex', 'Modulation', 'Qm', 'TargetCodeRate', 'SpectralEfficiency'});

switch tableIdx
    case 1 % TS 38.214 Table 5.2.2.1-2
        CQITable = cqitable52212;
    case 2 % TS 38.214 Table 5.2.2.1-3
        CQITable = cqitable52213;
    case 3 % TS 38.214 Table 5.2.2.1-4
        CQITable = cqitable52214;
    case 4 % TS 38.214 Table 5.2.2.1-5
        CQITable = cqitable52215;
    otherwise % Return default table (TS 38.214 Table 5.2.2.1-2)
        CQITable = cqitable52212;
end

end