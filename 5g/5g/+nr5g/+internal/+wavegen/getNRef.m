function nRef = getNRef(ch,bwpSize)
% Calculate NRef for limited buffer size rate matching control as described
% in TS 38.212 Section 5.4.2.1
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2024 The MathWorks, Inc.

%#codegen

    % Get nPRBLBRM from Table 5.4.2.1-1 of TS 38.212
    if bwpSize<33
        nPRBLBRM = 32;
    elseif bwpSize<=66
        nPRBLBRM = 66;
    elseif bwpSize<=107
        nPRBLBRM = 107;
    elseif bwpSize<=135
        nPRBLBRM = 135;
    elseif bwpSize<=162
        nPRBLBRM = 162;
    elseif bwpSize<=217
        nPRBLBRM = 217;
    else
        nPRBLBRM = 273;
    end

    % Get maxNumLayers
    maxNumLayers = min(4,max(ch.NumLayers,ch.MaxNumLayers));

    % Get modulation scheme corresponding to maximum modulation order
    modLBRM = nr5g.internal.wavegen.PXSCHConfigBase.getMaxQmModulation(ch.MCSTable,ch.Modulation);

    % Fixed constant parameters
    tcr = 948/1024;  % target code rate
    nREPerPRB = 156; % number of REs per PRB
    R = 2/3;         % matching rate

    % Calculate TBSLBRM and C
    tbsLBRM = nrTBS(modLBRM,maxNumLayers,nPRBLBRM,nREPerPRB,tcr);
    xlschInfo = nr5g.internal.getSCHInfo(tbsLBRM(1),tcr);
    C = xlschInfo.C;

    % Calculate NRef
    nRef = floor(tbsLBRM(1)/(C*R));

end