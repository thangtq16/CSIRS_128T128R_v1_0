function [ssbSCS,scsKSSB,scsNCRBSSB,scsBWP0] = blockPattern2SCS(blockPattern,varargin)
% This is an internal, undocumented function that can change anytime. It is
% currently used to map a block pattern to its corresponding SCS value.

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen
    
    idx = find(strcmpi(blockPattern, nrWavegenSSBurstConfig.BlockPattern_Values), 1);
    pattern2SCS = [15 30 30 120 240 480 960];
    tmp = pattern2SCS(idx);
    ssbSCS = tmp(1);

    if nargin > 1
        subcarrierSpacingCommon = varargin{1};
    elseif ssbSCS < 120
        subcarrierSpacingCommon = 15;
    else
        subcarrierSpacingCommon = 120;
    end    
    
    % Select the units of k_SSB and NCRB_SSB according to TS 38.211 Section
    % 7.4.3.1. Note that Case E exists in both FR2-1 and FR2-2. In FR2-2,
    % SubcarrierSpacingCommon must be always 120 kHz for operation without
    % shared spectrum channel access. When the block pattern is Case E and
    % SubcarrierSpacingCommon not equal to 120 kHz, this assumes
    % transmission in FR2-1.
    if ssbSCS < 60 % FR1
        scsNCRBSSB = 15;
        scsKSSB = 15;
        scsBWP0 = subcarrierSpacingCommon;
    else
        scsNCRBSSB = 60;
        if ssbSCS < 480 % FR2-1
            scsKSSB = subcarrierSpacingCommon;
            scsBWP0 = subcarrierSpacingCommon;
        else % FR2-2
            scsKSSB = ssbSCS;
            scsBWP0 = ssbSCS;
        end
    end
    
    
end