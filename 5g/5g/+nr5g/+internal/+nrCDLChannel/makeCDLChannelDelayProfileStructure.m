function model = makeCDLChannelDelayProfileStructure(NormalizePathGains,DelaySpread,DelayProfile,PathDelays,AveragePathGains,AnglesAoD,AnglesAoA,AnglesZoD,AnglesZoA,HasLOSCluster,KFactorFirstCluster,KFactorScaling,KFactor,AngleScaling,AngleSpreads,XPR,ClusterDelaySpread,NumStrongestClusters,MeanAngles)
%makeCDLChannelDelayProfileStructure make CDL channel structure containing
%delay profile information
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen
            
    coder.extrinsic('wireless.internal.channelmodels.getCDLPerClusterParameters');
    
    model = struct();
    model.NormalizePathGains = NormalizePathGains;
    
    model.DelaySpread = DelaySpread;

    [pathDelays,pathGains,AoD,AoA,ZoD,ZoA] = nr5g.internal.nrCDLChannel.getDelayProfile(DelayProfile,PathDelays,AveragePathGains,AnglesAoD,AnglesAoA,AnglesZoD,AnglesZoA,HasLOSCluster,KFactorFirstCluster,KFactorScaling,KFactor,DelaySpread);
    model.AveragePathGains = pathGains;
    model.PathDelays = pathDelays;
    model.AnglesAoA = AoA;
    model.AnglesAoD = AoD;
    model.AnglesZoA = ZoA;
    model.AnglesZoD = ZoD;
    model.HasLOSCluster = nr5g.internal.nrCDLChannel.hasLOSCluster(DelayProfile,HasLOSCluster);
    
    if (strcmpi(DelayProfile,'Custom') || AngleScaling)
        model.DesiredASD = AngleSpreads(1);
        model.DesiredASA = AngleSpreads(2);
        model.DesiredZSD = AngleSpreads(3);
        model.DesiredZSA = AngleSpreads(4);
    end
    
    if (strcmpi(DelayProfile,'Custom'))
        model.XPR = XPR;
        model.ClusterDelaySpread = ClusterDelaySpread;
        model.AngleScaling = false;
        model.NumStrongestClusters = NumStrongestClusters;
        model.AngleSpreads = AngleSpreads;
    else
        model.ClusterDelaySpread = NaN;
        per_cluster = coder.const(wireless.internal.channelmodels.getCDLPerClusterParameters(DelayProfile));
        model.AngleScaling = AngleScaling;
        if (AngleScaling)
            model.DesiredMeanAoD = MeanAngles(1);
            model.DesiredMeanAoA = MeanAngles(2);
            model.DesiredMeanZoD = MeanAngles(3);
            model.DesiredMeanZoA = MeanAngles(4);
        end
        model.XPR = per_cluster.XPR;
        model.AngleSpreads = [per_cluster.C_ASD per_cluster.C_ASA per_cluster.C_ZSD per_cluster.C_ZSA];
        model.NumStrongestClusters = 0;
    end
    
    model.DelayProfile = DelayProfile;
    
end
