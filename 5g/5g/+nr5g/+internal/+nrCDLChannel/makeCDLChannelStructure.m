function model = makeCDLChannelStructure(NormalizePathGains,NormalizeChannelOutputs,MaximumDopplerShift,UTDirectionOfTravel,CarrierFrequency,Seed,DelaySpread,SampleDensity,SampleRate,DelayProfile,PathDelays,AveragePathGains,AnglesAoD,AnglesAoA,AnglesZoD,AnglesZoA,HasLOSCluster,KFactorFirstCluster,KFactorScaling,KFactor,AngleScaling,AngleSpreads,XPR,ClusterDelaySpread,NumStrongestClusters,MeanAngles,channelFilterDelay,TransmitAntennaArray,ReceiveAntennaArray,TransmitArrayOrientation,ReceiveArrayOrientation,RayCoupling,InitialPhases,MovingScattererProportion,MaximumScattererSpeed)
%makeCDLChannelStructure make CDL channel structure
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%

%   Copyright 2017-2021 The MathWorks, Inc.

%#codegen
    
    % Create antenna arrays internal model
    modelArrays = nr5g.internal.nrCDLChannel.makeCDLChannelAntennaArrayStructure(TransmitAntennaArray,ReceiveAntennaArray,TransmitArrayOrientation,ReceiveArrayOrientation,CarrierFrequency);
    
    % Create cluster delay profile internal model
    modelProfile = nr5g.internal.nrCDLChannel.makeCDLChannelDelayProfileStructure(NormalizePathGains,DelaySpread,DelayProfile,PathDelays,AveragePathGains,AnglesAoD,AnglesAoA,AnglesZoD,AnglesZoA,HasLOSCluster,KFactorFirstCluster,KFactorScaling,KFactor,AngleScaling,AngleSpreads,XPR,ClusterDelaySpread,NumStrongestClusters,MeanAngles);
    
    % Additional model information
    model = struct();
    model.NormalizeChannelOutputs = NormalizeChannelOutputs;

    model.CarrierFrequency = CarrierFrequency;
    model.Seed = Seed;
    model.SampleDensity = SampleDensity;
    model.SampleRate = SampleRate;

    if isscalar(MaximumDopplerShift) && isvector(UTDirectionOfTravel) % if single mobility

        model.MaximumDopplerShift = [MaximumDopplerShift 0];
        model.UTDirectionOfTravel = [UTDirectionOfTravel UTDirectionOfTravel];
        model.MovingScattererProportion = 0;
        model.MaximumScattererSpeed = 0;

    else

        if isscalar(MaximumDopplerShift)
            model.MaximumDopplerShift = [MaximumDopplerShift MaximumDopplerShift];
        else
            model.MaximumDopplerShift = MaximumDopplerShift;
        end

        if isvector(UTDirectionOfTravel)
            model.UTDirectionOfTravel = [UTDirectionOfTravel,UTDirectionOfTravel];
        else
            model.UTDirectionOfTravel = UTDirectionOfTravel;
        end

        model.MovingScattererProportion = MovingScattererProportion;
        model.MaximumScattererSpeed = MaximumScattererSpeed;

    end

    if strcmpi(InitialPhases,'Random')
        model.InitPhase = '38.901';
    else
        model.InitPhase = InitialPhases*pi/180; % Convert to rad for internal processing
    end
    model.RayCoupling = RayCoupling;
    
    model.ChannelFilterDelay = channelFilterDelay;
    
    % Copy models of the arrays and delay profiles into the model at last
    model = mergeStructs(model,modelArrays);
    model = mergeStructs(model,modelProfile);
    
end

function s1 = mergeStructs(s1,s2)

    f = fieldnames(s2);
    for i=1:length(f)
        s1.(f{i}) = s2.(f{i});
    end
    
end