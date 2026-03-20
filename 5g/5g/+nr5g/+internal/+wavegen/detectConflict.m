%detectConflict Conflict detection of 5G channels and signals
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2021-2024 The MathWorks, Inc.

%#codegen

function conflictInfo = detectConflict(wgc,waveResources)
    
    conflictInfo = conflictUnitStruct();

    isDownlink = isa(wgc,'nrDLCarrierConfig');

    % Frequently used parameters
    numSubframes = wgc.NumSubframes;
    [~,nports] = nr5g.internal.wavegen.getNumPorts(wgc);
    bwps = wgc.BandwidthParts;
    bwpIDs = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(bwps,'BandwidthPartID','double');
    carriers = wgc.SCSCarriers;
    initnsf = wgc.InitialNSubframe;

    % Extract channel resources information from waveform resources and get
    % the associated channel configurations, indices and bandwidth parts
    [channelInfo,channelType,channelIndex,channelSubindex,channelBWPIDs] =...
                            getChannelInfo(wgc,waveResources);

    if isDownlink
        ssbstruct = nr5g.internal.wavegen.mapSSBObj2Struct(wgc.SSBurst,carriers);
        ssbstruct.NHalfFrame = fix(initnsf/5);
        ssbstruct.NFrame = fix(initnsf/10);
        ssbreserved = nr5g.internal.wavegen.ssburstResources(ssbstruct,carriers,bwps);
    end

    numChannels = length(channelInfo);
    count = 1; % Initialize conflict counter
    % Process all pairs of channels ch1-ch2 (except SSB) where ch1~=ch2
    for c1 = 1:numChannels

        ch1 = channelInfo{c1};
        if isempty(ch1.Resources) || isempty(ch1.Resources(1).NSlot)
            continue;
        end
        
        % Use the BWP associated with channel 1 to create an indicator grid
        % for channel 1 containing "true" in the REs where the
        % corresponding channel is present.
        b1 = find(channelBWPIDs(c1) == bwpIDs,1,'first');
        bwp1 = bwps{b1};
        p1 = nports(b1);
        g1 = indicatorGrid(bwp1,numSubframes,initnsf,p1,ch1.Resources);
        
        % Search for conflicts of channel 1 against each other channel
        for c2 = (c1+1):numChannels
            
            ch2 = channelInfo{c2};
            if isempty(ch2.Resources) || isempty(ch2.Resources(1).NSlot)
                continue;
            end

            % Create an indicator grid for channel 2
            b2 = find(channelBWPIDs(c2) == bwpIDs,1,'first');
            bwp2 = bwps{b2};
            p2 = nports(b2);
            g2 = indicatorGrid(bwp2,numSubframes,initnsf,p2,ch2.Resources);

            % Detect conflicts between channels in the same BWP
            if b1 == b2

                conflictGrid1 = g1 & g2;
                conflictGrid2 = conflictGrid1;

            else % Detect conflicts between channel in different BWPs
                
                [conflictGrid1,conflictGrid2] = detectConflictDifferentGrids(bwp1,bwp2,g1,g2);

            end

            isConflict = any(conflictGrid1,'all');
            if isConflict
                % Create conflict info structure
                conf = conflictUnitStruct();
                conf.BwpIdx = [b1(1), b2(1)];
                conf.Grid = {conflictGrid1, conflictGrid2};
                conf.ChannelType = {channelType{c1},channelType{c2}};
                conf.ChannelIdx = [channelIndex(c1),channelIndex(c2)];
                conf.ChannelSubidx = [channelSubindex(c1),channelSubindex(c2)];
                conflictInfo(count) = conf(1);
                count = count+1;
            end
        end

        % Detect conflicts between SSB reserved resources and other
        % channels or signals       
        if isDownlink && wgc.SSBurst.Enable

            % Create an indicator grid for the SSB in the BWP of the
            % channel
            ssbg = ssbIndicatorGrid(bwp1,numSubframes,initnsf,ssbreserved{b1(1)});

            conflictGrid = ssbg & g1;

            isConflict = any(conflictGrid,'all');
            if isConflict
                % Create conflict info structure
                conf = conflictUnitStruct();
                conf.BwpIdx = repmat(b1(1),1,2);
                conf.Grid = {conflictGrid,conflictGrid};
                conf.ChannelType = {channelType{c1},"SSBurst"};
                conf.ChannelIdx = [channelIndex(c1),1];
                conf.ChannelSubidx = [channelSubindex(c1),nan];
                conflictInfo(count) = conf(1);
                count = count+1;
            end

        end
    end

end

% Conflict detection between different grids
function [conflictGrid1,conflictGrid2] = detectConflictDifferentGrids(conf1,conf2,inGrid1,inGrid2)

    if isa(conf1,"nrWavegenBWPConfig")
        carrier1 = nr5g.internal.wavegen.getCarrierCfgObject(conf1,0);
        carrier2 = nr5g.internal.wavegen.getCarrierCfgObject(conf2,0);
    else
        carrier1 = conf1;
        carrier2 = conf2;
    end

    % Intialize returned conflict grids
    conflictGrid1 = false(size(inGrid1));
    conflictGrid2 = false(size(inGrid2));

    [commonGrid1,commonGrid2,fScale1,fScale2,...
        tScale1,tScale2,startIdx1,startIdx2] ...
            = makeComparableREGrids(carrier1,carrier2,inGrid1,inGrid2);
    
    conflictGrid = commonGrid1 & commonGrid2;
    isConflict = any(conflictGrid,'all');

    if isConflict

        % Subsample conflict grids and map to grids of size equal to input
        % grids. Partial overlaps are considered as conflicts.    
        sampledGrid1 = downsampleGrid(conflictGrid,fScale1,tScale1);
        k1 = floor((startIdx1-1)/fScale1) + (1:size(sampledGrid1,1));
        conflictGrid1(k1,:) = sampledGrid1;

        sampledGrid2 = downsampleGrid(conflictGrid,fScale2,tScale2);
        k2 = floor((startIdx2-1)/fScale2) + (1:size(sampledGrid2,1));
        conflictGrid2(k2,:) = sampledGrid2;

    end

end

% Create a logical indicator grid containing "true" in the REs where a
% channel is present.
function indg = indicatorGrid(bwp,numSubframes,initnsf,numPorts,resources)

    symPerSlot = nr5g.internal.wavegen.symbolsPerSlot(bwp);
    mu = fix(bwp.SubcarrierSpacing/15);

    K = bwp.NSizeBWP*12;
    L = numSubframes*symPerSlot*mu;
    P = numPorts(1);
    g = zeros(K,L,P);

    initslot = initnsf*mu;

    for s = 1:length(resources)

        if isempty(resources(s).NSlot)
            continue;
        end

        slotg = false(K,symPerSlot,P);
        ind = resources(s).Indices;
        slotg(ind) = true;

        nslot = resources(s).NSlot;
        symIdx = (nslot-initslot)*symPerSlot+(1:symPerSlot);
        g(:,symIdx,1:P) = g(:,symIdx,1:P) + slotg;
        
    end

    % Collapse all ports. Two channels are in conflict if they use the
    % same REs at any port
    indg = any(g,3);

end

% Create a logical indicator grid for the SSB containing "true" in the REs
% where the SS burst is present.
function g = ssbIndicatorGrid(bwp,numSubframes,initnsf,ssbreserved)

    symPerSlot = nr5g.internal.wavegen.symbolsPerSlot(bwp);
    mu = fix(bwp.SubcarrierSpacing/15);
    initslot = initnsf*mu;

    K = bwp.NSizeBWP*12;
    L = numSubframes*symPerSlot*mu;
    g = false(K,L);
    
    r = ssbreserved;
    prb = r.PRBSet;
    sym = r.SymbolSet;
    period = r.Period*symPerSlot*mu; % Period in symbols
    
    l = nr5g.internal.wavegen.expandbyperiod(sym,period,L,15,initslot*symPerSlot);
    k = reshape(prb*12 + (0:11)',[],1);

    g(k+1,l+1-initslot*symPerSlot) = true;

end

% Decimate the input resource grid in frequency and time
function sampledGrid = downsampleGrid(inGrid,fScale,tScale)

    % Spread detected conflicts across a time-frequency subset
    % of common grid resources and resample to match input grid
    % sizes. Partial overlaps are considered as conflicts.
    if fScale ~= 1 || tScale ~= 1
        spreadGrid = conv2(inGrid,ones(fScale,tScale),'Valid') ~= 0;
        sampledGrid = spreadGrid(1:fScale:end,1:tScale:end);
    else
        sampledGrid = inGrid ~= 0;
    end

end

% Extract channel resources information from waveform resources
function [chInfo,chName,chIndex,chSubidx,bwpID] = getChannelInfo(wgc,waveRes)

    chInfo = {};
    chName = {};
    chIndex = [];
    chSubidx = [];
    bwpID = [];

    % For each channel/signal type in waveform resources, obtain the
    % associated channel index, subindex (CSI-RS), configuration
    % object and BWP ID.
    chTypes = fieldnames(waveRes); % Channel/signal types
    numChTypes = numel(chTypes);
    for f = 1:numChTypes 

        % Flatten waveform resources
        chType = chTypes{f};
        channelConfig = wgc.(chType);

        if isempty(channelConfig)
            continue;
        end

        channelsResources = waveRes.(chType);
        for c = 1:numel(channelsResources)
            chInfo{end+1} = getChannelResourcesInfo(channelsResources(c));
            chName{end+1} = string(chType);
        end

        [idx,sidx,bwpid] = getChannelIndexInfo(channelConfig);

        chIndex = [chIndex idx]; %#ok<AGROW> 
        chSubidx = [chSubidx sidx]; %#ok<AGROW> 
        bwpID = [bwpID bwpid]; %#ok<AGROW> 

    end

end

% Create a structure array containing channel resources information 
function channelOut = getChannelResourcesInfo(channelIn)
    
    s = struct('Indices',uint32(zeros(0,1)),'NSlot',zeros(0,1));
    coder.varsize('s.Indices','s.NSlot',[inf inf],[1 1]);

    numRes = numel(channelIn.Resources);
    res = repmat(s,1,numRes);
    
    for r = 1:numRes
        res(r).Indices = getResourceIndices(channelIn.Resources(r));
        res(r).NSlot = channelIn.Resources(r).NSlot;
    end

    channelOut.Name = channelIn.Name;
    channelOut.Resources = res;    

end

% Return all channel/signal indices of channel resource including channel
% or signal indices, DM-RS and PT-RS if present.
function Indices = getResourceIndices(resource)

    if isfield(resource,'SignalIndices')
        indField = 'SignalIndices';
    else
        indField = 'ChannelIndices';
    end

    % Channel/signal indices
    Indices = resource.(indField)(:);

    if isfield(resource,'DMRSIndices')
        Indices = [Indices; resource.DMRSIndices(:)];
    end

    if isfield(resource,'PTRSIndices')
        Indices = [Indices; resource.PTRSIndices(:)];
    end    

end

% Return information about the mapping between the waveform resources and
% the associated configuration object
function [chIdx,chSubIdx,bwpID] = getChannelIndexInfo(cfgIn)

    if ~isempty(cfgIn) && isa(cfgIn{1},'nrWavegenCSIRSConfig')

        chIdx = [];
        chSubIdx = [];
        bwpID = [];
        for c = 1:numel(cfgIn)
            ch = cfgIn{c};
            if iscell(ch.CSIRSType)
                numRes = length(ch.CSIRSType);
                subidx = 1:numRes;
            else
                numRes = 1;
                subidx = nan;
            end
            chIdx = [chIdx c*ones(1,numRes)]; %#ok<AGROW>
            chSubIdx = [chSubIdx subidx]; %#ok<AGROW>
            id = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects({cfgIn{c}}, 'BandwidthPartID', 'double');
            bwpID = [bwpID id*ones(1,numRes)]; %#ok<AGROW> 
        end

    else % Channels/signals other than CSI-RS

        numCh = numel(cfgIn);
        chIdx = 1:numCh;
        chSubIdx = nan(1,numCh);
        bwpID = nr5g.internal.wavegen.getSinglePropValuesFromCellWithObjects(cfgIn, 'BandwidthPartID', 'double');

    end

end

% Create a default structure to store conflict information
function unitStruct = conflictUnitStruct(~)

    unitStruct = struct();
    unitStruct.BwpIdx = NaN(1,2);
    unitStruct.Grid = {false(0,1),false(0,1)};
    unitStruct.ChannelType = {"",""}; %#ok<CLARRSTR> 
    unitStruct.ChannelIdx = NaN(1,2);
    unitStruct.ChannelSubidx = NaN(1,2);

    coder.varsize('unitStruct.Grid{:}',[Inf Inf],[1 1]);

end

% Create element-wise comparable carrier RE resource grids
function [interGrid1,interGrid2,fScale1,fScale2,tScale1,tScale2,startIdx1,startIdx2,commonStart] = makeComparableREGrids(carrier1, carrier2, grid1, grid2)

    [fScale1,fScale2,tScale1,tScale2] = scalingFactors(carrier1,carrier2);

    % Create logical grids resampled to a common factor to make them
    % comparable by element-wise operations
    indicatorGrid1 = repelem(grid1~=0, fScale1, tScale1); 
    indicatorGrid2 = repelem(grid2~=0, fScale2, tScale2);

    % Subcarrier indices of the frequency range common to both grids
    [startIdx1,startIdx2,commonStart,len] = subcarrierIntersectionIndices(carrier1,carrier2,fScale1,fScale2);

    % Return comparable grids in the freq. range where they intersect
    interGrid1 = indicatorGrid1(startIdx1 + (0:len-1),:,:);
    interGrid2 = indicatorGrid2(startIdx2 + (0:len-1),:,:);

end

function [fScale1,fScale2,tScale1,tScale2] = scalingFactors(carrier1,carrier2)

    fScale1 = max(1, carrier1.SubcarrierSpacing/carrier2.SubcarrierSpacing);
    fScale2 = max(1, carrier2.SubcarrierSpacing/carrier1.SubcarrierSpacing);
    % Account for different cyclic prefix, as well as different number of slots/subframe:
    tScale1 = max(1, carrier2.SubcarrierSpacing/carrier1.SubcarrierSpacing);
    tScale2 = max(1, carrier1.SubcarrierSpacing/carrier2.SubcarrierSpacing);
    symPerSlot1 = carrier1.SymbolsPerSlot;
    symPerSlot2 = carrier2.SymbolsPerSlot;
    if symPerSlot1 ~= symPerSlot2 % Different CP length (normal vs extended)
        lcm12 = lcm(symPerSlot1,symPerSlot2);
        tScale1 = tScale1 * lcm12 / symPerSlot1;
        tScale2 = tScale2 * lcm12 / symPerSlot2;
    end

end

function [startIdx1,startIdx2,commonStart,len] = subcarrierIntersectionIndices(carrier1,carrier2,fScale1,fScale2)

    numREperRB = 12;

    % Bandwidth of the RE grids common (lowest) SCS
    K1 = carrier1.NSizeGrid * fScale1 * numREperRB;
    K2 = carrier2.NSizeGrid * fScale2 * numREperRB;

    % Frequency start of the two grids in common (lowest) SCS
    k1start = carrier1.NStartGrid * fScale1 * numREperRB;
    k2start = carrier2.NStartGrid * fScale2 * numREperRB;    

    % Frequency end of the two grids in common (lowest) SCS
    k1End = k1start + K1;
    k2End = k2start + K2;

    % Boundaries of the intersection
    commonStart = max(k1start,k2start);
    commonEnd = min(k1End,k2End);
    len = commonEnd - commonStart;

    % Starting index of the intersection of the two grids in common SCS
    startIdx1 = 1 + commonStart - k1start;
    startIdx2 = 1 + commonStart - k2start;
    
end