function info = resourcesInfo(carrier,pusch)
%resourcesInfo Resources information of physical uplink shared channel
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   INFO = resourcesInfo(carrier,pusch) returns the information related to
%   the number of resource elements available for data in each OFDM symbol
%   MULSCH, number of resource elements available for UCI transmission in
%   each OFDM symbol MUCI, the 1-based symbol locations of data, DM-RS,
%   PT-RS, the OFDM symbols and physical resource blocks allocated for
%   PUSCH in a structure INFO, when the input is physical uplink shared
%   channel configuration object PUSCH.
%
%   INFO contains the following fields:
%   MULSCH         - Number of resource elements available for data
%                    transmission in each OFDM symbol
%   MUCI           - Number of resource elements available for UCI
%                    transmission in each OFDM symbol
%   PUSCHSymbolSet - 1-based OFDM symbol locations carrying data relative
%                    to the first OFDM symbol of PUSCH allocation
%                    (excluding DM-RS)
%   DMRSSymbolSet  - 1-based OFDM symbol locations carrying DM-RS relative
%                    to the first OFDM symbol of PUSCH allocation
%   PTRSSymbolSet  - 1-based OFDM symbol locations carrying PT-RS relative
%                    to the first OFDM symbol of PUSCH allocation
%   SymbolSet      - 1-based OFDM symbol locations allocated for PUSCH
%                    relative to the first OFDM symbol of PUSCH allocation
%   PRBSet         - Physical resource blocks allocated for PUSCH
%   FrequencyHopping - Frequency hopping

%   Copyright 2020-2023 The MathWorks, Inc.

%#codegen

    if pusch.Interlacing
        freqHopping = 'neither';
    else
        freqHopping = pusch.FrequencyHopping;
    end

    % Assign the structure ftable to pass into the initializeResources
    % internal function
    ftable.ChannelName = 'PUSCH';
    ftable.MappingTypeB = strcmpi(pusch.MappingType,'B');
    ftable.DMRSSymbolSet = @nr5g.internal.pusch.lookupPUSCHDMRSSymbols;
    ftable.IntraSlotFreqHoppingFlag = strcmpi(freqHopping,'intraSlot');
    
    % Get prbset, symbolset and dmrssymbols
    if ~isempty(pusch.PRBSet) || pusch.Interlacing
        [prbset,symbolset,dmrssymbols] = nr5g.internal.pxsch.initializeResources(...
            carrier,pusch,max(pusch.PRBSet(:))+1,ftable);
    else
        prbset = zeros(1,0);
        symbolset = zeros(1,0);
        dmrssymbols = zeros(1,0);
    end

    % Number of resource blocks and subcarriers allocated for PUSCH
    nPUSCHRB = length(prbset);
    nPUSCHSC = 12 * nPUSCHRB;

    % Get the OFDM symbol locations allocated for PUSCH, excluding DM-RS
    % OFDM symbol locations
    nPUSCHsymall = length(symbolset);
    if nPUSCHsymall
        % Update dmrssymbols and symbolset to 1-based and shift the
        % starting point of PUSCH symbol allocation to 1.
        dmrssymbols = dmrssymbols+1-symbolset(1);
        symbolset = symbolset+1-symbolset(1);
        temp = zeros(1,nPUSCHsymall);
        temp(symbolset) = 1;
        temp(dmrssymbols) = 2;
        datasymbols = find(temp == 1);
    else
        datasymbols = zeros(1,0);
    end

    % Get the PT-RS OFDM symbol locations and number of subcarriers
    % allocated for PT-RS
    if pusch.EnablePTRS && nPUSCHsymall
        ptrssymbols = nr5g.internal.pxsch.ptrsSymbolIndicesCPOFDM(...
            symbolset,dmrssymbols,double(pusch.PTRS.TimeDensity));
        if pusch.TransformPrecoding
            scInd = nr5g.internal.pusch.ptrsSCIndicesDFTsOFDM(...
                double(pusch.PTRS.NumPTRSSamples),double(pusch.PTRS.NumPTRSGroups),nPUSCHSC);
            nPTRS = numel(scInd);
        else
            factor = 1;
            % In case of two PT-RS ports, determine the multiplication
            % factor and apply this value to find the number of resource
            % elements used for PT-RS
            if numel(pusch.PTRS.PTRSPortSet) == 2
                kRefTable = nr5g.internal.pxsch.ptrsSubcarrierInfo(pusch.DMRS.DMRSConfigurationType);
                colIndex = strcmpi(pusch.PTRS.REOffset,{'00','01','10','11'});
                kRERef = kRefTable(pusch.PTRS.PTRSPortSet+1,colIndex);
                if (kRERef(1) ~= kRERef(2))
                    % Both PT-RS ports comprise of different subcarrier
                    % offsets, which results in a factor of 2
                    factor = 2;
                end
            end

            % Resource block offset, kRBRef
            kPTRS = double(pusch.PTRS.FrequencyDensity);
            if mod(nPUSCHRB,kPTRS) == 0
                kRBRef = mod(double(pusch.RNTI),kPTRS);
            else
                kRBRef = mod(double(pusch.RNTI),mod(nPUSCHRB,kPTRS));
            end
            ip = 0:floor((nPUSCHRB-((1)/nPUSCHSC)-kRBRef(1))/kPTRS(1));
            nPTRS = numel(ip)*factor;

            % Update the PT-RS OFDM symbol locations, based on DM-RS symbol
            % locations in case of intra-slot frequency hopping enabled
            if strcmpi(freqHopping,'intraSlot')
                secondHopStartSym = floor(pusch.SymbolAllocation(end)/2); % 0-based
                dmrsIndex = dmrssymbols > secondHopStartSym;
                emptyHop = [isempty(dmrssymbols(~dmrsIndex)) isempty(dmrssymbols(dmrsIndex))];
                ptrsIndex = ptrssymbols > secondHopStartSym;
                if any(emptyHop)
                    % DM-RS is not present in one hop or both the hops.
                    % When DM-RS is not present in both the hops, PT-RS is
                    % empty, therefore, a check for single hop is
                    % sufficient
                    if emptyHop(1) == 1
                        % First hop doesn't contain DM-RS, place PT-RS
                        % positions that are present in second hop only
                        ptrssymbols = ptrssymbols(ptrsIndex);
                    else
                        % Second hop doesn't contain DM-RS, place PT-RS
                        % positions that are present in first hop only
                        ptrssymbols = ptrssymbols(~ptrsIndex);
                    end
                end % if any(emptyHop)
            end % if strcmpi(freqHopping,'intraSlot')
        end % if pusch.TransformPrecoding
    else
        ptrssymbols = zeros(1,0);
        nPTRS = 0;
    end

    % DM-RS subcarrier (SC) locations in a resource block
    cdmgroupsnodata = double(pusch.DMRS.NumCDMGroupsWithoutData);
    if pusch.DMRS.DMRSConfigurationType==1
        % Type 1: 6 DM-RS SC per PRB per CDM (every other SC)
        dmrssc = [0 2 4 6 8 10]';                   % RE indices in a PRB
        dshiftsnodata = 0:min(cdmgroupsnodata,2)-1; % Delta shifts for CDM groups without data
    else
        % Type 2: 4 DM-RS SC per PRB per CDM (2 groups of 2 SC)
        dmrssc = [0 1 6 7]';                            % RE indices in a PRB
        dshiftsnodata = 2*(0:min(cdmgroupsnodata,3)-1); % Delta shifts for CDM groups without data
    end
    dshifts = pusch.DMRS.DeltaShifts;

    % Non DM-RS resource elements in a DM-RS containing symbol
    fullprb = ones(12,1);        % Binary map of all the subcarriers in an RB
    dshiftsComp = [dshifts dshiftsnodata];
    dmrsre = repmat(dmrssc,1,numel(dshiftsComp)) + repmat(dshiftsComp, numel(dmrssc),1);
    fullprb(dmrsre+1) = 0;       % Clear all RE which will carry DM-RS in at least one port
    puschre = find(fullprb)-1;   % Find PUSCH (non DM-RS) RE in a DM-RS containing symbol

    % Get the number of subcarriers for UL-SCH on each OFDM symbol
    mULSCH = zeros(nPUSCHsymall,1);
    mULSCH(datasymbols) = nPUSCHSC;
    mULSCH(dmrssymbols) = nPUSCHRB*numel(puschre);
    mULSCH(ptrssymbols) = nPUSCHSC-nPTRS;

    % Get the number of subcarriers allocated for UCI in each OFDM symbol
    mUCI = mULSCH;
    mUCI(dmrssymbols) = 0;

    % Combine information
    info = struct;
    info.MULSCH = mULSCH;
    info.MUCI = mUCI;
    info.PUSCHSymbolSet = datasymbols;
    info.DMRSSymbolSet = dmrssymbols;
    info.PTRSSymbolSet = ptrssymbols;
    info.SymbolSet = symbolset;
    info.PRBSet = prbset;
    info.FrequencyHopping = freqHopping;

end
