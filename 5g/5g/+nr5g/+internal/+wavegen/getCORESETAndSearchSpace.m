function [cset, ss] = getCORESETAndSearchSpace(coresets, searchSpaces, pdcch)
%getCORESETAndSearchSpace Get CORESET and Search Space
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [CSET,SS] = getCORESETAndSearchSpace(CORESETS,SEARCHSPACES,PDCCH) provides
%   the nrCORESETConfig object CSET and the nrSearchSpacesConfig object SS
%   linked to the given nrWavegenPDCCHConfig object PDCCH.

%   Copyright 2021-2022 The MathWorks, Inc.

%#codegen

    % Assumes a proper link exists (PDCCH -> SS -> CORESET).
    % validateConfig(nrDLCarrierConfig) has ensured this.

    % Init needed for codegen
    % CORESET
    propsCSET = {'CORESETID', 'Label', 'FrequencyResources', 'Duration', 'CCEREGMapping', 'REGBundleSize', 'InterleaverSize', 'ShiftIndex','RBOffset','PrecoderGranularity'};
    csetinit = nrCORESETConfig;
    for idx = 1:length(propsCSET)
        csettmp.(propsCSET{idx}) = csetinit.(propsCSET{idx});
    end
    coder.varsize('csettmp.Label',[1 inf],[0 1]);
    coder.varsize('csettmp.FrequencyResources',[1 45],[0 1]);
    coder.varsize('csettmp.CCEREGMapping',[1 14],[0 1]);
    coder.varsize('csettmp.PrecoderGranularity',[1 16],[0 1]);
    coder.varsize('csettmp.RBOffset',[1 1],[1 1]);

    % SearchSpace
    propsSS = {'SearchSpaceID', 'Label', 'CORESETID', 'SearchSpaceType', 'StartSymbolWithinSlot', 'SlotPeriodAndOffset', 'Duration', 'NumCandidates'};
    ssinit = nrSearchSpaceConfig;
    for idx = 1:length(propsSS)
        sstmp.(propsSS{idx}) = ssinit.(propsSS{idx});
    end
    coder.varsize('sstmp.Label',[1 inf],[0 1]);
    coder.varsize('sstmp.SearchSpaceType',[1 6],[0 1]);

    % Find this PDCCH's Search Space
    for ssIdx = 1:numel(searchSpaces)
        if pdcch.SearchSpaceID == searchSpaces{ssIdx}.SearchSpaceID
            for idx = 1:length(propsSS)
                sstmp.(propsSS{idx}) = searchSpaces{ssIdx}.(propsSS{idx});
            end
            for csIdx = 1:numel(coresets)
                % Then find this SearchSpace's CORESET
                if coresets{csIdx}.CORESETID == sstmp.CORESETID
                    for idx = 1:length(propsCSET)
                        csettmp.(propsCSET{idx}) = coresets{csIdx}.(propsCSET{idx});
                    end
                end
            end
        end
    end
    
    % Create CORESET and SearchSpace configuration objects
    ss = nrSearchSpaceConfig('SearchSpaceType',sstmp.SearchSpaceType);
    for idx = 1:length(propsSS)
        ss.(propsSS{idx}) = sstmp.(propsSS{idx});
    end
    cset = nrCORESETConfig('FrequencyResources',csettmp.FrequencyResources);
    for idx = 1:length(propsCSET)
        cset.(propsCSET{idx}) = csettmp.(propsCSET{idx});
    end
end