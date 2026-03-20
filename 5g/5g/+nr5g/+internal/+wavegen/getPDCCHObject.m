function [pdcch, ss] = getPDCCHObject(wavePDCCH,bwp,coreset,searchSpaces,RNTI)
%getPDCCHObject Creates nrPDCCHConfig object
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   PDCCH = getPDCCHObject(WAVEPDCCH,BWP,CORESET,SEARCHSPACES,RNTI) provides
%   the PDCCH configuration object nrPDCCHConfig, given the input
%   nrWavegenPDCCHConfig object WAVEPDCCH, nrBandwidthPartConfig object
%   BWP, nrCORESETConfig object CORESET, nrSearchSpaces object
%   SEARCHSPACES, and RNTI value.

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    % Get a copy of the current CORESET for this PDCCH sequence
    [cset, ss] = nr5g.internal.wavegen.getCORESETAndSearchSpace(coreset, searchSpaces, wavePDCCH);
    
    pdcch = nrPDCCHConfig('CORESET',cset,'SearchSpace',ss); % CORESET and SearchSpace need to be defined at construction time for codegen
    pdcch.NStartBWP = bwp.NStartBWP;
    pdcch.NSizeBWP = bwp.NSizeBWP;
    pdcch.RNTI = RNTI;
    pdcch.DMRSScramblingID = wavePDCCH.DMRSScramblingID;
    pdcch.AggregationLevel = wavePDCCH.AggregationLevel;
    pdcch.AllocatedCandidate = wavePDCCH.AllocatedCandidate;
    if isprop(pdcch,'CCEOffset') && isprop(wavePDCCH,'CCEOffset')
        pdcch.CCEOffset = wavePDCCH.CCEOffset;
    end
end
