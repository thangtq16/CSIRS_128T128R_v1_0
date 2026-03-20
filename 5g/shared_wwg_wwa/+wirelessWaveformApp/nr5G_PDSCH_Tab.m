classdef nr5G_PDSCH_Tab < wirelessWaveformApp.nr5G_PXSCH_Tab
    % Dialog class that handles every graphical aspect related to PDSCH

    % Copyright 2024-2025 The MathWorks, Inc.

    properties
        % Object-specific properties
        pxschTable % PDSCH table object
    end

    properties (Constant, Access = private)
        % Store default config objects for quicker loading
        defaultPDSCHReservedConfig = nrPDSCHReservedConfig;
    end

    properties (Constant)
        PXSCHfigureName    = 'PDSCH'; % Channel and figure name
        pxschExtraFigTag   = 'pdschSingleChannelFig'; % Side panel figure tag
    end

    properties (SetAccess = protected, GetAccess = public)
        % Side panel class names
        classNamePXSCHAdv  = 'wirelessWaveformApp.nr5G_PDSCHAdvanced_Dialog';
        classNamePXSCHDMRS = 'wirelessWaveformApp.nr5G_PDSCHDMRS_Dialog';
        classNamePXSCHPTRS = 'wirelessWaveformApp.nr5G_PDSCHPTRS_Dialog';
    end

    % Constructor and public methods
    methods (Access = public)
        function obj = nr5G_PDSCH_Tab(defaultWaveConfig, invisibleEntries)
            obj@wirelessWaveformApp.nr5G_PXSCH_Tab({'TBScaling'}); % call base constructor

            % Construct the table object
            defaultConfigPXSCH = defaultWaveConfig.PDSCH;
            obj.pxschTable = wirelessWaveformApp.nr5G_PDSCH_Table(obj.pxschGridLayout, defaultConfigPXSCH, invisibleEntries);

            % Initialize the cached configuration object
            obj.pxschWaveConfig = defaultConfigPXSCH;
            obj.DefaultConfigPXSCH = defaultConfigPXSCH;
        end
    end

    methods (Access = protected)
        %% Side panel
        function dlg = mapCache2PXSCHAdv(~, pxsch, dlg)
            % Set PDSCH-specific advanced properties
            dlg.VRBToPRBInterleaving  = pxsch.VRBToPRBInterleaving;
            dlg.VRBBundleSize         = pxsch.VRBBundleSize;
            dlg.ReservedCORESET       = pxsch.ReservedCORESET;
            if ~isempty(pxsch.ReservedPRB)
                dlg.ReservedPRB       = pxsch.ReservedPRB{1}.PRBSet;
                dlg.ReservedSymbols   = pxsch.ReservedPRB{1}.SymbolSet;
                dlg.ReservedPeriod    = pxsch.ReservedPRB{1}.Period;
            end
            dlg.TBScaling             = pxsch.TBScaling;
        end

        function dlg = mapCache2PXSCHDMRS(~, pxsch, dlg)
            % Set PDSCH-specific DM-RS properties
            dlg.DMRSReferencePoint     = pxsch.DMRS.DMRSReferencePoint;
            dlg.DMRSDownlinkR16        = pxsch.DMRS.DMRSDownlinkR16;
        end

        function dlg = mapCache2PXSCHPTRS(~, ~, dlg)
            % No PDSCH-specific PT-RS properties
        end

        function pxsch = mapPXSCHAdv2Cache(obj, pxsch, dlg)
            % Set PDSCH-specific advanced properties
            pxsch.VRBToPRBInterleaving      = dlg.VRBToPRBInterleaving;
            pxsch.VRBBundleSize             = dlg.VRBBundleSize;
            pxsch.ReservedCORESET           = dlg.ReservedCORESET;
            pxsch.ReservedPRB{1}            = obj.defaultPDSCHReservedConfig;
            pxsch.ReservedPRB{1}.PRBSet     = dlg.ReservedPRB;
            pxsch.ReservedPRB{1}.SymbolSet  = dlg.ReservedSymbols;
            pxsch.ReservedPRB{1}.Period     = dlg.ReservedPeriod;
            pxsch.TBScaling                 = dlg.TBScaling;
        end

        function pxsch = mapPXSCHDMRS2Cache(~, pxsch, dlg)
            % Set PDSCH-specific DM-RS properties
            pxsch.DMRS.DMRSConfigurationType  = dlg.DMRSConfigurationType;
            pxsch.DMRS.NumCDMGroupsWithoutData= dlg.NumCDMGroupsWithoutData;
            pxsch.DMRS.DMRSReferencePoint     = dlg.DMRSReferencePoint;
            pxsch.DMRS.DMRSDownlinkR16        = dlg.DMRSDownlinkR16;
        end

        function pxsch = mapPXSCHPTRS2Cache(~, pxsch, ~)
            % No PDSCH-specific PT-RS properties
        end
    end
end