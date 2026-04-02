function [reportConfig,csirsInd] = validateCSIInputs(carrier,csirs,reportConfig,dmrsConfig,H,nVar)
%   [REPORTCONFIG,CSIRSIND] = validateCSIInputs(CARRIER,CSIRS,REPORTCONFIG,DMRSCONFIG,H,NVAR)
%   validates the inputs arguments and returns the validated CSI report
%   configuration structure REPORTCONFIG along with the NZP-CSI-RS indices
%   CSIRSIND and noise variance NVAR.

%   Copyright 2024 The MathWorks, Inc.

    fcnName = 'nrCSIReportCSIRS';
    validateattributes(carrier,{'nrCarrierConfig'},{'scalar'},fcnName,'CARRIER');
    validateattributes(csirs,{'nrCSIRSConfig'},{'scalar'},fcnName,'CSIRS');
    validateattributes(reportConfig,{'nrCSIReportConfig'},{'scalar'},fcnName,'REPORTCONFIG');
    reportConfig = validateConfig(reportConfig);
    validateattributes(dmrsConfig,{'nrPDSCHDMRSConfig'},{'scalar'},fcnName,'DMRSCONFIG');

    % Check whether the number of CSI-RS ports is same for all CSI-RS
    % resources or not
    coder.internal.errorIf(~isscalar(unique(csirs.NumCSIRSPorts)),...
        'nr5g:nrCSIReportCSIRS:InvalidCSIRSPorts');
    NumCSIRSPorts = csirs.NumCSIRSPorts(1); % Number of ports from CSI-RS configuration object

    % Check whether the CDM lengths is same for all CSI-RS resources or not
    if iscell(csirs.CDMType)
        cdmType = csirs.CDMType;
    else
        cdmType = {csirs.CDMType};
    end
    coder.internal.errorIf(~all(strcmpi(cdmType,cdmType{1})),...
        'nr5g:nrCSIReportCSIRS:InvalidCSIRSCDMTypes')

    % Validate BWP allocation within the carrier
    reportConfig = validateBWPAllocation(carrier,reportConfig);

    % Set the flags for the respective codebook types to use the parameters
    % accordingly
    isType1SinglePanel = strcmpi(reportConfig.CodebookType,"Type1SinglePanel");
    isType1MultiPanel = strcmpi(reportConfig.CodebookType,"Type1MultiPanel");
    isType2 = strcmpi(reportConfig.CodebookType,"Type2");
    isEnhType2 = strcmpi(reportConfig.CodebookType,"eType2");
    % ThangTQ23_128T128R_eTypeII_Rel19: Refined eTypeII (TS 38.214 §5.2.2.2.5a)
    isRefinedEnhType2 = strcmpi(reportConfig.CodebookType,"eTypeII-r19");

    % Validate 'PanelDimensions'
    Ng = reportConfig.PanelDimensions(1);
    N1 = reportConfig.PanelDimensions(2);
    N2 = reportConfig.PanelDimensions(3);    
    if isType2
        csirsPortsSupported = [4 8 12 16 24 32];
    elseif isEnhType2
        csirsPortsSupported = [4 8 12 16 24 32];
    % ThangTQ23_128T128R_eTypeII_Rel19: §5.2.2.2.5a supports 48/64/128 ports
    elseif isRefinedEnhType2
        csirsPortsSupported = [48 64 128];
    elseif isType1SinglePanel
        csirsPortsSupported = [2 4 8 12 16 24 32];
    else
        csirsPortsSupported = [8 16 32];
    end
    if ~isType1MultiPanel
        coder.internal.errorIf(~any(NumCSIRSPorts == csirsPortsSupported),'nr5g:nrCSIReportCSIRS:InvalidNumCSIRSPortsForSPAndXType2',reportConfig.CodebookType,csirsPortsSupported(1))
    else
        coder.internal.errorIf(~any(NumCSIRSPorts == [8 16 32]),'nr5g:nrCSIReportCSIRS:InvalidNumCSIRSPortsForMP');
    end
    Pcsirs = 2*prod(reportConfig.PanelDimensions);
    if ~isType1MultiPanel % "Type1SinglePanel", "Type2", "eType2", "eTypeII-r19"
        OverSamplingFactors = [1 1];
        if NumCSIRSPorts > 2
            coder.internal.errorIf(Ng ~= 1,'nr5g:nrCSIReportCSIRS:InvalidNumberOfPanels')            
            coder.internal.errorIf(Pcsirs ~= NumCSIRSPorts,'nr5g:nrCSIReportCSIRS:InvalidSinglePanelDimensions',NumCSIRSPorts,N1,N2,Pcsirs,NumCSIRSPorts);
            panelConfigs = reportConfig.Tables.SinglePanelConfigurations;
            configIdx = panelConfigs{:,2} == reportConfig.PanelDimensions(2:3);
            coder.internal.errorIf(~any(all(configIdx,2)),'nr5g:nrCSIReportCSIRS:InvalidSinglePanelConfiguration',N1,N2);
            % Extract the oversampling factors
            OverSamplingFactors = panelConfigs{all(configIdx,2),3};           
        end
    else        
        coder.internal.errorIf(Pcsirs ~= NumCSIRSPorts,'nr5g:nrCSIReportCSIRS:InvalidMultiPanelDimensions',NumCSIRSPorts,Ng,N1,N2,Pcsirs,NumCSIRSPorts);        
        panelConfigs = reportConfig.Tables.MultiPanelConfigurations;
        configIdx = panelConfigs{:,2} == reportConfig.PanelDimensions;
        coder.internal.errorIf(~any(all(configIdx,2)),'nr5g:nrCSIReportCSIRS:InvalidMultiePanelConfiguration',Ng,N1,N2);
        coder.internal.errorIf(reportConfig.CodebookMode == 2 && Ng ~= 2,'nr5g:nrCSIReportCSIRS:InvalidNumPanelsforGivenCodebookMode',Ng);

        % Extract the oversampling factors
        OverSamplingFactors = panelConfigs{all(configIdx,2),3};
        reportConfig.I2Restriction = ones(1,16);
    end

    % Validate 'SubbandSize'
    if strcmp(reportConfig.PMIFormatIndicator,'Subband') && isempty(reportConfig.PRGBundleSize)
        if isempty(reportConfig.NSizeBWP)
            nSizeBWP = carrier.NSizeGrid;
        else
            nSizeBWP = double(reportConfig.NSizeBWP);
        end
        if nSizeBWP >= 24
            NSBPRB = reportConfig.SubbandSize;

            % Validate the subband size, based on the size of BWP
            % BWP size ranges
            nSizeBWPRange = [24  72;
                            73  144;
                            145 275];
            % Possible values of subband size
            nSBPRBValues = [4  8;
                            8  16;
                            16 32];
            bwpRangeCheck = (nSizeBWP >= nSizeBWPRange(:,1)) & (nSizeBWP <= nSizeBWPRange(:,2));
            validNSBPRBValues = nSBPRBValues(bwpRangeCheck,:);
            coder.internal.errorIf(~any(NSBPRB == validNSBPRBValues),...
                'nr5g:nrCSIReportCSIRS:InvalidSubbandSize',nSizeBWP,NSBPRB,validNSBPRBValues(1),validNSBPRBValues(2));
        end
    end

    % Validate 'CodebookSubsetRestriction'
    temp = reportConfig.CodebookSubsetRestriction;
    codebookSubsetRestrictionLen = numel(temp);    
    if  isType1SinglePanel || isType1MultiPanel
        if NumCSIRSPorts > 2
            O1 = OverSamplingFactors(1);
            O2 = OverSamplingFactors(2);
            codebookSubsetRestrictionRefLen = N1*O1*N2*O2;            
        elseif NumCSIRSPorts == 2
            codebookSubsetRestrictionRefLen = 6;            
        end
        
    elseif isRefinedEnhType2
        % ThangTQ23_128T128R_eTypeII_Rel19: CBSR-r19 uses block bitmap (X1^c, X2^c).
        % Defer length validation until TypeIICBSR_r19 property is added.
        % For now, skip check (CodebookSubsetRestriction should be empty).
        codebookSubsetRestrictionRefLen = 0;
    else % Type II and enhanced type II codebooks
        if N2 == 1
            codebookSubsetRestrictionRefLen = 8*N1*N2;
        else
            codebookSubsetRestrictionRefLen = 11 + 8*N1*N2;
        end
    end
    if codebookSubsetRestrictionLen
        coder.internal.errorIf(codebookSubsetRestrictionLen ~= codebookSubsetRestrictionRefLen,...
            "nr5g:nrCSIReportCSIRS:InvalidCodebookSubsetRestrictionLen",...
            reportConfig.CodebookType,codebookSubsetRestrictionLen,codebookSubsetRestrictionRefLen);
    end

    if isType2
        % Validate 'NumberOfBeams'
        coder.internal.errorIf(NumCSIRSPorts == 4 && reportConfig.NumberOfBeams > 2,...
            'nr5g:nrCSIReportCSIRS:InvalidNumberOfBeamsFor4Ports',num2str(reportConfig.NumberOfBeams));
    elseif isEnhType2
        % Validate 'ParameterCombination'
        coder.internal.errorIf(NumCSIRSPorts == 4 && reportConfig.ParameterCombination >= 3,...
            'nr5g:nrCSIReportCSIRS:InvalidParameterCombination4Ports',num2str(reportConfig.ParameterCombination));
        coder.internal.errorIf(NumCSIRSPorts < 32 && reportConfig.ParameterCombination >= 7,...
            'nr5g:nrCSIReportCSIRS:InvalidParameterCombination32Ports',num2str(reportConfig.ParameterCombination));

        % Validate 'NumberOfPMISubbandsPerCQISubband'
        coder.internal.errorIf(reportConfig.ParameterCombination >= 7 && reportConfig.NumberOfPMISubbandsPerCQISubband == 2,...
            'nr5g:nrCSIReportCSIRS:InvalidPCAndNumberOfPMISubbandsPerCQISubbandComb',num2str(reportConfig.ParameterCombination));
    % ThangTQ23_128T128R_eTypeII_Rel19: §5.2.2.2.5a constraints
    elseif isRefinedEnhType2
        % paramCombination 7/8 (L=6) only valid when P_CSIRS>=32, already
        % guaranteed since csirsPortsSupported=[48 64 128].
        % R=2 not allowed when paramCombination in {7,8} per §5.2.2.2.5a
        coder.internal.errorIf(reportConfig.ParameterCombination >= 7 && reportConfig.NumberOfPMISubbandsPerCQISubband == 2,...
            'nr5g:nrCSIReportCSIRS:InvalidPCAndNumberOfPMISubbandsPerCQISubbandComb',num2str(reportConfig.ParameterCombination));
    end

    % Validate 'H'
    validateattributes(H,{'double','single'},{},fcnName,'H');
    validateattributes(numel(size(H)),{'double'},{'>=',2,'<=',4},fcnName,'number of dimensions of H');

    csirsInd = nr5g.internal.getCSIRSIndicesForCSI(carrier,csirs);
    if ~isempty(csirsInd)
        K = carrier.NSizeGrid*12;
        L = carrier.SymbolsPerSlot;
        validateattributes(H,{class(H)},{'size',[K L NaN NumCSIRSPorts]},fcnName,'H');        
    end

    % Validate 'nVar'
    validateattributes(nVar,{'double','single'},{'scalar','real','nonnegative','finite'},fcnName,'NVAR');
end

function reportConfig = validateBWPAllocation(carrier,reportConfig)
    % Validate 'NSizeBWP'
    nStartGrid = double(carrier.NStartGrid);
    nSizeGrid = double(carrier.NSizeGrid);
    if isempty(reportConfig.NSizeBWP)
        nSizeBWP = nSizeGrid;
    else
        nSizeBWP = double(reportConfig.NSizeBWP);
    end
    % Validate 'NStartBWP'
    if isempty(reportConfig.NStartBWP)
        nStartBWP = nStartGrid;
    else
        nStartBWP = double(reportConfig.NStartBWP);
    end
    % BWP start must be greater than or equal to starting resource block of
    % carrier
    coder.internal.errorIf(nStartBWP < nStartGrid,...
        'nr5g:nrCSIReportCSIRS:InvalidNStartBWP',nStartBWP,nStartGrid);

    % BWP must lie within the limits of carrier
    coder.internal.errorIf((nSizeBWP + nStartBWP)>(nStartGrid + nSizeGrid),...
        'nr5g:nrCSIReportCSIRS:InvalidBWPLimits',nStartBWP,nSizeBWP,nStartGrid,nSizeGrid);
end

