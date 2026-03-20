function validateLayerDependentParams(carrier,reportConfig,NumCSIRSPorts,H,nLayers)
%validateLayerDependentParams validate layer dependent properties of
%nrCSIReportConfig
% 
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2024 The MathWorks, Inc.

    fcnName = 'nrCSIReportCSIRS';
    % Validate 'nLayers'
    isType1SinglePanel = strcmpi(reportConfig.CodebookType,"Type1SinglePanel");
    isType2 = strcmpi(reportConfig.CodebookType,"Type2");
    isEnhType2 = strcmpi(reportConfig.CodebookType,"eType2");    
    if isType2
        maxNLayers = 2;
    elseif isEnhType2
        maxNLayers = 4;
        if any(reportConfig.ParameterCombination == [7 8])
            maxNLayers = 2;
        end
    elseif isType1SinglePanel
        maxNLayers = 8;
    else
        maxNLayers = 4;
    end
    validateattributes(nLayers,{'numeric'},{'scalar','integer','positive','<=',maxNLayers},fcnName,"NLAYERS(" + num2str(nLayers) + ") for " + reportConfig.CodebookType + " codebooks");
    if strcmpi(reportConfig.CodebookType,'eType2')
        coder.internal.errorIf(nLayers > 2 && reportConfig.ParameterCombination >= 7,...
            'nr5g:nrCSIReportCSIRS:InvalidParameterCombinationEType2',num2str(reportConfig.ParameterCombination));
    end
    
    K = carrier.NSizeGrid*12;
    L = carrier.SymbolsPerSlot;
    validateattributes(H,{class(H)},{'size',[K L NaN NumCSIRSPorts]},fcnName,'H');

    % Validate 'nLayers'
    nRxAnts = size(H,3);
    maxPossibleNLayers = min(nRxAnts,NumCSIRSPorts);
    coder.internal.errorIf(nLayers > maxPossibleNLayers,...
        'nr5g:nrCSIReportCSIRS:InvalidNumLayers',num2str(NumCSIRSPorts),num2str(nRxAnts),num2str(maxPossibleNLayers));
end