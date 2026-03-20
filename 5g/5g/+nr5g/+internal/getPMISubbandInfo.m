function info = getPMISubbandInfo(carrier,reportConfig)
% getPMISubbandInfo Downlink PMI subband information
%   INFO = getSubbandInfo(CARRIER,REPORTCONFIG) returns the PMI subband
%   information or the PRG information INFO considering the carrier
%   configuration CARRIER and CSI report configuration object
%   REPORTCONFIG.

%   Copyright 2024 The MathWorks, Inc.

    nSizeBWP = reportConfig.NSizeBWP;
    if isempty(nSizeBWP)
        nSizeBWP = carrier.NSizeGrid;
    end
    nStartBWP = reportConfig.NStartBWP;
    if isempty(nStartBWP)
        nStartBWP = carrier.NStartGrid;
    end

    % If PRGBundleSize is other than empty, consider the subband size as PRG size
    if  ~isempty(reportConfig.PRGBundleSize) && strcmpi(reportConfig.CodebookType,'Type1SinglePanel')
        reportingMode = 'subband';
        NSBPRB = reportConfig.PRGBundleSize;
        ignoreBWPSize = true; % To ignore the BWP size for the validation of PRG size
    else
        reportingMode = reportConfig.PMIFormatIndicator;
        NSBPRB = reportConfig.SubbandSize;
        ignoreBWPSize = false; % To consider the BWP size for the validation of subband size
    end

    % Get the subband information
    if strcmpi(reportingMode,'wideband') || (~ignoreBWPSize && nSizeBWP < 24)
        % According to TS 38.214 Table 5.2.1.4-2, if the size of BWP is
        % less than 24 PRBs, the division of BWP into subbands is not
        % applicable. In this case, the number of subbands is considered as
        % 1 and the subband size is considered as the size of BWP
        numSubbands = 1;
        NSBPRB = nSizeBWP;
        subbandSizes = NSBPRB;
        subbandSet = ones(1,nSizeBWP);
    else
        R = 1;
        if strcmpi(reportConfig.CodebookType,'eType2')
            R = reportConfig.NumberOfPMISubbandsPerCQISubband;
        end
        crb = nStartBWP + (0:nSizeBWP-1);
        NSBPRB = NSBPRB/R;
        subbandSet = floor(crb/NSBPRB) + 1;
        subbandSizes = histcounts(floor(crb/NSBPRB),BinMethod="integers");
        numSubbands = length(subbandSizes);
    end

    % Place the number of subbands and subband sizes in the output
    % structure
    info.NumSubbands = numSubbands;
    info.SubbandSizes = subbandSizes;
    info.SubbandSet = subbandSet;
end