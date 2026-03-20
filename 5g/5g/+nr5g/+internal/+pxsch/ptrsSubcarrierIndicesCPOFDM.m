function [ind,info] = ptrsSubcarrierIndicesCPOFDM(prbset,dmrsType,kPTRS,reOffset,ptrsPorts,rnti,freqHopping,freqHopOffset)
%ptrsSubcarrierIndicesCPOFDM PT-RS subcarrier indices for CP-OFDM
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [IND,INFO] = ptrsSubcarrierIndicesCPOFDM(PRBSET,DMRSTYPE,KPTRS,REOFFSET,PTRSPORTS,RNTI)
%   returns the 0-based PT-RS resource element indices in a cell array IND
%   according to TS 38.211 Section 7.4.1.2.2, given the inputs, vector of
%   resource blocks PRBSET, DM-RS configuration type DMRSTYPE, frequency
%   density of PT-RS KPTRS, PT-RS resource element offset REOFFSET, PT-RS
%   antenna ports PTRSPORTS and the radio network temporary identifier
%   RNTI. It also returns the structural information INFO related to
%   subcarrier offset and resource block offset of PT-RS.
%
%   IND is a cell array of length equal to number of number of PT-RS ports.
%   Each element of cell array IND contains the matrix of PT-RS subcarrier
%   locations with each column corresponding to each hop.
%   INFO is the structural information with the following fields:
%   KRERef        - PT-RS subcarrier offset in a resource block. It is a
%                   vector of length equal to number of PT-RS ports
%   KRBRef        - Starting resource block for PT-RS in PRB
%   DMRSSCPATTERN - DM-RS subcarrier locations pattern based on
%                   configuration type
%   NDMRSSC       - Number of DM-RS subcarriers in a resource block
%                   based on configuration type
%
%   Example:
%   % Get the PT-RS subcarrier locations for a PRB set of 0 to 2, DM-RS
%   % configuration type set to 1, frequency density set to 2, resource
%   % element offset set to '00', PT-RS port set to 0 and radio network
%   % temporary identifier is set to 1.
%
%   prbset = 0:2;
%   dmrsType = 1;
%   kptrs = 2;
%   reOffset = '00';
%   ptrsPort = 0;
%   rnti = 1;
%   ind = nr5g.internal.pxsch.ptrsSubcarrierIndicesCPOFDM(prbset,dmrsType,kptrs,reOffset,ptrsPort,rnti);
%   ind{1}

%   Copyright 2019 The MathWorks, Inc.

%#codegen

    % Number of resource blocks and subcarriers
    nRBSC = 12;
    nPXSCHRB = numel(prbset);
    nPXSCHSC = nPXSCHRB*12;

    % Resource element offset, kRERef
    [kRefTable,dmrsSCPattern,nDMRSSC] = nr5g.internal.pxsch.ptrsSubcarrierInfo(dmrsType);
    colIndex = strcmpi(reOffset,{'00','01','10','11'});
    kRERef = kRefTable(ptrsPorts+1,colIndex);

    % Resource block offset, kRBRef
    if mod(nPXSCHRB,kPTRS) == 0
        kRBRef = mod(double(rnti),kPTRS);
    else
        kRBRef = mod(double(rnti),mod(nPXSCHRB,kPTRS));
    end

    % Get the PT-RS subcarrier indices for each port
    ind = coder.nullcopy(cell(1,numel(ptrsPorts)));
    for p = 1:numel(ptrsPorts)
        ip = 0:floor((nPXSCHRB-((1+kRERef(p))/nPXSCHSC)-kRBRef(1))/kPTRS(1));
        indFirstHop = reshape(kRERef(p)+(prbset(ip*kPTRS(1)+1+kRBRef(1)))*nRBSC,[],1);
        if nargin == 8 && freqHopping
            indSecondHop = zeros(0,1);
            if ~isempty(prbset) % For codegen with empty value
                prbSetSecondHop = prbset-min(prbset(:))+freqHopOffset;
                indSecondHop = reshape(kRERef(p)+(prbSetSecondHop(ip*kPTRS(1)+1+kRBRef(1)))*nRBSC,[],1);
            end
            ind{p} = [indFirstHop indSecondHop];
        else
            ind{p} = indFirstHop;
        end
    end

    % Combine some useful information to structure
    info.KRERef = kRERef;
    info.KRBRef = kRBRef;
    info.DMRSSCPattern = dmrsSCPattern;
    info.NDMRSSC = nDMRSSC;

end
