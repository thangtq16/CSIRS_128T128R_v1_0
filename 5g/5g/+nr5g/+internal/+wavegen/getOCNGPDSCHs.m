function pdschs = getOCNGPDSCHs(cfgObj)
%getOCNGPDSCHs get PDSCH configurations for OCNG
%   Creates a set of PDSCH configurations which implement the data region 
%   OCNG described in TS 38.101-4 Table A.5.1.1-1.
% 
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    % Get the BWP configuration and OFDM information
    bwp = cfgObj.BandwidthParts{1}; % Only one BWP in DL FRCs
    bwpid = bwp.BandwidthPartID;
    carrierID = nr5g.internal.wavegen.getCarrierIDByBWPIndex(cfgObj.SCSCarriers,...
        cfgObj.BandwidthParts,bwpid);
    carrier = cfgObj.SCSCarriers{carrierID};
    ofdmInfo = nrOFDMInfo(carrier.NSizeGrid,carrier.SubcarrierSpacing,...
        'CyclicPrefix',bwp.CyclicPrefix);
    
    % Get the set of indices 'activepdschs' corresponding to enabled
    % PDSCHs in the target BWP
    activepdschs = [];
    for i = 1:numel(cfgObj.PDSCH)
        this = cfgObj.PDSCH{i};
        if (this.BandwidthPartID==bwpid && this.Enable)
            activepdschs = [activepdschs i]; %#ok<AGROW>
        end
    end

    % The "RMC PDSCH" is assumed to be the first enabled PDSCH in the
    % target BWP; if there are no enabled PDSCHs in the target BWP, a
    % default PDSCH is used
    if (isempty(activepdschs))
        refpdsch = nrWavegenPDSCHConfig;
    else
        refpdsch = cfgObj.PDSCH{activepdschs(1)};
    end
    
    % Create a PDSCH configuration associated with the target BWP
    pdsch = nrWavegenPDSCHConfig;
    pdsch.BandwidthPartID = bwpid;
    
    % "Power Level: Same as for RMC PDSCH in the active BWP"
    pdsch.Power = refpdsch.Power;
    
    % "Content: Uncorrelated pseudo random QPSK modulated data"
    % The NID and RNTI used are chosen to be different from those used by 
    % the reference PDSCH
    pdsch.Coding = false;
    pdsch.Modulation = 'QPSK';
    if (isempty(refpdsch.NID))
        pdsch.NID = mod(cfgObj.NCellID + 1,1008);
    else
        pdsch.NID = mod(refpdsch.NID + 1,1024);
    end
    pdsch.RNTI = mod(refpdsch.RNTI + 1,65519) + 1;
    pdsch.DataSource = 'PN9';
    
    % "Transmission scheme for multiple antennas ports transmission:
    % Spatial multiplexing using any precoding matrix with dimensions same
    % as the precoding matrix for PDSCH"
    pdsch.NumLayers = refpdsch.NumLayers;
    
    % "Resources allocated: All unused REs ... Unused available REs refer
    % to REs in PRBs not allocated for any physical channels, CORESETs,
    % synchronization signals or reference signals in channel bandwidth" -
    % the rest of this function creates a set of PDSCH configurations which
    % fill these unused REs
    
    % Configure the PDSCH allocation to occupy all OFDM symbols in the slot
    L = ofdmInfo.SymbolsPerSlot;
    pdsch.SymbolAllocation = [0 L];

    % Set the same DMRS configuration as for RMC PDSCH in the active BWP
    pdsch.DMRS = refpdsch.DMRS;

    % There is no reserved CORESET as the PDCCH is not active
    pdsch.ReservedCORESET = [];
    
    % Establish the overall slot period needed to span an integer number of
    % periods of all active PDSCHs in this BWP, and also ensure it
    % corresponds to an integer number of subframes
    nSlots = getSlotPeriod(cfgObj,ofdmInfo,activepdschs);
    pdsch.Period = nSlots; 
    
    % Create matrix indicating which PRBs in which slots and symbols need
    % filled with OCNG
    ocngPRBs = getOCNGPRBs(cfgObj,bwp,ofdmInfo,nSlots,activepdschs);
    
    % For each distinct set of OCNG PRBs in a slot, set up a PDSCH in the
    % corresponding slots and symbols
    pdschs = makePDSCHs(ocngPRBs,pdsch,ofdmInfo);
    
end

% Create 'ocngPRBs', a matrix of size NSizeBWP-by-symbols, where symbols
% refer to the total number of symbols in the slot period
% (Period*SymbolsPerSlot), indicating which PRBs in which symbols and slots
% need filled with OCNG
function ocngPRBs = getOCNGPRBs(cfgObj,bwp,ofdmInfo,nSlots,activepdschs)
                     
    % Initialize 'ocngPRBs' with all ones i.e. start by assuming that all
    % PRBs and symbols need filled with OCNG
    ocngPRBs = ones([bwp.NSizeBWP nSlots*ofdmInfo.SymbolsPerSlot]);
    
    % Calculate the number of subframes
    nSubframes = nSlots / ofdmInfo.SlotsPerSubframe;

    % For each active PDSCH in this BWP, mark its PRBs and symbols as not
    % needing OCNG by setting elements of 'ocngPRBs' to zero
    for i = 1:numel(activepdschs)
        
        % Get the PDSCH configuration
        this = cfgObj.PDSCH{activepdschs(i)};

        % Get its PRBs, slots and symbols
        PRBs = this.PRBSet(:);
        slots = nr5g.internal.wavegen.expandbyperiod(this.SlotAllocation,...
                this.Period,nSubframes,bwp.SubcarrierSpacing);
        startSym = this.SymbolAllocation(1);
        symPerSlot = this.SymbolAllocation(2);
        symbols = zeros(1,symPerSlot*length(slots));
        for j=1:length(slots)
            symbols((j-1)*symPerSlot+1:j*symPerSlot) = slots(j)*ofdmInfo.SymbolsPerSlot + ...
                startSym:slots(j)*ofdmInfo.SymbolsPerSlot+startSym+symPerSlot-1;   
        end
            
        % Create indices to those PRBs and symbols in the 'ocngPRBs' matrix
        % and mark them as not needing OCNG
        ind = sub2ind(size(ocngPRBs),repmat(PRBs+1,1,numel(symbols)),...
            repmat(symbols+1,numel(PRBs),1));
        ocngPRBs(ind) = 0;

    end
    
end

% For each distinct set of OCNG PRBs in a slot, set up a PDSCH in the
% corresponding slots and symbols
function pdschs = makePDSCHs(ocngPRBs,pdsch,ofdmInfo)    

    % Identify the slots that have the same OCNG allocations in their PRB
    % set and match those slots to a particular pattern number
    PRBsPerSlot = reshape(ocngPRBs,size(ocngPRBs,1),ofdmInfo.SymbolsPerSlot,[]);
    slotsToCheck = 1:size(PRBsPerSlot,3);
    patternPerSlot = ones(1,size(PRBsPerSlot,3));
    pattern = 1;
    while ~isempty(slotsToCheck)
        slotsWithPattern = 1;
        patternPerSlot(slotsToCheck(1)) = pattern;
        for i=2:length(slotsToCheck)
          if isequal(PRBsPerSlot(:,:,slotsToCheck(1)),PRBsPerSlot(:,:,slotsToCheck(i)))
              patternPerSlot(slotsToCheck(i)) = pattern;
              slotsWithPattern = [slotsWithPattern i]; %#ok<AGROW> 
          end
        end
        pattern = pattern + 1;
        slotsToCheck(slotsWithPattern) = [];
    end

    pdschs = {};
    for i = 1:max(patternPerSlot)
        
        % Find the slots
        slots = find(patternPerSlot==i)-1;

        % Configure the PDSCH slots
        pdsch.SlotAllocation = slots;

        % Find the PRB sets and symbol allocations
        symbolsToCheck = 1:ofdmInfo.SymbolsPerSlot;
        while ~isempty(symbolsToCheck)
            PRBs = find(PRBsPerSlot(:,symbolsToCheck(1),slots(1)+1))-1;
            symbols = symbolsToCheck(1);
            for j=2:length(symbolsToCheck)
                if isequal(PRBs,find(PRBsPerSlot(:,symbolsToCheck(j),slots(1)+1))-1)
                    symbols = [symbols symbolsToCheck(j)]; %#ok<AGROW> 
                else
                    break;
                end
            end
            symbolsToCheck(1:length(symbols)) = [];
        
            % If the PRBs are not empty
            if (~isempty(PRBs))
            
                % Configure the PDSCH PRBs
                pdsch.PRBSet = PRBs';
    
                % Configure the symbol allocation
                pdsch.SymbolAllocation = [symbols(1)-1 length(symbols)];
       
                % If there is no full symbol allocation in the slot, set a DMRS
                % configuration suitable for transmissions where the data can
                % appear anywhere in the slot
                if pdsch.SymbolAllocation(2) ~= 14
                    pdsch.MappingType = 'B';
                    pdsch.DMRS.DMRSConfigurationType = 1;
                    pdsch.DMRS.DMRSAdditionalPosition = 0;
                    pdsch.DMRS.DMRSLength = 1;
                end
                
                % Create OCNG label
                label = ['Data Region OCNG for BWP ' num2str(pdsch.BandwidthPartID)];
                pdsch.Label = createPDSCHLabel(PRBs,pdsch.SymbolAllocation,slots,label);
    
                % Add the PDSCH to the set of configurations
                pdschs = [pdschs {pdsch}]; %#ok<AGROW> 

                % If the same PDSCH configuration already exists for
                % different slots, eliminate the new PDSCH configuration
                % and add the current slots to the existing PDSCH
                for k=2:length(pdschs)
                    if (isequal(pdschs{k-1}.SymbolAllocation,pdschs{end}.SymbolAllocation) ...
                            && isequal(pdschs{k-1}.PRBSet,pdschs{end}.PRBSet))
                        pdschs{k-1}.SlotAllocation = sort([pdschs{k-1}.SlotAllocation slots]);
                        pdschs = {pdschs{1:end-1}};
                        break;
                    end
                end
                
            end
        end
        
    end
    
end

% Generate a label for the OCNG PDSCH to reflect its allocated PRBs,
% symbols and slots
function labelOCNG = createPDSCHLabel(PRBs,symbolAllocation,slots,label)
    
    % Add PRB(s) to label
    [PRBMention, PRBsString] = addVectorLabel(', PRB', PRBs);
    PRBLabel = strcat(label, PRBMention, PRBsString);

    % Add symbol(s) to label
    symbols = symbolAllocation(1)+(0:symbolAllocation(2)-1);
    [symbolMention,symbolsString] = addVectorLabel(', symbol',symbols);
    symbolAndPRBLabel = strcat(PRBLabel, symbolMention, symbolsString);

    % Add slot(s) to label
    [slotMention, slotsString] = addVectorLabel(', slot', slots);
    labelOCNG = strcat(symbolAndPRBLabel,slotMention,slotsString);

    function [mentionOut, stringOut] = addVectorLabel(mentionIn, vector)
        mentionOut = mentionIn;
        stringOut = strcat(" ",strjoin(string(vector)," "));
        if length(vector) > 1
            mentionOut = [mentionIn 's'];
            vectorDiff = diff(vector);
            if numel(unique(vectorDiff))==1
                stringOut = [' ' num2str(vector(1)) ':' repmat([num2str(vectorDiff(1)) ...
                    ':'],1,vectorDiff(1)~=1) num2str(vector(end))];
            end
        end
    end

end

% Establish the overall slot period needed to span an integer number of
% periods of all active PDSCHs in this BWP, and also ensure it
% corresponds to an integer number of subframes
function nSlots = getSlotPeriod(cfgObj,ofdmInfo,activepdschs)
    
    nSlots = 1;
    for i = 1:numel(activepdschs)
        
        this = cfgObj.PDSCH{activepdschs(i)};
        nSlots = lcm(nSlots,this.Period);
        
    end
    spsf = ofdmInfo.SlotsPerSubframe;
    nSubframes = nSlots / spsf;
    nSlots = lcm(nSlots,ceil(nSubframes) * spsf);
    
end





