%prbsDMRSSequenceSets PDSCH/PUSCH OFDM PRBS DM-RS Base Sequences
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2025 The MathWorks, Inc.

%#codegen

function [symcell,cnLen,port2baseseq] = prbsDMRSSequenceSets(carrier,...      % NCellID and NSlot part
                                              dmrs,...                        % DM-RS config (scrambling ID and ports/CDM groups)
                                              r16,...                         % R16 low-PAPR control
                                              prbcell,prbrefpoint,...         % PRB part
                                              dmrssymbols,...                 % Symbol numbers
                                              requiredports)                  % Antenna ports required
        
        % Effective NID value(s) to be used
        if isempty(dmrs.NIDNSCID)
            nidnscid = carrier.NCellID;
        else
            nidnscid = dmrs.NIDNSCID;
        end

        % 6 DM-RS QPSK symbols (type 1) or 4 DM-RS QPSK symbols (type 2) per PRB
        ndmrsre = 4 + (dmrs.DMRSConfigurationType==1)*2;

        % Initialization for codegen
        port2baseseq = zeros(1,0); %#ok<PREALL>
        if r16
            % When dmrs-Downlink-r16/dmrs-Uplink-r16 is signalled, R16 low PAPR DM-RS is used
            % and the DM-RS sequence cinit value has a dependency on the CDM group (TS 38.211 section 7.4.1.1.1)
            if nargin > 6
                [~,pidx] = ismember(requiredports,dmrs.Ports);
                port2baseseq = dmrs.CDMGroups(pidx)+1;
            else
                port2baseseq = dmrs.CDMGroups+1;            % Mapping between ports required and base DM-RS sequences
            end
            groupsused = zeros(1,3);
            groupsused(port2baseseq)=1;   
            basesequenceidxs = find(groupsused);            % Unique CDM group numbers (ordered, +1)  
            nscid = mod(dmrs.NSCID+(basesequenceidxs==2),2);% Complement the NSCID value when CDM group = 1 (basesequenceidxs is 1-based)
        else 
            % If not using dmrs-Downlink-r16/dmrs-Uplink-r16 then there is a single DM-RS sequence set for all DM-RS ports
            % (cinit value has no dependency on the CDM group)
            port2baseseq = ones(1,numel(dmrs.CDMGroups));  % All ports map to the same base DM-RS sequence set
            basesequenceidxs = 1;                          % Single sequence set for all ports
            nscid = dmrs.NSCID;                            % Get the NSCID value
        end    

        % - port2baseseq maps the active ports (by position in the list of active ports for the CDM ) into a position in the symcell 
        % - symcell data is a 2-D cell array, of the form {ncdmgroup,dmrssymbol}
        % - cnLen data is the total number of QPSK symbols for the DM-RS per port

        % Construct a cell array where each row is a given 'base' DM-RS PRBS sequence
        % across the OFDM symbols containing DM-RS. For R16 low PAPR DM-RS, the 'base'
        % sequences have a dependency on the CDM group
        symbperslot = carrier.SymbolsPerSlot;
        symcell = coder.nullcopy(cell(max(basesequenceidxs),length(dmrssymbols)));  % Each cell array row is for different OFDM symbol
        cnLen = zeros(1,length(dmrssymbols));                                       % Number of DM-RS symbols in each OFDM symbol
        % Loop over the base sequences required
        for bsi = 1:length(basesequenceidxs)
            seqidx = basesequenceidxs(bsi);
            % Loop over the OFDM symbols containing DM-RS 
            for i=1:length(dmrssymbols)
                % Included the empty check to avoid run-time error in codegen
                % for empty PRB set with reshape function
                if ~isempty(prbcell{dmrssymbols(i)+1})
                    nslot = mod(double(carrier.NSlot),carrier.SlotsPerFrame);
                    symcell{seqidx,i} = reshape(nr5g.internal.prbsDMRSSequence(...
                                            struct('NIDNSCID',nidnscid,'NSCID',nscid(bsi)),...
                                            ndmrsre,prbcell{dmrssymbols(i)+1},prbrefpoint,nslot,dmrssymbols(i),symbperslot,seqidx-1) ...
                                        ,[],1);
                else
                    symcell{seqidx,i} = zeros(0,1);
                end
                cnLen(i) = length(symcell{seqidx,i});
            end
        end

end