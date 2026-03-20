function sym = nrPDSCHDMRS(carrier,pdsch,varargin)
%nrPDSCHDMRS Physical downlink shared channel demodulation reference signal
%   SYM = nrPDSCHDMRS(CARRIER,PDSCH) returns the demodulation reference
%   signal (DM-RS) symbols, SYM, of physical downlink shared channel for
%   the given carrier configuration object CARRIER and channel transmission
%   configuration object PDSCH according to TS 38.211 Section 7.4.1.1.1.
%   The output SYM is a matrix with number of columns equal to the number
%   of antenna ports configured.
%
%   CARRIER is a carrier configuration object as described in
%   <a href="matlab:help('nrCarrierConfig')">nrCarrierConfig</a> with the following properties:
%
%   NCellID           - Physical layer cell identity (0...1007) (default 1)
%   SubcarrierSpacing - Subcarrier spacing in kHz
%                       (15 (default), 30, 60, 120, 240, 480, 960)
%   CyclicPrefix      - Cyclic prefix ('normal' (default), 'extended')
%   NSizeGrid         - Number of resource blocks in carrier resource
%                       grid (1...275) (default 52)
%   NStartGrid        - Start of carrier resource grid relative to common
%                       resource block 0 (CRB 0) (0...2199) (default 0)
%   NSlot             - Slot number (default 0)
%
%   PDSCH is the physical downlink shared channel configuration object as
%   described in <a href="matlab:help('nrPDSCHConfig')">nrPDSCHConfig</a> with the following properties:
%
%   NSizeBWP              - Size of the bandwidth part (BWP) in terms
%                           of number of physical resource blocks (PRBs)
%                           (1...275) (default []). The default value
%                           implies the value is equal to the size of
%                           carrier resource grid
%   NStartBWP             - Starting PRB index of BWP relative to CRB 0
%                           (0...2473) (default []). The default value
%                           implies the value is equal to the start of
%                           carrier resource grid
%   ReservedPRB           - Cell array of object(s) containing the reserved
%                           physical resource blocks and OFDM symbols
%                           pattern, as described in <a href="matlab:help('nrPDSCHReservedConfig')">nrPDSCHReservedConfig</a>
%                           with properties:
%       PRBSet    - Reserved PRB indices in BWP (0-based) (default [])
%       SymbolSet - OFDM symbols associated with reserved PRBs over one or
%                   more slots (default [])
%       Period    - Total number of slots in the pattern period (default [])
%   ReservedRE            - Reserved resource element (RE) indices
%                           within BWP (0-based) (default [])
%   NumLayers             - Number of transmission layers (1...8)
%                           (default 1)
%   MappingType           - Mapping type of physical downlink shared
%                           channel ('A' (default), 'B')
%   SymbolAllocation      - Symbol allocation of physical downlink shared
%                           channel (default [0 14]). This property is a
%                           two-element vector. First element represents
%                           the start of OFDM symbol in a slot. Second
%                           element represents the number of contiguous
%                           OFDM symbols
%   PRBSet                - Resource block allocation (VRB or PRB indices)
%                           (default 0:51)
%   PRBSetType            - Type of indices used in the PRBSet property
%                           ('VRB' (default), 'PRB')
%   RNTI                  - Radio network temporary identifier (0...65535)
%                           (default 1)
%   DMRS                  - PDSCH-specific DM-RS configuration object, as
%                           described in <a href="matlab:help('nrPDSCHDMRSConfig')">nrPDSCHDMRSConfig</a> with properties:
%       DMRSConfigurationType  - DM-RS configuration type (1 (default), 2)
%       DMRSReferencePoint     - The reference point for the DM-RS
%                                sequence to subcarrier resource mapping
%                                ('CRB0' (default), 'PRB0'). Use 'CRB0', if
%                                the subcarrier reference point for DM-RS
%                                sequence mapping is subcarrier 0 of common
%                                resource block 0 (CRB 0). Use 'PRB0', if
%                                the reference point is subcarrier 0 of the
%                                first PRB of the BWP (PRB 0). The latter
%                                should be used when the PDSCH is signaled
%                                via CORESET 0. In this case the BWP
%                                parameters should also be aligned with
%                                this CORESET
%       DMRSTypeAPosition      - Position of first DM-RS OFDM symbol in a
%                                slot (2 (default), 3)
%       DMRSLength             - Number of consecutive DM-RS OFDM symbols
%                                (1 (default), 2)
%       DMRSAdditionalPosition - Maximum number of DM-RS additional
%                                positions (0...3) (default 0)
%       CustomSymbolSet        - Custom DM-RS symbol locations (0-based)
%                                (default []). This property is used to
%                                override the standard defined DM-RS symbol
%                                locations. Each entry corresponds to a
%                                single-symbol DM-RS
%       DMRSPortSet            - DM-RS antenna port set (0...11)
%                                (default []). The default value implies
%                                that the values are in the range from 0 to
%                                NumLayers-1
%       NIDNSCID               - DM-RS scrambling identities (0...65535)
%                                (default []). Use empty ([]) to set the
%                                values to NCellID
%       NSCID                  - DM-RS scrambling initialization
%                                (0 (default), 1)
%       DMRSDownlinkR16        - Release 16 low PAPR DM-RS 
%                                (0 (default), 1)
%       DMRSEnhancedR18        - Release 18 enhanced DM-RS multiplexing
%                                (0 (default), 1) 
%
%   SYM = nrPDSCHDMRS(CARRIER,PDSCH,NAME,VALUE) specifies additional
%   options as NAME,VALUE pairs to allow control over the data type of the
%   output symbols:
%
%   'OutputDataType' - 'double' for double precision (default)
%                      'single' for single precision
%
%   Example:
%   % Generate DM-RS symbols of a physical downlink shared channel
%   % occupying the 10 MHz bandwidth for a 15 kHz subcarrier spacing (SCS)
%   % carrier. Configure DM-RS with type A position set to 2, number of
%   % additional positions set to 0, length set to 1, and configuration
%   % type set to 1.
%
%   carrier = nrCarrierConfig;
%   carrier.SubcarrierSpacing = 15; % 15 kHz SCS carrier
%   carrier.NSizeGrid = 52;         % 10 MHz bandwidth (52 resource blocks)
%   pdsch = nrPDSCHConfig;
%   pdsch.DMRS.DMRSTypeAPosition = 2;
%   pdsch.DMRS.DMRSAdditionalPosition = 0;
%   pdsch.DMRS.DMRSLength = 1;
%   pdsch.DMRS.DMRSConfigurationType = 1;
%   sym = nrPDSCHDMRS(carrier,pdsch);
%
%   See also nrPDSCHDMRSIndices, nrTimingEstimate, nrChannelEstimate,
%   nrPDSCHPTRS, nrPDSCHConfig, nrPDSCHDMRSConfig, nrCarrierConfig.

%   Copyright 2019-2025 The MathWorks, Inc.

%#codegen

    narginchk(2,4);

    % Validate inputs
    [~,~,nSizeBWP,~,~] = nr5g.internal.pdsch.validateInputs(carrier,pdsch);

    % Get prbset, symbolset and DM-RS symbols
    [prbset,symbolset,dmrssymbols,ldash] = nr5g.internal.pxsch.initializeResources(carrier,pdsch,nSizeBWP);

    % Derive the number of DM-RS antenna ports that will be used
    nports = double(pdsch.NumLayers);

    dmrs = complex(zeros(0,nports));
    if ~isempty(dmrssymbols)
        % Remove the reserved resource blocks
        prbcell = nr5g.internal.pdsch.extractActiveResourceBlocks(pdsch.ReservedPRB,prbset,symbolset,carrier.NSlot,carrier.SymbolsPerSlot);

        % Get the PRB reference point associated with PRB set values for
        % the DM-RS sequence indexing
        prbrefpoint = nr5g.internal.pdsch.getRBReferencePoint(carrier.NStartGrid,pdsch.NStartBWP,pdsch.DMRS.DMRSReferencePoint);

        % Construct the set of PRBS base sequences for the DM-RS antenna ports
        [symcell,cnLen,port2baseseq] = nr5g.internal.prbsDMRSSequenceSets(carrier,...
                                    pdsch.DMRS,...
                                    pdsch.DMRS.DMRSDownlinkR16,...
                                    prbcell,prbrefpoint,...              % PRB required
                                    dmrssymbols);                        % Specific OFDM symbol numbers required

        % Accumulated lengths of the DM-RS when concatenated across the OFDM symbols
        % This is used to identify ranges for the DM-RS values for a given symbol
        % within the concatenated array output
        cndmrs = [0 cumsum(cnLen)];
        % Preallocate array for the returned DM-RS
        dmrs = complex(zeros(cndmrs(end),nports));

        % Loop over the ports/layers and apply TD/TD OCC masks
        fmaskAllPorts = pdsch.DMRS.FrequencyWeights;
        tmaskAllPorts = pdsch.DMRS.TimeWeights;

        % 6 DM-RS QPSK symbols (type 1) or 4 DM-RS QPSK symbols (type 2) per PRB
        ndmrsre = 4 + (pdsch.DMRS.DMRSConfigurationType==1)*2;

        nwf = size(fmaskAllPorts,1);
        nfmaskrep = lcm(ndmrsre,nwf)/nwf;

        % Loop over the antenna ports and apply associated FD/TD OCC masks
        for pidx = 1:nports

            % Get FD OCC mask to be applied to every DM-RS carrying symbol of port
            fmask = fmaskAllPorts(:,pidx);
            % Replicate the basic FD OCC mask, across the minimum number of PRB for a complete cover             
            fmaskprb = reshape(repmat(fmask,nfmaskrep,1),ndmrsre,[]);   

            % ldash (l') contains the TD weight selection to use per symbol
            tmask = tmaskAllPorts(ldash+1,pidx);

            % Apply combined time and frequency mask values and concatenate
            % across all the OFDM symbols
            % For current port, step through each DM-RS symbols values, apply FD/TD OCC weights for that symbol, and assign result into a subset of the output matrix
            for i = 1:size(symcell,2)
               % Select the FD masks for each PRB in allocation and reshape into a single column
               rbidxs = prbcell{dmrssymbols(i)+1}+prbrefpoint;
               fmaskallprb = reshape(fmaskprb(:,mod(rbidxs,size(fmaskprb,2))+1),[],1);  
               % Apply time and frequency weights to DM-RS symbols
               dmrs(cndmrs(i)+1:cndmrs(i+1),pidx) = symcell{port2baseseq(pidx),i}.*fmaskallprb*tmask(i);
            end
        end
        
    end

    % Apply options
    if nargin > 2
        fcnName = 'nrPDSCHDMRS';
        opts = nr5g.internal.parseOptions(fcnName,{'OutputDataType'},varargin{:});
        sym = cast(dmrs,opts.OutputDataType);
    else
        % Cast to double to have same behavior for empty output in codegen
        % path and simulation path
        sym = double(dmrs);
    end

end