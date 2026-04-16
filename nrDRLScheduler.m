classdef nrDRLScheduler < nrScheduler
    % ===================================================================
    % DRL MOD START - Properties for DRL integration
    % ===================================================================
    properties (Access = public)
        %DRLAction Action matrix from DRL agent [NumLayers x NumRBGs]
        % Each element A(l,m) specifies:
        %   1..|U|: index of UE in eligibleUEs to allocate at layer l, RBG m
        %   |U|+1:  no allocation (skip this layer-RBG combination)
        % Must be set externally before each TTI scheduling call
        DRLAction = []

        %EnableDRL Flag to enable/disable DRL-based scheduling
        % When true: use DRL action for UE selection
        % When false: use default nrScheduler behavior
        EnableDRL = true

        %NumLayers Number of MU-MIMO spatial layers (default: 16)
        NumLayers = 1

        %SchedulingStrategy Strategy identifier
        % "DRL" - use DRL action
        % "Default" - use nrScheduler default (round-robin/PF/BestCQI)
        SchedulingStrategy = "DRL"

        %MaxUsersPerRBG Maximum number of UEs that can be co-scheduled on same RBG
        % This is typically limited by MU-MIMO capability
        MaxUsersPerRBG = 16
        
        %EnablePairingConstraints Master flag to enable/disable MU-MIMO pairing constraints
        % When true: both i1 matching AND precoder orthogonality checks are applied
        % When false: no pairing constraints (any UE can be co-scheduled on same RBG)
        EnablePairingConstraints = true
        
        %SemiOrthogonalityFactor Threshold for precoder orthogonality (0-1)
        % Two precoders are considered orthogonal if |W1'*W2|/max < (1 - SemiOrthogonalityFactor)
        % Higher value = stricter orthogonality requirement = fewer pairings but less interference
        % Typical values: 0.5-0.8
        SemiOrthogonalityFactor = 0.7
        
        %MU_MCSBackoff MCS reduction for MU-MIMO transmissions
        % Reduces MCS when multiple UEs share RBG to account for residual interference
        % Value is number of MCS levels to reduce per co-scheduled UE (beyond first)
        MU_MCSBackoff = 2

        %DRLDebug Enable debug output for DRL scheduling
        DRLDebug = false

        %LastDRLAllocation Store the last DRL allocation for debugging/logging
        LastDRLAllocation = struct('allottedUEs', [], 'freqAllocation', [], ...
            'layerAssignment', [], 'eligibleUEs', [])

        % ---------------------------------------------------------------
        % Socket Communication Properties
        % ---------------------------------------------------------------
        %DRL_IP IP address of Python DRL server
        DRL_IP = "127.0.0.1"

        %DRL_Port TCP port of Python DRL server
        DRL_Port = 5555

        %DRL_Socket TCP client socket object
        DRL_Socket = []

        %DRL_IsConnected Flag indicating socket connection status
        DRL_IsConnected = false

        %DRL_RxBuf Receive buffer for incoming data
        DRL_RxBuf uint8 = uint8([])

        %DRL_Terminator Line terminator character (newline)
        DRL_Terminator = uint8(10)

        %DRL_TimeoutSec Socket read timeout in seconds
        DRL_TimeoutSec = 5

        %UseSocket Flag to enable socket-based action retrieval
        % When true: automatically fetch action from Python server
        % When false: use manually set DRLAction property
        UseSocket = false

        %MaxUEs Maximum number of UEs (for feature matrix sizing)
        MaxUEs = 10

        %SubbandSize RBs per feature subband (for CQI features)
        SubbandSize = 16

        % ---------------------------------------------------------------
        % TRAINING MODE Properties (layer-by-layer protocol with Python)
        % ---------------------------------------------------------------
        %TrainingMode Enable layer-by-layer DRL training protocol
        % When true: each TTI uses LAYER_OBS/LAYER_ACT/LAYER_REWARD protocol
        % When false: use existing TTI_OBS/TTI_ALLOC protocol
        TrainingMode = false

        %TrainingTTICount Counter of TTIs processed in training mode
        TrainingTTICount = 0

        %TrainingTTISlotIndex Linear slot index per training TTI (0-based)
        % slotLinear = NFrame * NumSlotsFrame + NSlot
        TrainingTTISlotIndex = []

        %MaxBuffer Normalization constant for buffer status [bytes]
        % Used to normalize BufferStatusDL to [0,1] range
        MaxBuffer = 1e6

        %MaxTput Normalization constant for throughput [Mbps]
        MaxTput = 100.0

        %TBSTableBytes TBS lookup table [bytes] for tbs_38214_bytes approximation
        % Precomputed: TBSTableBytes(mcs+1, nprb) -> bytes
        % Using approximate formula: Ninfo * n_prb * overhead_factor / 8
        % We recompute inline for simplicity.
        TBSPrbSize = 18    % PRBs per RBG (typical)

        %TrainingTputEMA Per-UE throughput EMA [Mbps], maintained internally.
        % Updated each TTI in computeTrainingEvalMetrics using the same formula
        % as nrScheduler.updateUEsServedDataRate (window = SchedulerConfig.PFSWindowSize).
        % This mirrors UEsServedDataRate so logged metrics match the MATLAB visualizer
        % regardless of whether SchedulerStrategy==1 (PFS) is active.
        TrainingTputEMA = []   % initialised lazily (MaxUEs x 1)

        %UEList nrUE array — set externally (CSIRS_32T32R_SOCKET.m) after connectUE().
        % Required for ActualTputEMA: reads MAC.ReceivedBytes per UE per TTI.
        UEList = []

        %PrevRxBytes Cumulative DL ReceivedBytes per UE at end of last TTI [bytes].
        % Used to compute per-TTI delta for ActualTputEMA.
        PrevRxBytes = []   % [MaxUEs x 1], lazily initialised

        %ActualTputEMA Per-UE actual DL received throughput EMA [Mbps].
        % Ground-truth: updated from delta(UE MAC ReceivedBytes) each TTI.
        % Replaces TrainingTputEMA as avg_tp in obs (fR feature) and
        % reward denominator (Ru in PF formula).
        ActualTputEMA = []   % [MaxUEs x 1], lazily initialised

        %PythonUEPriority Per-UE priority counter mirroring adapter's ue_priority.
        % Used to replicate adapter's selected_ues ordering so MATLAB can correctly
        % map action local-index → RNTI.  Lazily initialised to zeros(MaxUEs,1).
        % Incremented every TTI; reset to 0 for allocated UEs (same logic as adapter).
        PythonUEPriority = []   % [MaxUEs x 1], lazily initialized

        %CSI_Available Flag to indicate if CSI has been reported by UEs.
        % Enforces SU-MIMO until true.
        CSI_Available = false

        %LastTTIStartWSubbandMap Store per-RNTI precoder W used for TTI_START
        % Used to sanity-check against pairing-constraint W in scheduling.
        LastTTIStartWSubbandMap = []

        %LastTTIStartEligibleUEs Store eligible UE list at last TTI_START
        LastTTIStartEligibleUEs = []

        %LastTTIStartTTI Store TTI index for last TTI_START payload
        LastTTIStartTTI = -1
    end
    % ===================================================================
    % DRL MOD END - Properties
    % ===================================================================

    methods (Access = public)
        function obj = nrDRLScheduler(varargin)
            %nrDRLScheduler Constructor
            % Calls parent constructor and initializes DRL-specific properties
        end

        function setDRLAction(obj, actionMatrix)
            %setDRLAction Set the DRL action matrix for current TTI
            %   obj.setDRLAction(actionMatrix) sets the action matrix A
            %   where A is [NumLayers x NumRBGs]
            %
            %   actionMatrix(l,m) ∈ {1..|U|+1}:
            %       1..|U|: allocate eligible UE at index i to layer l, RBG m
            %       |U|+1:  no allocation

            obj.DRLAction = actionMatrix;
            if obj.DRLDebug
                fprintf('[nrDRLScheduler] DRLAction set: size=[%d x %d]\n', ...
                    size(actionMatrix,1), size(actionMatrix,2));
            end
        end

        function clearDRLAction(obj)
            %clearDRLAction Clear the DRL action matrix
            obj.DRLAction = [];
        end

        % ---------------------------------------------------------------
        % Socket Communication Methods
        % ---------------------------------------------------------------
        function success = connectToDRLAgent(obj, ip, port)
            %connectToDRLAgent Connect to Python DRL server via TCP socket
            %   success = connectToDRLAgent(obj) connects using default IP/port
            %   success = connectToDRLAgent(obj, ip, port) connects to specified address
            %
            %   Returns true if connection successful, false otherwise

            if nargin >= 2 && ~isempty(ip)
                obj.DRL_IP = ip;
            end
            if nargin >= 3 && ~isempty(port)
                obj.DRL_Port = port;
            end

            success = false;

            % Clean up existing socket
            if ~isempty(obj.DRL_Socket)
                try
                    delete(obj.DRL_Socket);
                catch
                end
                obj.DRL_Socket = [];
            end

            try
                obj.DRL_Socket = tcpclient(obj.DRL_IP, obj.DRL_Port, ...
                    'Timeout', 60, 'ConnectTimeout', 60);
                obj.DRL_Socket.InputBufferSize  = 1048576;
                obj.DRL_Socket.OutputBufferSize = 1048576;
                obj.DRL_IsConnected = true;
                obj.UseSocket = true;
                fprintf('[nrDRLScheduler] Connected to Python DRL at %s:%d\n', ...
                    obj.DRL_IP, obj.DRL_Port);
                success = true;
            catch ME
                fprintf('[nrDRLScheduler] Connection failed: %s\n', ME.message);
                obj.DRL_IsConnected = false;
                obj.UseSocket = false;
            end
        end

        function disconnectFromDRLAgent(obj)
            %disconnectFromDRLAgent Disconnect from Python DRL server

            if ~isempty(obj.DRL_Socket)
                try
                    delete(obj.DRL_Socket);
                catch
                end
                obj.DRL_Socket = [];
            end
            obj.DRL_IsConnected = false;
            fprintf('[nrDRLScheduler] Disconnected from Python DRL\n');
        end

        function drlSendJSON(obj, data)
            %drlSendJSON Send one JSON message (newline-delimited) to Python.
            jsonStr = jsonencode(data);
            write(obj.DRL_Socket, uint8([jsonStr, newline]));
        end

        function data = drlRecvJSON(obj, ~)
            %drlRecvJSON Receive one JSON message from Python.
            line = obj.readLineFromSocket();
            if isempty(line)
                error('[nrDRLScheduler] drlRecvJSON: timeout – no response from Python');
            end
            data = jsondecode(line);
        end


        function line = readLineFromSocket(obj)
            %readLineFromSocket Read a complete line (until newline) from socket
            %   Helper method for socket communication

            line = '';
            startTime = tic;

            while toc(startTime) < obj.DRL_TimeoutSec
                if obj.DRL_Socket.NumBytesAvailable > 0
                    newData = read(obj.DRL_Socket, obj.DRL_Socket.NumBytesAvailable, 'uint8');
                    obj.DRL_RxBuf = [obj.DRL_RxBuf, newData];
                end

                % Check for terminator
                termIdx = find(obj.DRL_RxBuf == obj.DRL_Terminator, 1);
                if ~isempty(termIdx)
                    line = char(obj.DRL_RxBuf(1:termIdx-1));
                    obj.DRL_RxBuf = obj.DRL_RxBuf(termIdx+1:end);
                    return;
                end

                pause(0.001); % Small delay to avoid busy-waiting
            end

            if obj.DRLDebug
                fprintf('[nrDRLScheduler] Socket read timeout\n');
            end
        end

    end

    methods (Access = protected)
        function dlAssignments = scheduleNewTransmissionsDL(obj, timeFrequencyResource, schedulingInfo)
            %scheduleNewTransmissionsDL Assign resources for new DL transmissions
            %   Overrides the parent method to inject DRL-based UE selection
            %   while keeping all other scheduler logic intact.
            %
            %   When EnableDRL=true and ResourceAllocationType=0 (RAT-0):
            %       Uses runSchedulingStrategyDRL_RAT0 for UE selection
            %   Otherwise:
            %       Calls parent scheduleNewTransmissionsDL method

            % ===================================================================
            % DRL MOD START - Check DRL mode and RAT-0
            % ===================================================================
            schedulerConfig = obj.SchedulerConfig;

            % ---------------------------------------------------------------
            % TRAINING MODE: layer-by-layer protocol (Python server)
            % ---------------------------------------------------------------
            
            % DRL MOD: Check if this slot is reserved for CSI-RS. If so, skip DRL scheduling.
            isCSIRSSlot = false;
            csirsConfig = obj.UEContext(1).ComponentCarrier(1).CSIRSConfiguration;
            if ~isempty(csirsConfig) && isnumeric(csirsConfig.CSIRSPeriod)
                slotNum = timeFrequencyResource.NSlot;
                frameNum = timeFrequencyResource.NFrame;
                numSlotsFrame = obj.CellConfig(1).NumSlotsFrame;
                if mod(numSlotsFrame*frameNum + slotNum - csirsConfig.CSIRSPeriod(2), csirsConfig.CSIRSPeriod(1)) == 0
                    isCSIRSSlot = true;
                end
            end

            % hasAvailableRBGs = ~all(timeFrequencyResource.FrequencyResource(:));
            fprintf('[TRAINING-CHECK] F=%d S=%d | TrainingMode=%d, DRL_IsConnected=%d, EnableDRL=%d, RAType=%d, isCSIRSSlot=%d\n', ...
                obj.CurrFrame, obj.CurrSlot, ...
                obj.TrainingMode, obj.DRL_IsConnected, obj.EnableDRL, ...
                schedulerConfig.ResourceAllocationType, isCSIRSSlot);

            % DRL MOD: Early return if this is a CSI-RS slot to avoid DRL execution
            if isCSIRSSlot
                dlAssignments = struct([]);
                return;
            end

            if obj.TrainingMode && obj.DRL_IsConnected && obj.EnableDRL && ...
                    (schedulerConfig.ResourceAllocationType == 0)
                fprintf('[TRAINING-CHECK]: Training Path\n');
                dlAssignments = obj.scheduleWithTrainingProtocol( ...
                    timeFrequencyResource, schedulingInfo);
                return;
            end

            % Check if we should use DRL scheduling (DRLAction must be pre-set)
            useDRL = obj.EnableDRL && ...
                     ~isempty(obj.DRLAction) && ...
                     (obj.SchedulingStrategy == "DRL") && ...
                     (schedulerConfig.ResourceAllocationType == 0);  % RAT-0 only

            if useDRL
                % Use DRL-based scheduling
                fprintf('[TRAINING-CHECK]: Normal Path\n');
                dlAssignments = obj.scheduleNewTransmissionsDL_DRL(timeFrequencyResource, schedulingInfo);
            else
                % Use default nrScheduler behavior
                dlAssignments = scheduleNewTransmissionsDL@nrScheduler(obj, timeFrequencyResource, schedulingInfo);
            end
            % ===================================================================
            % DRL MOD END - Check DRL mode
            % ===================================================================
        end

        % ===================================================================
        % DRL MOD START - DRL-based scheduling method
        % ===================================================================
        function dlAssignments = scheduleNewTransmissionsDL_DRL(obj, timeFrequencyResource, schedulingInfo)
            %scheduleNewTransmissionsDL_DRL DRL-based DL scheduling implementation
            %   This method follows the exact same flow as the original
            %   scheduleNewTransmissionsDL but replaces the UE selection
            %   logic with DRL action interpretation.

            % Read time-frequency resources (same as original)
            scheduledSlot = timeFrequencyResource.NSlot; %#ok<NASGU>
            startSym = timeFrequencyResource.SymbolAllocation(1); %#ok<NASGU>
            numSym = timeFrequencyResource.SymbolAllocation(2);
            updatedFrequencyStatus = timeFrequencyResource.FrequencyResource;

            % Read scheduling info structure (same as original)
            eligibleUEs = schedulingInfo.EligibleUEs;
            numNewTxs = min(size(eligibleUEs,2), schedulingInfo.MaxNumUsersTTI);

            % Early return if no eligible UEs
            if isempty(eligibleUEs)
                dlAssignments = struct([]);
                return;
            end

            % Pre-allocate DL grants (same as original)
            dlAssignments = obj.DLGrantArrayStruct(1:numNewTxs);

            cellConfig = obj.CellConfig;
            schedulerConfig = obj.SchedulerConfig;
            ueContext = obj.UEContext;

            % ---------------------------------------------------------------
            % Select rank and precoding matrix for the eligible UEs
            % (Reuse logic from parent class)
            % ---------------------------------------------------------------
            numEligibleUEs = size(eligibleUEs,2);
            W = cell(numEligibleUEs, 1);
            H = cell(numEligibleUEs, 1);
            SINR = cell(numEligibleUEs, 1);
            selectedRanks = zeros(numEligibleUEs, 1);  % Renamed from 'rank' to avoid conflict with built-in
            rbRequirement = zeros(obj.NumUEs, 1);
            channelQuality = zeros(obj.NumUEs, cellConfig.NumResourceBlocks);
            cqiSizeArray = ones(cellConfig.NumResourceBlocks, 1);
            nVar = 0;

            % ----------------------------------------------------------
            % DRL MOD: Pre-allocate i1 storage for PMI constraint
            % i1 is wideband PMI component - UEs with same i1 have
            % similar beam direction and can be paired for MU-MIMO
            % ----------------------------------------------------------
            ueI1Values = cell(numEligibleUEs, 1);  % Store PMI i1 for each eligible UE
            
            for i = 1:numEligibleUEs
                rnti = eligibleUEs(i);
                eligibleUEContext = ueContext(rnti);
                carrierContext = eligibleUEContext.ComponentCarrier(1);
                csiMeasurement = carrierContext.CSIMeasurementDL;
                % Dùng mean(CQI) thay vì max(CQI) để tránh overestimate kênh
                % max(CQI) lấy giá trị tốt nhất qua tất cả layers và subbands → MCS quá cao → BLER cao
                csiMeasurementCQI = floor(mean(csiMeasurement.CSIRS.CQI(:))) * cqiSizeArray(1);
                channelQuality(rnti, :) = csiMeasurementCQI;

                if isempty(carrierContext.CSIRSConfiguration)
                    numCSIRSPorts = obj.CellConfig(1).NumTransmitAntennas;
                else
                    numCSIRSPorts = carrierContext.CSIRSConfiguration.NumCSIRSPorts;
                end

                [selectedRanks(i), W{i}] = selectRankAndPrecodingMatrixDL(obj, rnti, csiMeasurement, numCSIRSPorts);

                % DRL MOD: Extract PMI i1 for pairing constraint
                % i1(1:2) contains [i11, i12] - the wideband beam indices
                if isfield(csiMeasurement, 'CSIRS') && ...
                   isfield(csiMeasurement.CSIRS, 'PMISet') && ...
                   isfield(csiMeasurement.CSIRS.PMISet, 'i1') && ...
                   numel(csiMeasurement.CSIRS.PMISet.i1) >= 2
                    ueI1Values{i} = csiMeasurement.CSIRS.PMISet.i1(1:2);
                else
                    % If no i1 available, use empty (will skip i1 check)
                    ueI1Values{i} = [];
                end

                % For SRS-based DL MU-MIMO CSI measurements
                if ~isempty(schedulerConfig.MUMIMOConfigDL) && ~isempty(csiMeasurement.SRS)
                    H{i} = csiMeasurement.SRS.H;
                    nVar = csiMeasurement.SRS.nVar;
                    SINR{i} = csiMeasurement.SRS.sinr;
                end

                [~, rbRequirement(rnti)] = calculateRBRequirement(obj, rnti, obj.DLType, numSym, selectedRanks(i));
            end

            % ---------------------------------------------------------------
            % DRL MOD: Sanity check W used in TTI_START vs pairing constraints
            % ---------------------------------------------------------------
            if obj.TrainingMode && obj.DRLDebug && ~isempty(obj.LastTTIStartWSubbandMap)
                lastUEs = obj.LastTTIStartEligibleUEs(:);
                currUEs = eligibleUEs(:);
                sameTTI = (obj.LastTTIStartTTI == (obj.TrainingTTICount - 1));
                sameUEs = (numel(lastUEs) == numel(currUEs)) && isequal(sort(lastUEs), sort(currUEs));
                if sameTTI && sameUEs
                    mismatchCount = 0;
                    for i = 1:numEligibleUEs
                        rnti = eligibleUEs(i);
                        W_tti = obj.LastTTIStartWSubbandMap{rnti};
                        W_pair = W{i};
                        [isSame, maxAbsDiff] = obj.comparePrecoderMatrices(W_tti, W_pair);
                        if ~isSame
                            mismatchCount = mismatchCount + 1;
                            fprintf('[DRL][W-SANITY] TTI %d UE %d: W mismatch (max|diff|=%.3g)\n', ...
                                obj.LastTTIStartTTI, rnti, maxAbsDiff);
                        end
                    end
                    if mismatchCount == 0
                        fprintf('[DRL][W-SANITY] TTI %d: All %d eligible UE precoders identical between TTI_START and pairing.\n', ...
                            obj.LastTTIStartTTI, numEligibleUEs);
                    end
                else
                    fprintf('[DRL][W-SANITY] TTI_START W cache not aligned (sameTTI=%d, sameUEs=%d) — skipping.\n', ...
                        sameTTI, sameUEs);
                end
            end

            % ---------------------------------------------------------------
            % Create scheduler input structure (same format as original)
            % ---------------------------------------------------------------
            schedulerInput = obj.SchedulerInputStruct;
            schedulerInput.linkDir = obj.DLType;
            schedulerInput.eligibleUEs = eligibleUEs;
            schedulerInput.selectedRank = selectedRanks;
            schedulerInput.bufferStatus = [ueContext(eligibleUEs).BufferStatusDL];
            schedulerInput.lastSelectedUE = obj.LastSelectedUEDL;
            schedulerInput.channelQuality = channelQuality(eligibleUEs, :);
            schedulerInput.freqOccupancyBitmap = updatedFrequencyStatus;
            schedulerInput.rbAllocationLimit = obj.RBAllocationLimitDL;
            schedulerInput.rbRequirement = rbRequirement(eligibleUEs);
            schedulerInput.maxNumUsersTTI = schedulingInfo.MaxNumUsersTTI;
            schedulerInput.numSym = numSym;
            schedulerInput.W = W;
            schedulerInput.ueI1Values = ueI1Values;  % DRL MOD: Store i1 for pairing constraint

            % For SRS-based MU-MIMO
            csiMeasurementSignalDLType = obj.SchedulerConfig.CSIMeasurementSignalDLType;
            if ~isempty(schedulerConfig.MUMIMOConfigDL) && csiMeasurementSignalDLType
                isSRSApplicable = any(arrayfun(@(x) ~isempty(obj.UEContext(x).CSIMeasurementDL.SRS), eligibleUEs));
                if isSRSApplicable
                    schedulerInput.channelMatrix = H;
                    schedulerInput.nVar = nVar;
                    schedulerInput.SINRs = SINR;
                end
            end

            % ---------------------------------------------------------------
            % DRL-based UE selection (replaces runSchedulingStrategyRAT0)
            % ---------------------------------------------------------------
            [allottedUEs, freqAllocation, mcsIndex, W] = runSchedulingStrategyDRL_RAT0(obj, schedulerInput);
            fprintf("DL MCS Indices:%s\n",mat2str(mcsIndex))
            % ---------------------------------------------------------------
            % Create DL assignments (same as original)
            % ---------------------------------------------------------------
            numAllottedUEs = numel(allottedUEs);
            for index = 1:numAllottedUEs
                gNBCarrierIndex = 1;
                dlAssignments(index).GNBCarrierIndex = gNBCarrierIndex;
                selectedUE = allottedUEs(index);

                % Find UE index in eligible UEs set
                selectedUEIdx = find(eligibleUEs == selectedUE, 1);

                % MCS offset value
                carrierContext = ueContext(selectedUE).ComponentCarrier(gNBCarrierIndex);
                mcsOffset = fix(carrierContext.MCSOffset(schedulerInput.linkDir+1));
                % mcsOffset = 0;
                rawMCS = mcsIndex(index);
                effectiveMCS = min(max(rawMCS - mcsOffset, 0), 28);

                % Fill DL assignment properties
                dlAssignments(index).RNTI = selectedUE;
                dlAssignments(index).FrequencyAllocation = freqAllocation(index, :);
                dlAssignments(index).MCSIndex = effectiveMCS;
            
                fprintf('[LA-DBG] F=%d S=%d UE=%d rawMCS=%d offsetRaw=%.3f offsetFix=%d effMCS=%d\n', ...
                    obj.CurrFrame, obj.CurrSlot, selectedUE, rawMCS, ...
                    carrierContext.MCSOffset(schedulerInput.linkDir+1), mcsOffset, effectiveMCS);
                
                % --- Kiểm tra precoding matrix W ---
                W_ue = W{selectedUEIdx};
                ueRank = schedulerInput.selectedRank(selectedUEIdx);
                if obj.DRLDebug
                    if isempty(W_ue)
                        fprintf('[W-CHECK] UE %d: W EMPTY (rank=%d) — sẽ dùng fallback của MATLAB\n', selectedUE, ueRank);
                    elseif isscalar(W_ue) && W_ue == 1
                        fprintf('[W-CHECK] UE %d: W là FALLBACK (scalar 1) rank=%d\n', ...
                            selectedUE, ueRank);
                    else
                        colNorms = vecnorm(W_ue);
                        fprintf('[W-CHECK] UE %d: W=[%d x %d] rank=%d Valid=OK colNorms=[%s]\n', ...
                            selectedUE, size(W_ue,1), size(W_ue,2), ueRank, num2str(colNorms, '%.3f '));
                    end
                end
                dlAssignments(index).W = W_ue;

                % Mark frequency resources as assigned
                updatedFrequencyStatus = updatedFrequencyStatus | freqAllocation(index,:);
            end

            % Remove invalid trailing entries - ensure column vector for concatenation
            if numAllottedUEs > 0
                dlAssignments = dlAssignments(1:numAllottedUEs);
                % Ensure column vector (required by parent nrScheduler)
                dlAssignments = dlAssignments(:);
            else
                % Empty case: return empty struct array compatible with parent
                dlAssignments = struct([]);
            end

            % Update last selected UE
            if numAllottedUEs > 0
                obj.LastSelectedUEDL = allottedUEs(end);
            end
        end
        % ===================================================================
        % DRL MOD END - DRL-based scheduling method
        % ===================================================================

        % ===================================================================
        % DRL MOD START - DRL scheduling strategy for RAT-0
        % ===================================================================
        function [allottedUEs, freqAllocation, mcsIndex, W] = runSchedulingStrategyDRL_RAT0(obj, schedulerInput)
            %runSchedulingStrategyDRL_RAT0 DRL-based UE selection for RAT-0
            %   This function implements DRL action interpretation for MU-MIMO
            %   scheduling with Resource Allocation Type 0 (RBG-based bitmap).
            %
            %   [ALLOTTEDUES, FREQALLOCATION, MCSINDEX, W] =
            %   runSchedulingStrategyDRL_RAT0(OBJ, SCHEDULERINPUT) returns:
            %
            %   ALLOTTEDUES - Vector of RNTIs actually allocated (unique)
            %   FREQALLOCATION - Matrix [numAllottedUEs x numRBGs] with RBG bitmap per UE
            %   MCSINDEX - Vector of MCS indices for each allotted UE
            %   W - Cell array of precoding matrices for each allotted UE
            %
            %   DRL Action Interpretation:
            %   - obj.DRLAction is [NumLayers x NumRBGs] matrix
            %   - DRLAction(l,m) ∈ {1..|U|+1}:
            %       * 1..|U|: allocate eligibleUEs(i) to layer l, RBG m
            %       * |U|+1:  no allocation (skip)
            %   - Action masking enforces:
            %       * No duplicate UE allocation on same RBG across layers
            %       * No allocation on RBGs occupied by retransmissions

            eligibleUEs = schedulerInput.eligibleUEs;
            numEligibleUEs = numel(eligibleUEs);
            rbgOccupancyBitmap = schedulerInput.freqOccupancyBitmap;
            numRBGs = numel(rbgOccupancyBitmap);

            % Get DRL action from property (set by scheduleWithTrainingProtocol)
            drlAction = obj.DRLAction;

            % Handle empty or invalid action
            if isempty(drlAction)
                if obj.DRLDebug
                    fprintf('[nrDRLScheduler] Empty DRLAction, using no-allocation default\n');
                end
                drlAction = (numEligibleUEs + 1) * ones(obj.NumLayers, numRBGs);
            end

            numLayers =  obj.NumLayers;

            % Initialize output arrays
            ueRBGAllocation = zeros(obj.NumUEs, numRBGs);  % Full allocation matrix
            ueLayerCount = zeros(obj.NumUEs, numRBGs);     % Track layers per UE per RBG

            % DRL MOD: Get i1 values for i1 matching constraint
            ueI1Values = schedulerInput.ueI1Values;
            
            % DRL MOD: Get precoders for orthogonality check
            W = schedulerInput.W;  % Cell array of precoders per eligible UE
            
            % DRL MOD: Respect per-TTI remaining newTx budget from caller.
            % assignDLResourceTTI sets schedulingInfo.MaxNumUsersTTI as:
            %   MaxNumUsersPerTTI - numUEsRetx
            % so DRL newTx and reTx share one total limit.
            maxUsersPerTTI = schedulerInput.maxNumUsersTTI;
            if isempty(maxUsersPerTTI) || ~isfinite(maxUsersPerTTI)
                maxUsersPerTTI = obj.SchedulerConfig.MaxNumUsersPerTTI;
            end
            maxUsersPerTTI = max(0, floor(double(maxUsersPerTTI)));

            totalUniqueUEsAllocated = [];  % Track unique UEs across all RBGs
            fprintf("Final Allocation Map: %s\n",mat2str(drlAction))
            fprintf("Max Number of UEs for New Tx %d\n",maxUsersPerTTI)
            % Helper function for ternary operation
            ternary = @(cond, trueVal, falseVal) subsref({falseVal, trueVal}, struct('type', '{}', 'subs', {{cond + 1}}));
            
            allocMap = (-1)*ones(numLayers, numRBGs);
            % Process each RBG
            for rbg = 1:numRBGs
                % Skip if RBG is occupied by retransmission
                if rbgOccupancyBitmap(rbg)
                    continue;
                end

                % Track which UEs are already allocated on this RBG (action masking)
                allocatedUEsThisRBG = [];
                allocatedUEIndicesThisRBG = [];  % DRL MOD: Track indices for i1 lookup

                % Process each layer
                for layer = 1:numLayers
                    actionIndex = drlAction(layer, rbg);

                    % Validate action index
                    if actionIndex <= 0 || actionIndex > numEligibleUEs + 1
                        % Invalid action: treat as no allocation
                        continue;
                    end

                    % Action = numEligibleUEs + 1 means "no allocation"
                    if actionIndex == numEligibleUEs + 1
                        continue;
                    end

                    % Get selected UE RNTI
                    selectedUE = eligibleUEs(actionIndex);

                    % Action Masking: Check if UE already allocated on this RBG
                    if ismember(selectedUE, allocatedUEsThisRBG)
                        % Skip duplicate allocation (action masked)
                        if obj.DRLDebug
                            fprintf('[DRL] Layer %d, RBG %d: UE %d already allocated, skipping\n', ...
                                layer, rbg, selectedUE);
                        end
                        continue;
                    end

                    % Check MaxUsersPerRBG constraint
                    if numel(allocatedUEsThisRBG) >= obj.MaxUsersPerRBG
                        if obj.DRLDebug
                            fprintf('[DRL] Layer %d, RBG %d: MaxUsersPerRBG reached, skipping\n', ...
                                layer, rbg);
                        end
                        continue;
                    end
                    
                    % ----------------------------------------------------------
                    % DRL MOD: Enforce SU-MIMO if valid CSI is not yet available
                    % ----------------------------------------------------------
                    if ~obj.CSI_Available
                        if ~isempty(allocatedUEsThisRBG)
                            if obj.DRLDebug
                                fprintf('[DRL] Layer %d, RBG %d: CSI not yet available, enforcing SU-MIMO (skipping UE %d)\n', layer, rbg, selectedUE);
                            end
                            continue;
                        end
                    end

                    % Check MaxNumUsersPerTTI constraint (total unique UEs in this TTI)
                    if ~ismember(selectedUE, totalUniqueUEsAllocated) && ...
                            numel(totalUniqueUEsAllocated) >= maxUsersPerTTI
                        if obj.DRLDebug
                            fprintf('[DRL] Layer %d, RBG %d: MaxNumUsersPerTTI reached (%d), skipping UE %d\n', ...
                                layer, rbg, maxUsersPerTTI, selectedUE);
                        end
                        continue;
                    end
                    
                    % fprintf("UE I1 VALUES: %s\n", jsonencode(ueI1Values));
                    
                    % ----------------------------------------------------------
                    % DRL MOD: PMI i1 Matching Constraint
                    % UEs co-scheduled on same RBG must have same i1 (wideband PMI)
                    % This ensures good beam orthogonality for MU-MIMO pairing
                    % ----------------------------------------------------------
                    if obj.EnablePairingConstraints && ~isempty(allocatedUEIndicesThisRBG)
                        currentUEI1 = ueI1Values{actionIndex};
                        if ~isempty(currentUEI1)
                            % Check i1 matching with all already allocated UEs on this RBG
                            i1Matched = true;
                            for allocIdx = 1:numel(allocatedUEIndicesThisRBG)
                                existingUEIdx = allocatedUEIndicesThisRBG(allocIdx);
                                existingUEI1 = ueI1Values{existingUEIdx};
                                if ~isempty(existingUEI1) && ~all(currentUEI1 == existingUEI1)
                                    i1Matched = false;
                                    break;
                                end
                            end
                            if ~i1Matched
                                if obj.DRLDebug
                                    fprintf('[DRL] Layer %d, RBG %d: UE %d i1 mismatch, skipping\n', ...
                                        layer, rbg, selectedUE);
                                end
                                continue;
                            end
                        end
                    end
                    
                    % ----------------------------------------------------------
                    % DRL MOD: Precoder Orthogonality Constraint (CSI-RS based)
                    % Check that paired UEs have semi-orthogonal precoders
                    % |W_new' * W_existing| / max < (1 - SemiOrthogonalityFactor)
                    % ----------------------------------------------------------
                    if obj.EnablePairingConstraints && ~isempty(allocatedUEIndicesThisRBG)
                        W_new = W{actionIndex};
                        if ~isempty(W_new)
                            isOrthogonal = true;
                            for allocIdx = 1:numel(allocatedUEIndicesThisRBG)
                                existingUEIdx = allocatedUEIndicesThisRBG(allocIdx);
                                W_existing = W{existingUEIdx};
                                if ~isempty(W_existing)
                                    % Compute correlation between precoders
                                    % Handle both 2D (wideband) and 3D (subband) precoders
                                    % W can be [numPorts x numLayers] or [numLayers x numPorts x numSubbands]
                                    try
                                        if ismatrix(W_new) && ismatrix(W_existing)
                                            % Wideband: flatten to column vectors and compute correlation
                                            w1 = W_new(:);
                                            w2 = W_existing(:);
                                            % Normalize and compute inner product
                                            w1 = w1 / (norm(w1) + 1e-10);
                                            w2 = w2 / (norm(w2) + 1e-10);
                                            maxCorr = abs(w1' * w2);
                                        else
                                            % Subband: W is 3D, take max correlation across subbands
                                            numSB = min(size(W_new, 3), size(W_existing, 3));
                                            maxCorr = 0;
                                            for sb = 1:numSB
                                                w1 = W_new(:,:,sb);
                                                w2 = W_existing(:,:,sb);
                                                w1 = w1(:) / (norm(w1(:)) + 1e-10);
                                                w2 = w2(:) / (norm(w2(:)) + 1e-10);
                                                corrSB = abs(w1' * w2);
                                                maxCorr = max(maxCorr, corrSB);
                                            end
                                        end
                                    catch
                                        % If computation fails, assume not orthogonal (conservative)
                                        maxCorr = 1;
                                    end
                                    
                                    % Check orthogonality threshold
                                    % maxCorr is already normalized to [0,1]
                                    if maxCorr > (1 - obj.SemiOrthogonalityFactor)
                                        isOrthogonal = false;
                                        if obj.DRLDebug
                                            fprintf('[DRL] Layer %d, RBG %d: UE %d not orthogonal to UE %d (corr=%.3f), skipping\n', ...
                                                layer, rbg, selectedUE, allocatedUEsThisRBG(allocIdx), maxCorr);
                                        end
                                        break;
                                    end
                                end
                            end
                            if ~isOrthogonal
                                continue;
                            end
                        end
                    end

                    % Allocate RBG to UE
                    allocMap(layer, rbg) = selectedUE;
                    ueRBGAllocation(selectedUE, rbg) = 1;
                    ueLayerCount(selectedUE, rbg) = ueLayerCount(selectedUE, rbg) + 1;
                    allocatedUEsThisRBG = [allocatedUEsThisRBG, selectedUE]; %#ok<AGROW>
                    allocatedUEIndicesThisRBG = [allocatedUEIndicesThisRBG, actionIndex]; %#ok<AGROW> % DRL MOD: Track index for i1
                    
                    % Track unique UEs for MaxNumUsersPerTTI
                    if ~ismember(selectedUE, totalUniqueUEsAllocated)
                        totalUniqueUEsAllocated = [totalUniqueUEsAllocated, selectedUE]; %#ok<AGROW>
                    end

                    if obj.DRLDebug
                        fprintf('[DRL] Layer %d, RBG %d: Allocated to UE %d\n', layer, rbg, selectedUE);
                    end
                end
                
                % ---------------------------------------------------------------
                % DRL MOD: Detailed per-RBG logging (when DRLDebug enabled)
                % ---------------------------------------------------------------
                if obj.DRLDebug && ~isempty(allocatedUEsThisRBG)
                    fprintf('[DRL Scheduler] RBG %d:\n', rbg);
                    
                    % Group UEs by their allocated layers
                    uniqueUEsOnRBG = unique(allocatedUEsThisRBG, 'stable');
                    for ueIdx = 1:numel(uniqueUEsOnRBG)
                        ue = uniqueUEsOnRBG(ueIdx);
                        ueLayersOnRBG = find(allocatedUEsThisRBG == ue) - 1;  % 0-indexed layers
                        numAllocatedLayers = ueLayerCount(ue, rbg);  % Layers allocated by scheduler
                        
                        % Get precoder info
                        ueEligibleIdx = find(eligibleUEs == ue, 1);
                        if ~isempty(ueEligibleIdx) && ueEligibleIdx <= numel(W)
                            W_ue = W{ueEligibleIdx};
                            if ~isempty(W_ue)
                                wSize = size(W_ue);
                                % W format: [rank x numPorts] or [rank x numPorts x numSubbands]
                                if numel(wSize) == 2
                                    wSizeStr = sprintf('[%d layers x %d ports]', wSize(1), wSize(2));
                                elseif numel(wSize) >= 3
                                    wSizeStr = sprintf('[%d layers x %d ports x %d subbands]', wSize(1), wSize(2), wSize(3));
                                else
                                    wSizeStr = sprintf('[%dx%d]', wSize);
                                end
                                fprintf('  UE%d: Layers=%d [%s], W%s\n', ...
                                    ue, numAllocatedLayers, strjoin(string(ueLayersOnRBG), ','), wSizeStr);
                            else
                                fprintf('  UE%d: Layers=%d [%s], W[empty]\n', ...
                                    ue, numAllocatedLayers, strjoin(string(ueLayersOnRBG), ','));
                            end
                        else
                            fprintf('  UE%d: Layers=%d [%s]\n', ...
                                ue, numAllocatedLayers, strjoin(string(ueLayersOnRBG), ','));
                        end
                    end
                    
                    % Check and log i1 match status
                    if obj.EnablePairingConstraints && numel(uniqueUEsOnRBG) > 1
                        allI1Match = true;
                        for i = 1:numel(uniqueUEsOnRBG)-1
                            ue1Idx = find(eligibleUEs == uniqueUEsOnRBG(i), 1);
                            ue2Idx = find(eligibleUEs == uniqueUEsOnRBG(i+1), 1);
                            if ~isempty(ue1Idx) && ~isempty(ue2Idx) && ...
                               ue1Idx <= numel(ueI1Values) && ue2Idx <= numel(ueI1Values)
                                i1_1 = ueI1Values{ue1Idx};
                                i1_2 = ueI1Values{ue2Idx};
                                if ~isempty(i1_1) && ~isempty(i1_2) && ~all(i1_1 == i1_2)
                                    allI1Match = false;
                                    break;
                                end
                            end
                        end
                        fprintf('  i1 match: %s\n', ternary(allI1Match, 'YES', 'NO'));
                    end
                    
                    % Check and log orthogonality status
                    if obj.EnablePairingConstraints && numel(uniqueUEsOnRBG) > 1
                        maxCorrRBG = 0;
                        for i = 1:numel(uniqueUEsOnRBG)
                            for j = i+1:numel(uniqueUEsOnRBG)
                                ue1Idx = find(eligibleUEs == uniqueUEsOnRBG(i), 1);
                                ue2Idx = find(eligibleUEs == uniqueUEsOnRBG(j), 1);
                                if ~isempty(ue1Idx) && ~isempty(ue2Idx) && ...
                                   ue1Idx <= numel(W) && ue2Idx <= numel(W)
                                    W1 = W{ue1Idx};
                                    W2 = W{ue2Idx};
                                    if ~isempty(W1) && ~isempty(W2)
                                        try
                                            if ismatrix(W1) && ismatrix(W2)
                                                w1 = W1(:) / (norm(W1(:)) + 1e-10);
                                                w2 = W2(:) / (norm(W2(:)) + 1e-10);
                                                corrVal = abs(w1' * w2);
                                            else
                                                numSB = min(size(W1, 3), size(W2, 3));
                                                sbIdx = min(rbg, numSB);
                                                w1 = W1(:,:,sbIdx);
                                                w2 = W2(:,:,sbIdx);
                                                w1 = w1(:) / (norm(w1(:)) + 1e-10);
                                                w2 = w2(:) / (norm(w2(:)) + 1e-10);
                                                corrVal = abs(w1' * w2);
                                            end
                                            maxCorrRBG = max(maxCorrRBG, corrVal);
                                        catch
                                            % Skip on error
                                        end
                                    end
                                end
                            end
                        end
                        orthThreshold = 1 - obj.SemiOrthogonalityFactor;
                        orthPass = maxCorrRBG <= orthThreshold;
                        fprintf('  Orthogonality check: %s (corr=%.2f %s threshold %.2f)\n', ...
                            ternary(orthPass, 'PASS', 'FAIL'), maxCorrRBG, ...
                            ternary(orthPass, '<', '>'), orthThreshold);
                    end
                end
            end
            
            fprintf("Official Allocation Map for NewTx: %s\n",mat2str(allocMap));
            % ---------------------------------------------------------------
            % Extract allotted UEs (UEs with at least one RBG)
            % ---------------------------------------------------------------
            allottedUEsMask = any(ueRBGAllocation, 2);
            allottedUEsList = find(allottedUEsMask);

            % Filter to only include eligible UEs (in case of RNTI mismatch)
            allottedUEsList = intersect(allottedUEsList, eligibleUEs, 'stable');
            numAllottedUEs = numel(allottedUEsList);

            % ---------------------------------------------------------------
            % Build output arrays
            % ---------------------------------------------------------------
            allottedUEs = zeros(numAllottedUEs, 1);
            freqAllocation = zeros(numAllottedUEs, numRBGs);
            mcsIndex = zeros(numAllottedUEs, 1);
            W = schedulerInput.W;  % Use precomputed precoding from parent

            for idx = 1:numAllottedUEs
                rnti = allottedUEsList(idx);
                allottedUEs(idx) = rnti;
                freqAllocation(idx, :) = ueRBGAllocation(rnti, :);

                % Find index in original eligibleUEs
                ueIdxInEligible = find(eligibleUEs == rnti, 1);

                % Calculate MCS based on average CQI of allotted RBs
                % (Reusing logic from parent runSchedulingStrategyRAT0)
                gNBCarrierIndex = 1;
                allottedRBs = convertRBGBitmapToRBs(obj, rnti, gNBCarrierIndex, freqAllocation(idx,:));

                % Get channel quality for this UE
                cqiRB = schedulerInput.channelQuality(ueIdxInEligible, allottedRBs+1);
                cqiSetRB = floor(mean(cqiRB, 2));
                % tuanpa44: hardcode cqi
                % cqiSetRB = max(cqiSetRB-5,1);
                % Calculate DL MCS value using parent method
                mcsIndex(idx) = selectMCSIndexDL(obj, cqiSetRB, rnti);
    
                % ----------------------------------------------------------
                % DRL MOD: MCS Backoff for MU-MIMO transmissions
                % Reduce MCS when sharing RBG with other UEs to account for
                % residual inter-user interference (CSI-RS based has imperfect
                % interference suppression)
                % ----------------------------------------------------------
                if obj.MU_MCSBackoff > 0
                    % Count max co-scheduled UEs on any RBG allocated to this UE
                    myRBGs = find(freqAllocation(idx, :));
                    maxCoScheduled = 0;
                    for rbgCheck = myRBGs
                        numUEsOnRBG = sum(ueRBGAllocation(:, rbgCheck) > 0);
                        maxCoScheduled = max(maxCoScheduled, numUEsOnRBG);
                    end
                    % Apply backoff: reduce MCS by backoff * (numCoScheduled - 1)
                    if maxCoScheduled > 1
                        backoffAmount = obj.MU_MCSBackoff * (maxCoScheduled - 1);
                        mcsIndex(idx) = max(0, mcsIndex(idx) - backoffAmount);
                        if obj.DRLDebug
                            fprintf('[DRL] UE %d: MCS backoff %d (co-scheduled with %d UEs)\n', ...
                                rnti, backoffAmount, maxCoScheduled);
                        end
                    end
                end
            end

            % ---------------------------------------------------------------
            % Store allocation for debugging/logging
            % ---------------------------------------------------------------
            obj.LastDRLAllocation.allottedUEs = allottedUEs;
            obj.LastDRLAllocation.freqAllocation = freqAllocation;
            obj.LastDRLAllocation.layerAssignment = ueLayerCount;
            obj.LastDRLAllocation.eligibleUEs = eligibleUEs;

            if obj.DRLDebug
                fprintf('[nrDRLScheduler] DRL allocation complete: %d UEs allocated\n', numAllottedUEs);
                for idx = 1:numAllottedUEs
                    fprintf('  UE %d: %d RBGs, MCS=%d\n', allottedUEs(idx), ...
                        sum(freqAllocation(idx,:)), mcsIndex(idx));
                end
            end
        end
        % ===================================================================
        % DRL MOD END - DRL scheduling strategy
        % ===================================================================
    end

    % ===================================================================
    % DRL MOD START - Helper methods for DRL scheduling
    % ===================================================================
    methods (Access = public)
        function info = getDRLSchedulerInfo(obj)
            %getDRLSchedulerInfo Get information about DRL scheduler state
            %   Returns a struct with current DRL scheduler configuration and state

            info = struct();
            info.EnableDRL = obj.EnableDRL;
            info.NumLayers = obj.NumLayers;
            info.SchedulingStrategy = obj.SchedulingStrategy;
            info.MaxUsersPerRBG = obj.MaxUsersPerRBG;
            info.DRLActionSize = size(obj.DRLAction);
            info.HasDRLAction = ~isempty(obj.DRLAction);
            info.LastDRLAllocation = obj.LastDRLAllocation;

            % Get RBG info from cell config
            if ~isempty(obj.CellConfig)
                info.NumResourceBlocks = obj.CellConfig(1).NumResourceBlocks;
                if obj.NumUEs > 0 && ~isempty(obj.UEContext)
                    carrierContext = obj.UEContext(1).ComponentCarrier(1);
                    info.RBGSize = carrierContext.RBGSize;
                    info.NumRBGs = carrierContext.NumRBGs;
                end
            end
        end

        function valid = validateDRLAction(obj, action, numEligibleUEs, numRBGs)
            %validateDRLAction Validate DRL action matrix
            %   valid = validateDRLAction(obj, action, numEligibleUEs, numRBGs)
            %   Returns true if action matrix has valid dimensions and values

            valid = true;

            % Check dimensions
            [nL, nR] = size(action);
            if nL < 1 || nR < 1
                valid = false;
                if obj.DRLDebug
                    warning('DRL action has invalid dimensions: [%d x %d]', nL, nR);
                end
                return;
            end

            if nR ~= numRBGs
                valid = false;
                if obj.DRLDebug
                    warning('DRL action RBG count mismatch: %d vs expected %d', nR, numRBGs);
                end
            end

            % Check value range
            minVal = min(action(:));
            maxVal = max(action(:));
            expectedMax = numEligibleUEs + 1;

            if minVal < 1 || maxVal > expectedMax
                valid = false;
                if obj.DRLDebug
                    warning('DRL action values out of range: [%d, %d], expected [1, %d]', ...
                        minVal, maxVal, expectedMax);
                end
            end
        end

        function mask = computeActionMask(obj, eligibleUEs, freqOccupancyBitmap, currentAllocation)
            %computeActionMask Compute action mask for valid DRL actions
            %   mask = computeActionMask(obj, eligibleUEs, freqOccupancyBitmap, currentAllocation)
            %
            %   Returns a logical mask [NumLayers x NumRBGs x (NumEligibleUEs+1)]
            %   where mask(l,m,a) = true means action a is valid at layer l, RBG m
            %
            %   Masking rules:
            %   1. RBGs occupied by retransmissions: only "no allocation" action valid
            %   2. UE already allocated on RBG: that UE's action is invalid
            %   3. MaxUsersPerRBG reached: all UE allocation actions invalid

            numEligibleUEs = numel(eligibleUEs);
            numRBGs = numel(freqOccupancyBitmap);
            numActions = numEligibleUEs + 1;  % +1 for "no allocation"

            % Initialize mask: all actions valid by default
            mask = true(obj.NumLayers, numRBGs, numActions);

            for rbg = 1:numRBGs
                % Rule 1: RBG occupied by retransmission
                if freqOccupancyBitmap(rbg)
                    % Only "no allocation" action is valid
                    mask(:, rbg, 1:numEligibleUEs) = false;
                    continue;
                end

                % Check current allocation on this RBG
                if ~isempty(currentAllocation)
                    allocatedUEsThisRBG = find(currentAllocation(:, rbg) > 0);

                    % Count allocated UEs
                    numAllocated = numel(allocatedUEsThisRBG);

                    % Rule 3: MaxUsersPerRBG check
                    if numAllocated >= obj.MaxUsersPerRBG
                        mask(:, rbg, 1:numEligibleUEs) = false;
                        continue;
                    end

                    % Rule 2: Mask already allocated UEs
                    for ue = allocatedUEsThisRBG'
                        ueIdx = find(eligibleUEs == ue, 1);
                        if ~isempty(ueIdx)
                            mask(:, rbg, ueIdx) = false;
                        end
                    end
                end
            end
        end

        function state = getSchedulerState(obj, eligibleUEs, freqOccupancyBitmap)
            %getSchedulerState Get current scheduler state for DRL agent
            %   state = getSchedulerState(obj, eligibleUEs, freqOccupancyBitmap)
            %
            %   Returns a struct containing:
            %   - eligibleUEs: RNTIs of eligible UEs
            %   - numEligibleUEs: count of eligible UEs
            %   - numRBGs: number of RBGs
            %   - freqOccupancyBitmap: current RBG occupancy (1=occupied)
            %   - channelQuality: CQI per UE per RB
            %   - bufferStatus: buffer status per UE
            %   - rank: selected rank per UE
            %   - precoders: precoding matrices per UE

            state = struct();
            state.eligibleUEs = eligibleUEs;
            state.numEligibleUEs = numel(eligibleUEs);
            state.numRBGs = numel(freqOccupancyBitmap);
            state.freqOccupancyBitmap = freqOccupancyBitmap;
            state.numLayers = obj.NumLayers;

            % Gather per-UE information
            numEligibleUEs = numel(eligibleUEs);
            state.channelQuality = zeros(numEligibleUEs, obj.CellConfig(1).NumResourceBlocks);
            state.bufferStatus = zeros(numEligibleUEs, 1);
            state.rank = ones(numEligibleUEs, 1);

            for i = 1:numEligibleUEs
                rnti = eligibleUEs(i);
                ueCtx = obj.UEContext(rnti);
                carrierCtx = ueCtx.ComponentCarrier(1);

                % Channel quality
                csiMeasurement = carrierCtx.CSIMeasurementDL;
                if ~isempty(csiMeasurement.CSIRS.CQI)
                    cqiVal = max(csiMeasurement.CSIRS.CQI(:));
                    state.channelQuality(i, :) = cqiVal;
                    state.rank(i) = csiMeasurement.CSIRS.RI;
                end

                % Buffer status
                state.bufferStatus(i) = ueCtx.BufferStatusDL;
            end
        end
    end
    % ===================================================================
    % DRL MOD END - Helper methods
    % ===================================================================

    % ===================================================================
    % TRAINING MODE - Layer-by-layer protocol (matches toy5g interface)
    % ===================================================================
    methods (Access = protected)

        function dlAssignments = scheduleWithTrainingProtocol(obj, timeFrequencyResource, schedulingInfo)
            %scheduleWithTrainingProtocol Raw-data training protocol.
            %   MATLAB gửi toàn bộ raw channel state 1 lần / TTI (TTI_RAW_DATA).
            %   Python tự tính OBS, mask, reward bên trong (giống toy5g).
            %   Python trả về toàn bộ allocation matrix sau khi xong (TTI_ALLOC).
            %
            %   Protocol (matches matlab_env_adapter_tuanpa44.py):
            %     MATLAB → Python : TTI_START  {tti, n_layers, n_rbg,
            %                                   buf[MaxUEs], avg_tp[MaxUEs](Mbps),
            %                                   ue_rank[MaxUEs], wb_cqi[MaxUEs],
            %                                   curr_mcs[MaxUEs],
            %                                   sub_cqi[MaxUEs x n_rbg],
            %                                   max_cross_corr[MaxUEs x MaxUEs x n_rbg]}
            %     Python → MATLAB : LAYER_ACT  {actions[n_layers x n_rbg]}
            %     MATLAB → Python : TTI_DONE   {metrics}

            eligibleUEs = schedulingInfo.EligibleUEs;
            if isempty(eligibleUEs)
                dlAssignments = struct([]);
                return;
            end

            carrierIndex  = 1;
            numRBs        = obj.CellConfig(carrierIndex).NumResourceBlocks;
            rbgSize       = obj.UEContext(eligibleUEs(1)).ComponentCarrier(carrierIndex).RBGSize;
            numRBGs       = ceil(numRBs / rbgSize);
            numEligible   = numel(eligibleUEs);
            numSubbandsFeat = numRBGs;   % 1 feature subband per RBG (toy5g-compatible)

            % ── Build feature matrix (same as before) ─────────────────────────
            [~, WprgMap, sb_cqi_all, rank_all, buffer_all, ~, wb_cqi_all, i1_all] = ...
                obj.buildTrainingFeatureMatrix(eligibleUEs, numRBGs, rbgSize, numSubbandsFeat, numRBs);
            
            % Instantaneous DL throughput from previous slot (Mbps)
            if isempty(obj.PrevRxBytes) || numel(obj.PrevRxBytes) ~= obj.MaxUEs
                obj.PrevRxBytes = zeros(obj.MaxUEs, 1);
            end
            inst_tp_all = 1e-6*ones(1, obj.MaxUEs); % [Mbps], indexed by RNTI
            slot_s = obj.CellConfig(1).SlotDuration * 1e-3; % Slot duration in seconds
            if ~isempty(obj.UEList)
                for ueIdx = 1:numel(obj.UEList)
                    ue   = obj.UEList(ueIdx);
                    rnti = ue.RNTI;
                    if rnti < 1 || rnti > obj.MaxUEs
                        continue;
                    end
                    try
                        rxBytes = statistics(ue).MAC.ReceivedBytes;
                    catch
                        continue;
                    end
                    deltaBytes = max(rxBytes - obj.PrevRxBytes(rnti), 0);
                    inst_tp_all(rnti) = deltaBytes * 8 / 1e6 / slot_s;
                    obj.PrevRxBytes(rnti) = rxBytes;
                end
            end

            % ── Per-UE MCS from scheduler logic (same selectMCSIndexDL path) ─
            curr_mcs_all = zeros(1, obj.MaxUEs);
            for rnti = 1:obj.MaxUEs

                cqiForMCS = wb_cqi_all(rnti);
                cqiForMCS = min(max(cqiForMCS, 0), 15);
                mcsForReport = min(max(selectMCSIndexDL(obj, cqiForMCS, rnti), 0), 28);

                carrierIndex = 1;
                mcsOffset = fix(obj.UEContext(rnti).ComponentCarrier(carrierIndex).MCSOffset(1));
                % mcsOffset = 0;
                effectiveMCS = min(max(mcsForReport - mcsOffset, 0), 28);
                curr_mcs_all(rnti) = effectiveMCS;
            end
            
            fprintf("wb cqi: %s\n", mat2str(wb_cqi_all));
            fprintf("effective MCS: %s\n", mat2str(curr_mcs_all));

            % ── Build per-UE precoder W (subband/PRG) for TTI_START kappa ──
            % Use same selection path as pairing constraints to align metrics.
            WsubMap = cell(1, obj.MaxUEs);
            for i = 1:numEligible
                rnti = eligibleUEs(i);
                if rnti > obj.MaxUEs, continue; end
                ueCtx = obj.UEContext(rnti);
                carrierCtx = ueCtx.ComponentCarrier(1);
                csiMeasurement = carrierCtx.CSIMeasurementDL;
                if isempty(carrierCtx.CSIRSConfiguration)
                    numCSIRSPorts = obj.CellConfig(1).NumTransmitAntennas;
                else
                    numCSIRSPorts = carrierCtx.CSIRSConfiguration.NumCSIRSPorts;
                end
                [~, WsubMap{rnti}] = selectRankAndPrecodingMatrixDL(obj, rnti, ...
                    csiMeasurement, numCSIRSPorts);
            end

            % ── Build pairwise cross-correlation [MaxUEs x MaxUEs x numRBGs] ──
            %    cross_corr_3d(ri, rj, m) = max kappa between subband/PRG W(ri), W(rj)
            %    using kappa (max column correlation), not flattened dot product.
            %    Python: max_cross_corr[i,j,m] = corr between UE i and UE j on RBG m
            %            where i,j are 0-based (= RNTI i+1, RNTI j+1). ✓
            cross_corr_3d = zeros(obj.MaxUEs, obj.MaxUEs, numRBGs);
            for i = 1:numEligible
                for j = 1:numEligible
                    if i == j, continue; end
                    ri = eligibleUEs(i);
                    rj = eligibleUEs(j);
                    if ri <= obj.MaxUEs && rj <= obj.MaxUEs && ...
                       ~isempty(WsubMap{ri}) && ~isempty(WsubMap{rj})
                        kappa_val = obj.computeMaxCorrSubband(WsubMap{ri}, WsubMap{rj});
                        cross_corr_3d(ri, rj, :) = kappa_val;
                    else
                        % No precoder available yet (CSI-RS not received) →
                        % conservative: treat as fully correlated (not orthogonal)
                        cross_corr_3d(ri, rj, :) = 1.0;
                    end
                end
            end

            % ----------------------------------------------------------
            % DRL MOD: Update CSI_Available flag
            % ----------------------------------------------------------
            fprintf("wb_cqi_all: %s\n",mat2str(wb_cqi_all));
            if ~obj.CSI_Available
                if sum(wb_cqi_all)~=obj.MaxUEs
                    obj.CSI_Available = true;
                end
            end

            % ── Increment TTI counter ─────────────────────────────────────────
            obj.TrainingTTICount = obj.TrainingTTICount + 1;
            % Track linear slot index for mapping to logs
            slotLinear = timeFrequencyResource.NFrame * obj.CellConfig(1).NumSlotsFrame + ...
                         timeFrequencyResource.NSlot;
            obj.TrainingTTISlotIndex(obj.TrainingTTICount) = slotLinear;

            % ── Store W used for TTI_START to sanity-check against pairing W ──
            obj.LastTTIStartWSubbandMap = WsubMap;
            obj.LastTTIStartEligibleUEs = eligibleUEs;
            obj.LastTTIStartTTI = obj.TrainingTTICount - 1;

            % ── Send TTI_START ────────────────────────────────────────────────
            %    Field names and array sizes match adapter's begin_tti() exactly.
            rbgOccupancyBitmap = timeFrequencyResource.FrequencyResource;
            fprintf("[PYTHON-TTI %d] occupied_rbgs %s\n",obj.TrainingTTICount - 1,mat2str(rbgOccupancyBitmap(:)'));
            fprintf("[PYTHON-TTI %d] eligible ues: %s\n",obj.TrainingTTICount - 1,mat2str(eligibleUEs));
            fprintf("[PYTHON_TTI %d] cross_corr_3d: %s\n",obj.TrainingTTICount - 1,mat2str(cross_corr_3d(:,:,1)));
            sb_cqi_cell     = obj.mat2NestedList(sb_cqi_all);
            cross_corr_cell = obj.mat3DToNestedList(cross_corr_3d);
            ue_i1_cell      = obj.mat2NestedList(i1_all);
            
            % MaxNumUsersPerTTI after reTx subtraction (same as DRL scheduler path)
            maxUsersPerTTI = schedulingInfo.MaxNumUsersTTI;
            if isempty(maxUsersPerTTI) || ~isfinite(maxUsersPerTTI)
                maxUsersPerTTI = obj.SchedulerConfig.MaxNumUsersPerTTI;
            end
            maxUsersPerTTI = max(0, floor(double(maxUsersPerTTI)));

            payload = struct( ...
                'type',           'TTI_START', ...
                'tti',            obj.TrainingTTICount - 1, ...
                'n_layers',       obj.NumLayers, ...
                'n_rbg',          numRBGs, ...
                'buf',            {num2cell(double(buffer_all))}, ...
                'avg_tp',         {num2cell(double(inst_tp_all))}, ...
                'ue_rank',        {num2cell(double(rank_all))}, ...
                'wb_cqi',         {num2cell(double(wb_cqi_all))}, ...
                'curr_mcs',       {num2cell(double(curr_mcs_all))}, ...
                'sub_cqi',        {sb_cqi_cell}, ...
                'max_cross_corr', {cross_corr_cell}, ...
                'orthogonality_corr_threshold', 1 - obj.SemiOrthogonalityFactor, ...
                'eligible_ues',   {num2cell(double(eligibleUEs))},...
                'occupied_rbgs', {num2cell(double(rbgOccupancyBitmap(:)'))},...
                'max_users_tti',  maxUsersPerTTI, ...
                'ue_i1',          {ue_i1_cell}, ...
                'csi_on',         obj.CSI_Available...
            );
            obj.drlSendJSON(payload);

            % ── Receive LAYER_ACT from Python ─────────────────────────────────
            %    adapter sends: {"type":"LAYER_ACT","actions":[[n_layers][n_rbg]]}
            %    jsondecode reads Python's [n_layers][n_rbg] JSON as MATLAB [n_layers x n_rbg].
            resp = obj.drlRecvJSON(obj.DRL_TimeoutSec * obj.NumLayers * 2);
            if ~strcmp(resp.type, 'LAYER_ACT')
                error('[nrDRLScheduler] Expected LAYER_ACT, got %s', resp.type);
            end

            actions_raw = resp.actions;   % [n_layers x n_rbg] MATLAB matrix
          
            % ── Map Python local-index → RNTI using selectedOrder ────────────
            drlAction = (numEligible + 1) * ones(obj.NumLayers, numRBGs);
            for l = 1:obj.NumLayers
                for m = 1:numRBGs
                    val = double(actions_raw(l, m));
                    if val >= 0 && val < obj.MaxUEs
                        rnti_mapped = val + 1;
                        ue_pos = find(eligibleUEs == rnti_mapped, 1);
                        if ~isempty(ue_pos)
                            drlAction(l, m) = ue_pos;   % 1-based index
                        end
                    end
                end
            end

            obj.DRLAction = drlAction;
           
            % ── Compute eval metrics and send TTI_DONE ────────────────────────
            % Convert drlAction [L x M] (1-based) to allocMatrix [M x L] (0-based) for metrics
            NOOP_LOCAL  = obj.MaxUEs;
            allocMatrix = drlAction.' - 1;
            allocMatrix(allocMatrix == numEligible) = NOOP_LOCAL; % Remap NOOP
            metrics = obj.computeTrainingEvalMetrics(allocMatrix, eligibleUEs, ...
                WprgMap, sb_cqi_all, numRBGs, rbgSize, 0, 0);
            obj.drlSendJSON(struct('type', 'TTI_DONE', 'metrics', metrics));

            dlAssignments = obj.scheduleNewTransmissionsDL_DRL(timeFrequencyResource, schedulingInfo);
        end

        % ──────────────────────────────────────────────────────────────────────
        function [exportMatrix, WprgMap, sbCQIMap, rankMap, buffers, avg_tp_bps, wb_cqi_raw, i1_all] = ...
                buildTrainingFeatureMatrix(obj, eligibleUEs, numRBGs, ~, numSubbandsFeat, ~)
            %buildTrainingFeatureMatrix Build per-UE feature matrix [MaxUEs x (5+2*numSubbandsFeat)].
            %   Mirrors SchedulerDRL.m exportMatrix computation.
            %   Extra outputs avg_tp_bps and wb_cqi_raw (indexed by RNTI) are used
            %   by the new TTI_RAW_DATA protocol so Python can compute obs/reward.

            carrierIndex = 1;
            featDim = 5 + 2 * numSubbandsFeat;
            exportMatrix = zeros(obj.MaxUEs, featDim);
            WprgMap  = cell(1, obj.MaxUEs);
            sbCQIMap = zeros(obj.MaxUEs, numSubbandsFeat);
            rankMap  = ones(1, obj.MaxUEs);
            buffers  = zeros(obj.MaxUEs, 1);
            avg_tp_bps  = zeros(obj.MaxUEs, 1);   % UEsServedDataRate DL [bps]
            wb_cqi_raw  = zeros(obj.MaxUEs, 1);   % wideband CQI [0-15]
            i1_all      = -1 * ones(obj.MaxUEs, 2); % Array to store i1 [i11, i12, i13] per UE

            CQI_TO_SE = [0.0000,0.1523,0.2344,0.3770,0.6016,0.8770,1.1758,1.4766, ...
                         1.9141,2.4063,2.7305,3.3223,3.9023,4.5234,5.1152,5.5547];
            SE_MAX = 5.5547;

            % Build list of previously-scheduled UEs (wideband) from last TTI
            scheduledUEsWideband = [];
            if ~isempty(obj.LastDRLAllocation) && ~isempty(obj.LastDRLAllocation.allottedUEs)
                scheduledUEsWideband = obj.LastDRLAllocation.allottedUEs(:).';
            end

            for k = 1:numel(eligibleUEs)
                rnti = eligibleUEs(k);
                if rnti > obj.MaxUEs, continue; end

                ueCtx     = obj.UEContext(rnti);
                carrierCtx = ueCtx.ComponentCarrier(carrierIndex);

                % Decode CSI
                [dlRank, wbCQI, sbCQI_feat, Wprg, i1_vals] = obj.decodeCSIForTraining( ...
                    carrierCtx, rnti, numSubbandsFeat);

                rankMap(rnti)     = dlRank;
                sbCQIMap(rnti, :) = sbCQI_feat;
                WprgMap{rnti}     = Wprg;
                buffers(rnti)     = ueCtx.BufferStatusDL;
                
                if ~isempty(i1_vals)
                    numI1 = min(2, length(i1_vals));
                    i1_all(rnti, 1:numI1) = i1_vals(1:numI1);
                end

                % Feature 1: norm avg DL throughput — ActualTputEMA (ground-truth
                % ReceivedBytes EMA); fallback to TrainingTputEMA if not yet available.
                if ~isempty(obj.ActualTputEMA) && numel(obj.ActualTputEMA) >= rnti
                    fR = min(obj.ActualTputEMA(rnti) / obj.MaxTput, 1.0);
                elseif ~isempty(obj.TrainingTputEMA) && numel(obj.TrainingTputEMA) >= rnti
                    fR = min(obj.TrainingTputEMA(rnti) / obj.MaxTput, 1.0);
                else
                    fR = 0;
                end

                % Feature 2: norm rank
                fH = min(double(dlRank) / 2.0, 1.0);

                % Feature 3: norm alloc ratio (filled after alloc, start with 0)
                fD = 0;

                % Feature 4: norm buffer
                fB = ueCtx.BufferStatusDL;   % raw; normalized later in buildToyObs

                % Feature 5: norm WB CQI via SE
                cqiIdxWB = min(max(round(wbCQI), 0), 15);
                fO = CQI_TO_SE(cqiIdxWB + 1) / SE_MAX;

                % Features 6: norm subband CQI [numSubbandsFeat]
                sbIdx = min(max(round(sbCQI_feat), 0), 15);
                fG    = CQI_TO_SE(sbIdx + 1) / SE_MAX;  % [1 x numSubbandsFeat]

                % Features 7: rho (cross-correlation) [numSubbandsFeat]
                if ~isempty(scheduledUEsWideband)
                    rho = obj.computeMaxKappaWideband(WprgMap, scheduledUEsWideband, rnti);
                else
                    rho = 0;
                end
                fRho_feat = rho * ones(1, numSubbandsFeat);

                exportMatrix(rnti, :) = [fR, fH, fD, fB, fO, fG, fRho_feat];

                % avg_tp_bps cho TTI_RAW_DATA: dùng ActualTputEMA (ground-truth),
                % fallback TrainingTputEMA nếu chưa có data.
                if ~isempty(obj.ActualTputEMA) && numel(obj.ActualTputEMA) >= rnti
                    avg_tp_bps(rnti) = obj.ActualTputEMA(rnti) * 1e6;   % Mbps → bps
                elseif ~isempty(obj.TrainingTputEMA) && numel(obj.TrainingTputEMA) >= rnti
                    avg_tp_bps(rnti) = obj.TrainingTputEMA(rnti) * 1e6;
                end
                wb_cqi_raw(rnti) = wbCQI;
            end
            fprintf("wb_cqi_raw: %s\n",mat2str(wb_cqi_raw));
        end

        % ──────────────────────────────────────────────────────────────────────
        function [dlRank, wbCQI, sbCQI_feat, Wprg, i1_vals] = decodeCSIForTraining( ...
                obj, carrierCtx, rnti, numSubbandsFeat)
            %decodeCSIForTraining Extract CSI from carrier context.

            dlRank = 1;  wbCQI = 0;
            sbCQI_feat = zeros(1, numSubbandsFeat);
            Wprg   = [];   % empty until a real CSI-RS precoder is received
            i1_vals = [];

            csi = struct();
            has_csirs = false;
            % Direct property access (isfield on handle objects is unreliable;
            % carrierCtx is nrComponentCarrierContext, a handle class, not a struct)
            try
                dlMeas = carrierCtx.CSIMeasurementDL;
                if isstruct(dlMeas) && isfield(dlMeas, 'CSIRS') && ~isempty(dlMeas.CSIRS)
                    csi = dlMeas.CSIRS;
                    has_csirs = true;
                end
            catch
                % CSIMeasurementDL not accessible (e.g. non-primary carrier)
            end

            % ── CSI diagnostic log (prints once per UE per TTI) ─────────────
            has_ri  = isfield(csi, 'RI')  && ~isempty(csi.RI);
            has_cqi = isfield(csi, 'CQI') && ~isempty(csi.CQI);
            has_w   = isfield(csi, 'W')   && ~isempty(csi.W);
            fprintf('[CSI] UE%2d: has_CSIRS=%d  has_RI=%d  has_CQI=%d  has_W=%d', ...
                rnti, has_csirs, has_ri, has_cqi, has_w);
            if has_cqi
                fprintf('  CQI(1)=%g  numel=%d', csi.CQI(1), numel(csi.CQI));
            end
            if has_ri
                fprintf('  RI=%g', csi.RI);
            end
            fprintf('\n');
            % ── end CSI diagnostic ───────────────────────────────────────────

            if isfield(csi, 'RI') && ~isempty(csi.RI)
                dlRank = csi.RI;
            end
            if isfield(csi, 'PMISet') && isfield(csi.PMISet, 'i1')
                i1_vals = csi.PMISet.i1;
            end
            if isfield(csi, 'CQI') && ~isempty(csi.CQI)
                raw = csi.CQI;
                if isscalar(raw)
                    wbCQI = raw;
                    sbCQI_feat = ones(1, numSubbandsFeat) * raw;
                else
                    wbCQI = mean(raw(:));
                    tmp = raw(:).';
                    if numel(tmp) ~= numSubbandsFeat
                        % Resample to numSubbandsFeat (nearest-neighbour)
                        xi  = linspace(1, numel(tmp), numSubbandsFeat);
                        tmp = interp1(1:numel(tmp), double(tmp), xi, 'nearest');
                    end
                    sbCQI_feat = tmp;
                end
            end
            wbCQI = min(max(round(wbCQI), 0), 15);
            sbCQI_feat = min(max(round(sbCQI_feat), 0), 15);

            if isfield(csi, 'W') && ~isempty(csi.W)
                Wraw = csi.W;
                if ismatrix(Wraw)
                    W2 = Wraw;
                    if size(W2, 1) ~= dlRank && size(W2, 2) == dlRank
                        W2 = W2.';
                    end
                    Pports = size(W2, 2);
                    Wprg   = complex(zeros(dlRank, Pports, 1));
                    Wprg(:,:,1) = W2;
                else
                    sz = size(Wraw);
                    if sz(2) == dlRank
                        WriP = permute(Wraw, [2 1 3]);
                    elseif sz(1) == dlRank
                        WriP = Wraw;
                    else
                        WriP = [];
                    end
                    if ~isempty(WriP)
                        Pports = size(WriP, 2);
                        Wprg   = complex(zeros(dlRank, Pports, 1));
                        Wprg(:,:,1) = mean(WriP, 3);
                    end
                end
            end
        end

        % ──────────────────────────────────────────────────────────────────────
        function obs = buildToyObs(obj, exportMatrix, allocMatrix, layer, WprgMap, ...
                                   eligibleUEs, numRBGs, ~, buffers)
            %buildToyObs Build obs vector [obs_dim] matching toy5g _build_obs.
            %   obs_dim = (5 + 2*numSubbandsFeat) * MaxUEs

            NOOP_LOCAL = obj.MaxUEs;

            % ── Per-UE base features ─────────────────────────────────────────
            % Columns: [avg_tp, rank, alloc_rbgs, buffer, wb_cqi]
            base_feats = zeros(obj.MaxUEs, 5);

            % Count previous-layer alloc per UE on this TTI
            alloc_counts = zeros(obj.MaxUEs, 1);
            if layer > 0
                prev_alloc = allocMatrix(:, 1:layer);  % [numRBGs x layer]
                for u_local = 0:obj.MaxUEs-1
                    alloc_counts(u_local+1) = sum(prev_alloc(:) == u_local);
                end
            end
            norm_alloc_rbgs = min(alloc_counts / max(numRBGs, 1), 1.0);

            % Buffer normalization across eligible UEs
            max_buf = max(max(buffers(eligibleUEs)), 1e-9);

            for k = 1:numel(eligibleUEs)
                rnti    = eligibleUEs(k);
                u_local = k - 1;   % 0-based local index
                if rnti > obj.MaxUEs, continue; end

                row = exportMatrix(rnti, :);
                fR = row(1);                          % norm avg tp
                fH = row(2);                          % norm rank
                fD = norm_alloc_rbgs(rnti);           % norm alloc rbgs (current TTI)
                fB = buffers(rnti) / max_buf;         % norm buffer (relative)
                fO = row(5);                          % norm WB CQI

                base_feats(u_local+1, :) = [fR, fH, fD, fB, fO];
            end

            % ── Subband CQI features [MaxUEs x numSubbandsFeat] ─────────────
            sb_cqi_feats = exportMatrix(:, 6 : 5+numRBGs);  % [MaxUEs x M]

            % ── Max cross-correlation features [MaxUEs x numSubbandsFeat] ───
            max_corr_feats = zeros(obj.MaxUEs, numRBGs);
            if layer > 0
                cross_corr_tensor = obj.buildCrossCorr(allocMatrix, layer, ...
                    eligibleUEs, WprgMap, numRBGs, NOOP_LOCAL);
                max_corr_feats = cross_corr_tensor;  % [MaxUEs x numRBGs]
            end

            % ── Concatenate and flatten ──────────────────────────────────────
            % per UE: [fR, fH, fD, fB, fO, sb_cqi[M], max_corr[M]]
            ue_mat = [base_feats, sb_cqi_feats, max_corr_feats];  % [MaxUEs x (5+2M)]
            obs = ue_mat(:);    % flatten column-major → [obs_dim] = (5+2M)*MaxUEs
            obs = double(obs);
        end

        % ──────────────────────────────────────────────────────────────────────
        function cross_corr = buildCrossCorr(obj, allocMatrix, layer, ...
                eligibleUEs, WprgMap, numRBGs, NOOP_LOCAL)
            %buildCrossCorr Compute max kappa with previously-scheduled UEs per RBG.
            %   Returns [MaxUEs x numRBGs] where entry (u_local, m) =
            %   max kappa between UE u_local and any UE already on RBG m in prev layers.

            cross_corr = zeros(obj.MaxUEs, numRBGs);

            for m = 1:numRBGs
                % UEs scheduled on RBG m in previous layers (0-based local idx)
                sched_local = [];
                for l_prev = 0:layer-1
                    a = allocMatrix(m, l_prev+1);
                    if a ~= NOOP_LOCAL && a < numel(eligibleUEs)
                        sched_local(end+1) = a; %#ok<AGROW>
                    end
                end
                sched_local = unique(sched_local);

                if isempty(sched_local), continue; end

                sched_rntis = eligibleUEs(sched_local + 1);  % 1-based index

                for k = 1:numel(eligibleUEs)
                    u_local = k - 1;
                    rnti_u  = eligibleUEs(k);
                    if rnti_u > obj.MaxUEs || isempty(WprgMap{rnti_u}), continue; end

                    max_kappa = 0;
                    for v_rnti = sched_rntis
                        if v_rnti == rnti_u, continue; end
                        if v_rnti > obj.MaxUEs || isempty(WprgMap{v_rnti}), continue; end
                        k_val = obj.computeKappaWideband(WprgMap{rnti_u}, WprgMap{v_rnti});
                        max_kappa = max(max_kappa, k_val);
                    end
                    cross_corr(u_local+1, m) = max_kappa;
                end
            end
        end

        % ──────────────────────────────────────────────────────────────────────
        function masks = buildToyMasks(obj, allocMatrix, layer, numRBGs, NOOP_LOCAL, buffers, rankMap)
            %buildToyMasks Build masks [numRBGs x (MaxUEs+1)] matching toy5g _build_masks.
            %   Constraints: buffer>0, rank, continuity. NOOP always valid.

            act_dim = obj.MaxUEs + 1;   % MaxUEs local indices + 1 NOOP
            masks = true(numRBGs, act_dim);

            for m = 1:numRBGs
                for u_local = 0:obj.MaxUEs-1
                    % Buffer constraint
                    if u_local < obj.MaxUEs
                        % Map local index to rnti: u_local → eligibleUEs(u_local+1)
                        % We use buffers indexed by rnti; but here we use buffers(u_local+1)
                        % because buildTrainingFeatureMatrix stores by rnti (1-based).
                        % For simplicity, use buffers indexed 1-based by local position.
                        buf_val = buffers(u_local + 1);
                    else
                        buf_val = 0;
                    end

                    if buf_val <= 0
                        masks(m, u_local+1) = false;
                        continue;
                    end

                    if layer > 0
                        prev_alloc = allocMatrix(:, 1:layer);   % [numRBGs x layer]

                        % Count how many previous layers have UE u_local on RBG m
                        count_u_m = sum(prev_alloc(m, :) == u_local);

                        % Rank constraint: count < rank[u_local]
                        rank_u = rankMap(u_local + 1);
                        if count_u_m >= rank_u
                            masks(m, u_local+1) = false;
                            continue;
                        end

                        % Continuity constraint:
                        % If ever_seen (count_u_m > 0), UE must be in the immediately previous layer
                        ever_seen = count_u_m > 0;
                        if ever_seen
                            in_prev_layer = (allocMatrix(m, layer) == u_local);
                            if ~in_prev_layer
                                masks(m, u_local+1) = false;
                                continue;
                            end
                        end
                    end
                end
                % NOOP always valid (last column)
                masks(m, obj.MaxUEs+1) = true;
            end
        end

        % ──────────────────────────────────────────────────────────────────────
        function rewards = computePerRBGRewards(obj, allocMatrix, layer, masks, ...
                eligibleUEs, WprgMap, buffers, numRBGs)
            %computePerRBGRewards Port of toy5g new_reward_compute.
            %   Returns [numRBGs x 1] rewards ∈ [-1, 1].

            rewards = zeros(numRBGs, 1);
            NOOP_LOCAL = obj.MaxUEs;

            for m = 1:numRBGs
                % Previous allocation on this RBG
                prev_set = [];
                if layer > 0
                    for l_prev = 0:layer-1
                        a = allocMatrix(m, l_prev+1);
                        if a ~= NOOP_LOCAL && a < numel(eligibleUEs)
                            prev_set(end+1) = a; %#ok<AGROW>
                        end
                    end
                end

                T_prev = obj.computeSetTput(prev_set, m, eligibleUEs, WprgMap, buffers);

                chosen = allocMatrix(m, layer+1);  % 0-based local index or NOOP

                % Compute marginal raw reward for all valid UEs
                raw_all = zeros(obj.MaxUEs, 1);
                for u_local = 0:obj.MaxUEs-1
                    if ~masks(m, u_local+1), continue; end
                    buf_u = buffers(u_local + 1);
                    if buf_u <= 0, continue; end

                    curr_set = [prev_set, u_local];
                    T_cur    = obj.computeSetTput(curr_set, m, eligibleUEs, WprgMap, buffers);
                    raw_all(u_local+1) = (T_cur - T_prev) / max(buf_u, 1e-9);
                end

                max_raw = max(raw_all);

                if max_raw > 0
                    if chosen == NOOP_LOCAL
                        rewards(m) = 0.0;
                    else
                        rewards(m) = min(max(raw_all(chosen+1) / max_raw, -1.0), 1.0);
                    end
                elseif max_raw < 0
                    rewards(m) = (chosen == NOOP_LOCAL);  % 1 if NOOP, 0 otherwise
                else
                    rewards(m) = 0.0;
                end
            end
        end

        % ──────────────────────────────────────────────────────────────────────
        function tput = computeSetTput(obj, alloc_set, m, eligibleUEs, WprgMap, buffers)
            %computeSetTput Throughput of UE set on RBG m (port of toy5g compute_set_tput).
            %   penalty = 1 - max_kappa_between_pairs
            %   tput = sum(TBS_u_m) * penalty

            if isempty(alloc_set)
                tput = 0.0;
                return;
            end

            % Max kappa between any pair
            max_kappa = 0.0;
            for i = 1:numel(alloc_set)-1
                u = alloc_set(i);
                if u >= numel(eligibleUEs) || u < 0, continue; end
                rnti_u = eligibleUEs(u+1);
                if rnti_u > obj.MaxUEs || isempty(WprgMap{rnti_u}), continue; end
                for j = i+1:numel(alloc_set)
                    v = alloc_set(j);
                    if v >= numel(eligibleUEs) || v < 0, continue; end
                    rnti_v = eligibleUEs(v+1);
                    if rnti_v > obj.MaxUEs || isempty(WprgMap{rnti_v}), continue; end
                    k_val = obj.computeKappaWideband(WprgMap{rnti_u}, WprgMap{rnti_v});
                    max_kappa = max(max_kappa, k_val);
                end
            end

            penalty = 1.0 - max_kappa;

            % Sum TBS for all UEs in set
            tput_sum = 0.0;
            for i = 1:numel(alloc_set)
                u = alloc_set(i);
                if u < 0 || u >= numel(eligibleUEs), continue; end
                rnti_u = eligibleUEs(u+1);
                buf_u = buffers(rnti_u);
                if buf_u <= 0, continue; end
                tbs = obj.computeTBSMbps(rnti_u, m);
                tput_sum = tput_sum + tbs;
            end

            tput = tput_sum * penalty;
        end

        % ──────────────────────────────────────────────────────────────────────
        function tbs_mbps = computeTBSMbps(obj, rnti, m) %#ok<INUSD>
            %computeTBSMbps 3GPP-accurate TBS throughput [Mbps] for UE rnti on one RBG.
            %   Uses nrTBS (3GPP TS 38.214 Sec 5.1.3.2) with:
            %     - Wideband CQI -> MCS (MCS Table 1, 3GPP TS 38.214 Table 5.1.3.1-1)
            %     - RI from CSI report for number of spatial layers (nLayers)
            %     - Correct slot duration from CellConfig.SlotDuration (ms)
            %     - NREPerPRB = 156: 168 total - 12 DMRS RE
            %       (DMRS Type A, pos 2, single symbol, 2 CDM groups)

            MCS_TABLE = [...
                2, 120; 2, 157; 2, 193; 2, 251; 2, 308; 2, 379;
                4, 449; 4, 526; 4, 602; 4, 679; 6, 340; 6, 378;
                6, 434; 6, 490; 6, 553; 6, 616; 6, 658; 8, 438;
                8, 466; 8, 517; 8, 567; 8, 616; 8, 666; 8, 719;
                8, 772; 8, 822; 8, 873; 8, 910; 8, 948];

            CQI_TO_MCS = [0,0,1,3,5,7,9,11,13,15,18,20,22,24,26,28];

            % Get WB CQI and RI (rank) from UE context
            wbCQI   = 7;  % fallback CQI
            nLayers = 1;  % fallback rank
            if rnti >= 1 && rnti <= numel(obj.UEContext) && ~isempty(obj.UEContext(rnti))
                carrierCtx_ = obj.UEContext(rnti).ComponentCarrier(1);
                if isfield(carrierCtx_, 'CSIMeasurementDL') && ...
                   isfield(carrierCtx_.CSIMeasurementDL, 'CSIRS')
                    csi = carrierCtx_.CSIMeasurementDL.CSIRS;
                    if isfield(csi, 'CQI') && ~isempty(csi.CQI)
                        wbCQI = min(max(round(mean(csi.CQI(:))), 0), 15);
                    end
                    if isfield(csi, 'RI') && ~isempty(csi.RI)
                        nLayers = max(1, min(double(csi.RI), 8));
                    end
                end
            end

            mcs_idx = min(max(CQI_TO_MCS(wbCQI + 1), 0), 28);
            Qm = MCS_TABLE(mcs_idx+1, 1);
            R  = MCS_TABLE(mcs_idx+1, 2) / 1024.0;

            % Map bits-per-symbol to modulation string required by nrTBS
            switch Qm
                case 2,    modStr = 'QPSK';
                case 4,    modStr = '16QAM';
                case 6,    modStr = '64QAM';
                case 8,    modStr = '256QAM';
                otherwise, modStr = 'QPSK';
            end

            % NREPerPRB = 156 (12*14 - 12 DMRS RE per PRB, see above)
            NREPerPRB = 156;
            n_prb     = obj.TBSPrbSize;

            % 3GPP TS 38.214 Sec 5.1.3.2 TBS via MATLAB nrTBS
            tbs_bits = nrTBS(modStr, nLayers, n_prb, NREPerPRB, R);
            if tbs_bits <= 0
                tbs_mbps = 0; return;
            end

            % Slot duration in seconds from cell config (SlotDuration is in ms)
            % e.g. 0.5 ms for SCS = 30 kHz, 1 ms for SCS = 15 kHz
            slot_s   = obj.CellConfig(1).SlotDuration * 1e-3;
            tbs_mbps = tbs_bits / 1e6 / slot_s;
        end

        % ──────────────────────────────────────────────────────────────────────
        function kappa = computeKappaWideband(~, W1, W2)
            %computeKappaWideband Wideband precoder correlation.
            if isempty(W1) || isempty(W2), kappa = 0; return; end
            if ndims(W1) == 3, P1 = W1(:,:,1).'; else, P1 = W1.'; end
            if ndims(W2) == 3, P2 = W2(:,:,1).'; else, P2 = W2.'; end
            P1n = P1 ./ max(vecnorm(P1, 2, 1), 1e-10);
            P2n = P2 ./ max(vecnorm(P2, 2, 1), 1e-10);
            C   = P1n' * P2n;
            kappa = max(abs(C(:)));
        end

        % ──────────────────────────────────────────────────────────────────────
        function maxCorr = computeMaxCorrSubband(obj, W1, W2)
            %computeMaxCorrSubband Max kappa across subbands/PRGs.
            % Uses kappa (max column correlation) per subband; no flatten-dot metric.
            if isempty(W1) || isempty(W2)
                maxCorr = 1;  % conservative when precoder is missing
                return;
            end
            try
                if ismatrix(W1) && ismatrix(W2)
                    maxCorr = obj.computeKappaWideband(W1, W2);
                else
                    numSB = min(size(W1, 3), size(W2, 3));
                    maxCorr = 0;
                    for sb = 1:numSB
                        w1 = W1(:,:,sb);
                        w2 = W2(:,:,sb);
                        kappa_sb = obj.computeKappaWideband(w1, w2);
                        maxCorr = max(maxCorr, kappa_sb);
                    end
                end
            catch
                maxCorr = 1;  % conservative on error
            end
        end

        % ──────────────────────────────────────────────────────────────────────
        function [isSame, maxAbsDiff] = comparePrecoderMatrices(~, W1, W2)
            %comparePrecoderMatrices Check if two precoders are identical (within tol).
            if isempty(W1) && isempty(W2)
                isSame = true;
                maxAbsDiff = 0;
                return;
            end
            if ~isequal(size(W1), size(W2))
                isSame = false;
                maxAbsDiff = inf;
                return;
            end
            if isempty(W1) || isempty(W2)
                isSame = false;
                maxAbsDiff = inf;
                return;
            end
            diff = W1 - W2;
            maxAbsDiff = max(abs(diff(:)));
            isSame = maxAbsDiff < 1e-9;
        end

        % ──────────────────────────────────────────────────────────────────────
        function rho = computeMaxKappaWideband(obj, WprgMap, scheduledUEs, rnti)
            %computeMaxKappaWideband Max kappa(rnti, u) for u in scheduledUEs.
            rho = 0;
            if rnti > obj.MaxUEs || isempty(WprgMap{rnti}), return; end
            for k = 1:numel(scheduledUEs)
                c = scheduledUEs(k);
                if c == rnti || c > obj.MaxUEs || isempty(WprgMap{c}), continue; end
                rho = max(rho, obj.computeKappaWideband(WprgMap{rnti}, WprgMap{c}));
            end
        end

        % ──────────────────────────────────────────────────────────────────────
        function metrics = computeTrainingEvalMetrics(obj, allocMatrix, eligibleUEs, ...
                ~, ~, numRBGs, ~, invalid_count, total_count)
            %computeTrainingEvalMetrics Full per-TTI eval metrics for TTI_DONE.
            %
            %   Throughput:
            %     total_cell_tput   – sum of scheduled rate[u,m] over all (m,l) [Mbps]
            %     total_ue_tput     – [MaxUEs x 1] per-UE tput this TTI [Mbps]
            %   Fairness:
            %     alloc_counts      – [MaxUEs x 1] num (m,l) branches each UE got
            %     jain_throughput   – Jain fairness index on total_ue_tput (eligible UEs)
            %   PF utility:
            %     pf_utility        – sum_u log(tput_u + eps), eligible UEs only
            %   Sanity:
            %     invalid_action_rate – fraction of actions not in mask
            %     no_schedule_rate    – fraction of NOOP actions
            %     avg_layers_per_rbg  – avg #scheduled layers per RBG

            NOOP_LOCAL  = obj.MaxUEs;
            eps_val     = 1e-9;

            alloc_cnts  = zeros(obj.MaxUEs, 1);   % allocation branch counts per UE
            total_decisions = 0;
            noop_count      = 0;
            layers_sum      = 0;

            for m = 1:numRBGs
                % ── Collect scheduled locals and count NOOPs ──────────────────
                scheduled_locals = [];
                for l = 0:obj.NumLayers-1
                    a = allocMatrix(m, l+1);
                    total_decisions = total_decisions + 1;
                    if a == NOOP_LOCAL
                        noop_count = noop_count + 1;
                    else
                        scheduled_locals(end+1) = a; %#ok<AGROW>
                        % Count per-UE (m,l) allocation branches
                        if a < numel(eligibleUEs)
                            rnti_a = eligibleUEs(a+1);
                            if rnti_a <= obj.MaxUEs
                                alloc_cnts(rnti_a) = alloc_cnts(rnti_a) + 1;
                            end
                        end
                    end
                end
                layers_sum = layers_sum + numel(scheduled_locals);

                unique_locals = unique(scheduled_locals);
                if isempty(unique_locals), continue; end

            end

            % ── Throughput EMA: mirrors nrScheduler.updateUEsServedDataRate ──────
            % Maintains TrainingTputEMA[MaxUEs] in Mbps with the same PFS formula:
            %   decay  : EMA *= (1 - 1/W)           for ALL UEs each TTI
            %   update : EMA += (1/W) * instant_Mbps for scheduled UEs
            % instantaneous = tbs_bits / slot_s / 1e6  [Mbps]   (same as nrScheduler)
            % W = SchedulerConfig.PFSWindowSize (default 20)
            W = 1;
            try
                W = double(obj.SchedulerConfig.PFSWindowSize(1));
            catch
            end
            W = max(W, 1);   % ensure positive scalar

            % Lazy initialise EMA vector
            if isempty(obj.TrainingTputEMA) || numel(obj.TrainingTputEMA) ~= obj.MaxUEs
                obj.TrainingTputEMA = zeros(obj.MaxUEs, 1);
            end

            % Step 1 – decay all UEs
            obj.TrainingTputEMA = (1 - 1/W) * obj.TrainingTputEMA;

            % Step 2 – update scheduled UEs with instantaneous TBS-based rate
            for m = 1:numRBGs
                unique_loc = unique(allocMatrix(m, :));
                unique_loc = unique_loc(unique_loc ~= NOOP_LOCAL);
                for u_local = unique_loc
                    if u_local >= numel(eligibleUEs), continue; end
                    rnti_u = eligibleUEs(u_local + 1);
                    if rnti_u > obj.MaxUEs, continue; end
                    tbs_mbps = obj.computeTBSMbps(rnti_u, m);   % uses CQI+nLayers
                    obj.TrainingTputEMA(rnti_u) = obj.TrainingTputEMA(rnti_u) + ...
                        (1/W) * tbs_mbps;
                end
            end

            % ── ActualTputEMA: ground-truth DL received bytes per UE ─────────────
            % Dùng cumulative ReceivedBytes / elapsed_time thay vì delta per-TTI.
            % Lý do: ReceivedBytes chỉ tăng khi HARQ ACK về (không phải mỗi TTI)
            % nên delta per-TTI = 0 hầu hết thời gian → EMA-decay làm mất signal.
            % cumBytes/elapsed là running average ổn định, không bị bursty.
            if ~isempty(obj.UEList)
                slot_s = obj.CellConfig(1).SlotDuration * 1e-3 * 2;
                % obj.TrainingTTICount = obj.TrainingTTICount + 1;
                elapsed_s = max(obj.TrainingTTICount * slot_s, slot_s);  % tránh /0

                if isempty(obj.ActualTputEMA) || numel(obj.ActualTputEMA) ~= obj.MaxUEs
                    obj.ActualTputEMA = zeros(obj.MaxUEs, 1);
                end

                for ueIdx = 1:numel(obj.UEList)
                    ue   = obj.UEList(ueIdx);
                    rnti = ue.RNTI;
                    if rnti < 1 || rnti > obj.MaxUEs, continue; end
                    try
                        rxBytes = statistics(ue).MAC.ReceivedBytes;
                    catch
                        continue;
                    end
                    % Cumulative average: tổng bytes nhận được / thời gian đã chạy [Mbps]
                    obj.ActualTputEMA(rnti) = rxBytes * 8 / 1e6 / elapsed_s;
                end
            end

            % Step 3 – use ActualTputEMA (ground-truth) if available, else TBS EMA
            if ~isempty(obj.ActualTputEMA) && numel(obj.ActualTputEMA) == obj.MaxUEs
                tput_ue = obj.ActualTputEMA;
            else
                tput_ue = obj.TrainingTputEMA;
            end

            % ── Aggregate metrics ─────────────────────────────────────────────
            total_cell_tput = sum(tput_ue);

            % Restrict fairness / PF to eligible UEs (avoid log(0) from non-eligible)
            tput_eligible = tput_ue(eligibleUEs);
            nE = numel(tput_eligible);

            % Jain fairness index on eligible-UE throughputs
            s1 = sum(tput_eligible)^2;
            s2 = nE * sum(tput_eligible.^2) + eps_val;
            jain_throughput = s1 / s2;

            % PF utility: sum_u log(tput_u + eps), eligible UEs only
            pf_utility = sum(log(tput_eligible + eps_val));

            % Sanity rates
            avg_layers_per_rbg  = layers_sum / max(numRBGs, 1);
            no_schedule_rate    = noop_count  / max(total_decisions, 1);
            invalid_action_rate = invalid_count / max(total_count, 1);
            % Log tất cả MaxUEs; UE không eligible TTI này đánh dấu *
            if islogical(eligibleUEs)
                elig_set = find(eligibleUEs);
            else
                elig_set = eligibleUEs(:).';
            end
            parts = cell(1, obj.MaxUEs);
            for r = 1:obj.MaxUEs
                if ismember(r, elig_set)
                    parts{r} = sprintf("%d:%.2f", r, tput_ue(r));
                else
                    parts{r} = sprintf("%d*:%.2f", r, tput_ue(r));
                end
            end
            % fprintf("[DRLScheduler] ActualCellTput=%.2f Mbps (TBS-EMA=%.2f) | %s\n", ...
            %     total_cell_tput, sum(obj.TrainingTputEMA), ...
            %     strjoin(string(parts), "  "));
            metrics = struct( ...
                'total_cell_tput',    total_cell_tput, ...
                'total_ue_tput',      {num2cell(tput_ue.')}, ...
                'alloc_counts',       {num2cell(alloc_cnts.')}, ...
                'jain_throughput',    jain_throughput, ...
                'pf_utility',         pf_utility, ...
                'invalid_action_rate',invalid_action_rate, ...
                'no_schedule_rate',   no_schedule_rate, ...
                'avg_layers_per_rbg', avg_layers_per_rbg ...
            );
        end

        % ──────────────────────────────────────────────────────────────────────
        function list = boolMatToNestedList(~, mask)
            %boolMatToNestedList Convert [M x A] logical matrix to nested cell/list.
            %   Needed for jsonencode to produce [[...],[...]] format.
            [M, ~] = size(mask);
            list = cell(M, 1);
            for m = 1:M
                list{m} = num2cell(double(mask(m, :)));
            end
        end

        function list = mat2NestedList(~, mat)
            %mat2NestedList Convert [M x N] numeric matrix to nested cell array.
            %   Produces [[row1], [row2], ...] in JSON (row-major).
            [M, ~] = size(mat);
            list = cell(M, 1);
            for m = 1:M
                list{m} = num2cell(double(mat(m, :)));
            end
        end

        function list = mat3DToNestedList(~, arr)
            %mat3DToNestedList Convert [A x B x C] numeric array to 3-level nested cell.
            %   Produces [[[...],[...]], ...] in JSON with shape [A][B][C].
            %   Used to serialize cross_corr_3d [MaxUEs x MaxUEs x numRBGs] so that
            %   Python's torch.as_tensor(msg["max_cross_corr"]) gets shape [A, B, C].
            [A, B, C] = size(arr);
            list = cell(A, 1);
            for a = 1:A
                row = cell(B, 1);
                for b = 1:B
                    row{b} = num2cell(double(reshape(arr(a, b, :), 1, C)));
                end
                list{a} = row;
            end
        end

    end  % methods (Access = protected) - TRAINING MODE
    % ===================================================================
    % TRAINING MODE END
    % ===================================================================
    % ===================================================================
    % DRL MOD END - Helper methods
    % ===================================================================
end
