classdef (Abstract) nrPDCCHConfigBase < comm.internal.ConfigBase
  % This is an internal class that may change any time. It contains common
  % properties between nrWavegenPDCCHConfig and nrPDCCHConfig.
  
  % Copyright 2019-2022 The MathWorks, Inc.
   
%#codegen

    % Public properties
    properties (SetAccess = 'public')
        %DMRSScramblingID PDCCH DM-RS scrambling identity
        %   Specify the PDCCH DM-RS scrambling identity as a scalar
        %   nonnegative integer (0...65535), if pdcch-DMRS-ScramblingID is
        %   configured, or as [] for the physical layer cell identity,
        %   NCellID, to be used instead. The default is 2.
        DMRSScramblingID = 2;

        %AggregationLevel PDCCH aggregation level
        %   Specify the PDCCH aggregation level as a positive integer from
        %   the set of {1, 2, 4, 8, 16}. The default is 8.
        AggregationLevel (1,1) {mustBeMember(AggregationLevel, [1 2 4 8 16])} = 8;

        %AllocatedCandidate Candidate used for the PDCCH instance
        %   Specify the candidate used for the PDCCH instance as a scalar
        %   positive index (1-based), from the set of candidates specified by the
        %   NumCandidates property of <a href="matlab:help('nrSearchSpaceConfig.NumCandidates')"
        %   >nrSearchSpaceConfig</a> at the specified
        %   AggregationLevel. The default is 1.
        AllocatedCandidate (1,1) {mustBeNumeric, mustBeInteger, mustBePositive, mustBeLessThanOrEqual(AllocatedCandidate, 8)} = 1;
        
        %CCEOffset Explicit CCE offset used for the PDCCH instance
        %   Specify the index of first CCE for the PDCCH instance.
        %   When non-empty it overrides the AllocatedCandidate property.
        %   The default is [].
        CCEOffset = [];
    end

    methods
        function obj = nrPDCCHConfigBase(varargin)
            obj@comm.internal.ConfigBase(varargin{:});
            if ~isempty(coder.target) % Allow to be var-size in codegen
                obj.DMRSScramblingID = nr5g.internal.parseProp('DMRSScramblingID', ...
                    2,varargin{:});
            end
        end

        % PDCCH DMRS scrambling identity (0...65535) (nID) or [] for 
        % NCellID (0...1007)
        function obj = set.DMRSScramblingID(obj,val)            
            prop = 'DMRSScramblingID';
            
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},...
                    {'scalar','integer','nonnegative','<=', 65535},...
                    [class(obj) '.' prop],prop);
            end

            obj.DMRSScramblingID = temp;
        end

        % Explicit CCE offset of candidate
        function obj = set.CCEOffset(obj,val)
            prop = 'CCEOffset';
           
            % To allow codegen for varying length in a single function script
            temp = val;
            coder.varsize('temp',[1 1],[1 1]);
            if ~(isnumeric(temp) && isempty(temp))
                validateattributes(temp,{'numeric'},...
                    {'scalar','integer','nonnegative'},...
                    [class(obj) '.' prop],prop);
            end

            obj.(prop) = temp;
        end
        
    end

    methods (Access=protected)
        function flag = isInactiveProperty(obj,prop)
            % Controls the conditional display of properties

            flag = false;            
      
            % Hide AllocatedCandidate if it is overridden by a non-empty CCEOffset
            if strcmp(prop,'AllocatedCandidate')
                flag = ~isempty(obj.CCEOffset);
            end
       
        end
    end

    methods (Access = protected)
      
        function baseValidate(obj, mode, coreset, searchspace, startBWP, sizeBWP)
            % Check the cross-compatibility of the BWP, search space and CORESET

            % Check CORESETIDs
            coder.internal.errorIf(coreset.CORESETID ~= searchspace.CORESETID, ...
                'nr5g:nrPDCCHConfig:InvCORESETID', coreset.CORESETID, ...
                searchspace.CORESETID);

            % Check CORESET.FrequencyResources fits within NSizeBWP, accounting for RBOffset and NStartBWP
            % Frequency resources are defined in terms of whole blocks of 6 CRB within the BWP 
            RBs = coreset.FrequencyResources;         % Bitmap where each bit is a whole block of 6 CRB, starting after the BWP start
            allRBs = reshape(repmat(RBs,6,1),[],1);   % Expand into bitmap where each bit is a single RB
            % Establish first PRB of CORESET
            if coreset.CORESETID==0
                nrb0 = 0; % startBWP = 0
            else
                if isempty(coreset.RBOffset)  % If rb-Offset-r16 is not in use then use the R15 offset calculation (start of first complete block of 6 CRB) 
                    nrb0 = 6*ceil(double(startBWP)/6) - double(startBWP); % Place coreset, relative to NStartBWP, at 6*ceil(NStartBWP/6)
                else
                    nrb0 = double(coreset.RBOffset(1));
                end
            end
            allRBsExp = [zeros(nrb0,1); allRBs];      % Offset the bitmap to account for any fraction of 6 CRB block after the start of BWP
            allocatedRBs = find(allRBsExp==1);        % 1-based indices of signalled CORESET PRB i.e. within associated BWP

            % Check CORESET PRB fits inside the BWP
            coder.internal.errorIf(max(allocatedRBs) > sizeBWP, ...
                'nr5g:nrPDCCHConfig:InvFreqAllocation',max(allocatedRBs),sizeBWP);

            % Calculate basic resource dimensionality of CORESET
            numREGs = numel(allocatedRBs)*coreset.Duration;
            crstCCEs = numREGs/6;

            % Check first symbol and CORESET Duration are within a slot
            % (check for normal CP only), redo with carrier grid dim info
            firstSymLoc = uint32(searchspace.StartSymbolWithinSlot); % 0-based
            coder.internal.errorIf((firstSymLoc + ...
                uint32(coreset.Duration) > 14), ...
                'nr5g:nrPDCCHConfig:InvCORESETinSlot',coreset.Duration, ...
                firstSymLoc);

            % Additional client based checks
            switch lower(mode)
                case 'resources'   % PDCCH instance level check
                    validateAggregationLevel(obj,crstCCEs, searchspace);

                case 'space'       % Search space(/CORESET) level check
                    validateNumCandidates(obj,crstCCEs, searchspace);

                otherwise          % Full check
                    % check the full object (both levels)
                    validateAggregationLevel(obj,crstCCEs, searchspace);
                    validateNumCandidates(obj,crstCCEs, searchspace);
            end
        end
        
        function validateNumCandidates(~,crstCCEs, searchspace)
            % For search space, check *all* the number of candidates entries

            % Check aggregation level per SearchSpace.NumCandidates to be 
            % within crstCCEs
            numCandidates = searchspace.NumCandidates;
            aggLevels = [1 2 4 8 16];               % Aggregration levels associated with the entries in the number of candidates list
            idx = find(numCandidates,1,'last'); 
            maxAggLvl = aggLevels(idx(1));          % Largest aggregation level defined for use in search space i.e. non-zero number of candidates 
            coder.internal.errorIf(crstCCEs < maxAggLvl, ...  % Does the max aggregation level fit inside the BWP (in terms of CCE capacity)
                'nr5g:nrPDCCHConfig:InvCCEsNumCand',maxAggLvl,idx(1),crstCCEs);
        end

        function validateAggregationLevel(obj,crstCCEs, searchspace)
            % Check AggregationLevel, AllocatedCandidate for the PDCCH instance resources alone

            % Check that aggregation level used is <= numCCE in CORESET
            coder.internal.errorIf(crstCCEs < obj.AggregationLevel, ...
                'nr5g:nrPDCCHConfig:InvCCEsAggLevel',obj.AggregationLevel,crstCCEs);

            if isempty(obj.CCEOffset)       
                % Check that pdcch.AllocatedCandidate is less than or equal
                % to SearchSpace.NumCandidates for the aggregation level
                numCandidates = searchspace.NumCandidates;
                aggLevels = [1 2 4 8 16];  % Aggregration levels associated with the entries in the number of candidates list
                nCandidate = numCandidates(obj.AggregationLevel==aggLevels);
                coder.internal.errorIf(obj.AllocatedCandidate > nCandidate(1), ...
                    'nr5g:nrPDCCHConfig:InvCandidate',nCandidate(1),obj.AggregationLevel);
            else
                % Number of complete blocks of aggregation level CCE in CORESET = fix(crstCCEs/obj.AggregationLevel)
                % The *search space* defined candidates are generally a subset all possible 'candidates' in the CORESET itself

                % Largest viable CCE offset for the AL (CCE index of last complete block of AL CCE)
                largestcoff = obj.AggregationLevel*(fix(crstCCEs/obj.AggregationLevel)-1);   

                % Check that the CCE offset is an integer multiple of the AL and also not greater
                % than the first CCE of the last block of AL CCE in the CORESET
                coder.internal.errorIf(mod(obj.CCEOffset(1), obj.AggregationLevel) || obj.CCEOffset(1)>largestcoff, ...
                    'nr5g:nrPDCCHConfig:InvCCEOffsetAggLevel',obj.CCEOffset(1),obj.AggregationLevel,largestcoff);
            end
           
        end        
    end
end
