%hVectorDataSource Create data vector source of given input vector or PN source
%
%   VECSOURCE = hVectorDataSource(...) constructs a vector data
%   source object VECSOURCE, which then can be used to generate data.
%
%   VECSOURCE = hVectorDataSource(DATASOURCE) constructs a vector data
%   source object VECSOURCE, from a scalar, vector, character vector or
%   cell array specified by DATASOURCE. If DATASOURCE is scalar or vector,
%   the values are looped around to generate data. DATASOURCE can also be
%   one of the pseudo-random sequences specified by a value in the set
%   ('PN9-ITU', 'PN9', 'PN11', 'PN15', 'PN23') or can be a cell array
%   containing the pseudo-random sequence source definition and its seed in
%   the format {PNSOURCE, SEED}, where PNSOURCE can be one of the above
%   mentioned values. If no seed is specified for the PN sources, the shift
%   register will be initialized with all ones. If seed is specified as 0,
%   the shift register will be random initialized.
%
%   VECTOR = VECSOURCE.getPacket(LENGTH) creates a data vector VECTOR by
%   appropriate looping of the vector specified in DATASOURCE or as per the
%   specified PN sequence for given LENGTH.
%
%   VECSOURCE.reset() resets the internal state to point to the start of
%   the input sequence.
%
%   Examples:
%   % Example 1:
%   % Create a data source from the sequence [1 0 1]
%   source = hVectorDataSource([1 0 1]);
%   % Generate data of length 10
%   data = source.getPacket(10)'
%
%   % The above example returns:
%   data = [1 0 1 1 0 1 1 0 1 1]
%
%   % Example 2:
%   % Create a data source from standard PN sequence 'PN9-ITU' and seed 2
%   source = hVectorDataSource({'PN9-ITU',2});
%   % Generate data of length 10
%   data = source.getPacket(10)'
%
%   % The above example returns:
%   data = [0 1 0 0 0 0 0 0 0 0]

%   Copyright 2007-2021 The MathWorks, Inc.

%#codegen

classdef hVectorDataSource < handle
    
    properties
        sourcePN9
        sourcePN9ITU
        sourcePN11
        sourcePN15
        sourcePN23
        
        currentstate
        initialstate = 0
        sourcespec
    end
    
    methods
        
        function obj = hVectorDataSource(sourcespec, varargin)
            
            defaultseed = 1; % Flag to indicate default PN seed
            if nargin == 0
                % Zeros as default
                obj.sourcespec = 0;
            elseif iscell(sourcespec)
                % If the source is PN, sourcespec can be a character vector or a cell
                % array {'PN9',seed} or {'PN9'} or 'PN9'
                if numel(sourcespec) == 2
                    obj.initialstate = sourcespec{2};
                    defaultseed = 0;
                end
                obj.sourcespec = sourcespec{1};
            else
                obj.sourcespec = sourcespec;
            end
            
            shiftreglength = createPNSources(obj, obj.sourcespec, defaultseed, obj.initialstate, varargin{:});
            
            % Validate initial state - can only be positive integer or 0
            coder.internal.errorIf(~isnumeric(obj.sourcespec) && ...
                ~(isnumeric(obj.initialstate) && isscalar(obj.initialstate) && ...
                ((obj.initialstate >=0) && (obj.initialstate <= 2^shiftreglength-1))), ...
                'nr5g:nrWaveformGenerator:InvalidPNSeed', obj.sourcespec, 2^shiftreglength-1);
            
            obj.currentstate = obj.initialstate;
            
            coder.internal.errorIf(isnumeric(obj.sourcespec) && obj.currentstate >= length(obj.sourcespec), ...
                'nr5g:nrWaveformGenerator:InvalidNumSeed',  length(obj.sourcespec)-1);
        end
        
        % Class 'Method' implementations
        function bitsout = getPacket(obj, psize)
            
            if ischar(obj.sourcespec) || isstring(obj.sourcespec)
                if ~psize
                    % Return an empty vector if psize==0
                    bitsout = zeros(0,1);
                else
                    bitsout = getPacketPN(obj, psize);
                end
            else
                bitsout = zeros(psize,1);
                for i = 1:psize
                    bitsout(i) = obj.sourcespec(obj.currentstate+1);
                    obj.currentstate = mod(obj.currentstate+1,length(obj.sourcespec));
                end
            end
        end
        
        function bitsout = getPacketPN(obj, psize)
            if strcmpi(obj.sourcespec, 'PN9-ITU')
                bitsout = obj.sourcePN9ITU(psize);
            elseif strcmpi(obj.sourcespec, 'PN9')
                bitsout = obj.sourcePN9(psize);
            elseif strcmpi(obj.sourcespec, 'PN11')
                bitsout = obj.sourcePN11(psize);
            elseif strcmpi(obj.sourcespec, 'PN15')
                bitsout = obj.sourcePN15(psize);
            else
                bitsout = obj.sourcePN23(psize);
            end
        end
        
        function reset(obj)
            obj.currentstate = obj.initialstate;
        end
        
        function shiftreglengthOut = createPNSources(obj, sourcespec, defaultseed, initState, varargin)
            % 1) comm.PNSequence is an sfun-based System object, thus inputs must
            % be constant in codegen. Hence, the initialstateBin calculation is
            % duplicated instead of reused in a function
            % 2) All possible PN objects are unconditionally created to help codegen,
            % as otherwise getPacketPN cannot be persuaded that the desired PN obj has been
            % created for all execution paths
            
            shiftreglengthOut = NaN; % init for codegen
            
            if nargin >= 5
                maxSize = [varargin{1} 1];
            else
                maxSize = [2745600 1];
                % only Uplink waveform generation enters this code for now.
                % It can be adjusted at the caller level to provide tighter memory
                % bounds. The 2745600 number is the same as maximum TBS size (same
                % for DL and UL).
            end
            
            %% PN9-ITU
            poly = [9 4 0];
            shiftreglength = 9;
            if strcmp(sourcespec, 'PN9-ITU')
                shiftreglengthOut = shiftreglength;
                if defaultseed
                    obj.initialstate = 2^shiftreglength-1;
                end
            end
            
            if defaultseed
                initialstateBin = ones(1, shiftreglength);
            else
                % If user-specified seed is 0, randomize the seed
                if initState==0
                    initialstateBin = randi([0, 1], 1, shiftreglength);
                else
                    initialstateBin = comm.internal.utilities.convertInt2Bit(initState,shiftreglength)';
                end
            end
            
            obj.sourcePN9ITU = comm.PNSequence('Polynomial',poly,'InitialConditions', initialstateBin,...
                'Mask',[zeros(1,shiftreglength-1) 1], 'VariableSizeOutput', true, 'MaximumOutputSize', maxSize);
            
            %% PN9
            poly = [9 5 0];
            shiftreglength = 9;
            if strcmp(sourcespec, 'PN9')
                shiftreglengthOut = shiftreglength;
                if defaultseed
                    obj.initialstate = 2^shiftreglength-1;
                end
            end
            
            if defaultseed
                initialstateBin = ones(1, shiftreglength);
            else
                % If user-specified seed is 0, randomize the seed
                if initState==0
                    initialstateBin = randi([0, 1], 1, shiftreglength);
                else
                    initialstateBin = comm.internal.utilities.convertInt2Bit(initState,shiftreglength)';
                end
            end
            
            obj.sourcePN9 = comm.PNSequence('Polynomial',poly,'InitialConditions', initialstateBin, ...
                'Mask',[zeros(1,shiftreglength-1) 1], 'VariableSizeOutput', true, 'MaximumOutputSize', maxSize);
            
            %% PN11
            poly = [11 2 0];
            shiftreglength = 11;
            if strcmp(sourcespec, 'PN11')
                shiftreglengthOut = shiftreglength;
                if defaultseed
                    obj.initialstate = 2^shiftreglength-1;
                end
            end
            
            if defaultseed
                initialstateBin = ones(1, shiftreglength);
            else
                % If user-specified seed is 0, randomize the seed
                if initState==0
                    initialstateBin = randi([0, 1], 1, shiftreglength);
                else
                    initialstateBin = comm.internal.utilities.convertInt2Bit(initState,shiftreglength)';
                end
            end
            
            obj.sourcePN11 = comm.PNSequence('Polynomial',poly,'InitialConditions',initialstateBin, ...
                'Mask',[zeros(1,shiftreglength-1) 1], 'VariableSizeOutput', true, 'MaximumOutputSize', maxSize);
            
            %% PN15
            poly = [15 14 0];
            shiftreglength = 15;
            if strcmp(sourcespec, 'PN15')
                shiftreglengthOut = shiftreglength;
                if defaultseed
                    obj.initialstate = 2^shiftreglength-1;
                end
            end
            
            if defaultseed
                initialstateBin = ones(1, shiftreglength);
            else
                % If user-specified seed is 0, randomize the seed
                if initState==0
                    initialstateBin = randi([0, 1], 1, shiftreglength);
                else
                    initialstateBin = comm.internal.utilities.convertInt2Bit(initState,shiftreglength)';
                end
            end
            
            obj.sourcePN15 = comm.PNSequence('Polynomial',poly,'InitialConditions',initialstateBin, ...
                'Mask',[zeros(1,shiftreglength-1) 1], 'VariableSizeOutput', true, 'MaximumOutputSize', maxSize);
            
            %% PN23
            poly = [23 5 0];
            shiftreglength = 23;
            if strcmp(sourcespec, 'PN23')
                shiftreglengthOut = shiftreglength;
                if defaultseed
                    obj.initialstate = 2^shiftreglength-1;
                end
            end
            
            if defaultseed
                initialstateBin = ones(1, shiftreglength);
            else
                % If user-specified seed is 0, randomize the seed
                if initState==0
                    initialstateBin = randi([0, 1], 1, shiftreglength);
                else
                    initialstateBin = comm.internal.utilities.convertInt2Bit(initState,shiftreglength)';
                end
            end
            
            obj.sourcePN23 = comm.PNSequence('Polynomial',poly,'InitialConditions',initialstateBin, ...
                'Mask',[zeros(1,shiftreglength-1) 1], 'VariableSizeOutput', true, 'MaximumOutputSize', maxSize);
        end
    end
end

