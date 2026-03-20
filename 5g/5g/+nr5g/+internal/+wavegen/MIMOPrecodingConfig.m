classdef (Abstract) MIMOPrecodingConfig
%
%   Note: This is an internal undocumented class and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2024 The MathWorks, Inc.

%#codegen

    % Public, tunable properties
    properties

        %AntennaMapping Antenna mapping vector
        % Specify AntennaMapping with a vector of ordered unique positive
        % integers to map the logical ports to columns in the output
        % waveform. The default is [], indicating that the mapping starts
        % from the first column
        AntennaMapping (1,:) = [];

        %PrecodingMatrix MIMO precoding matrix
        % Specify PrecodingMatrix with a complex matrix of size
        % Ncolumns-by-Nports, where Ncolumns is the number of columns in
        % the output waveform and Nports is the number of logical ports.
        % The default is [], indicating no MIMO precoding
        PrecodingMatrix = [];

    end

    % Dependent, hidden properties
    properties (Abstract = true, Dependent = true, Hidden = true)
        NumColumns % Number of columns required by current channel/signal
        Wpa % Effective precoding and antenna mapping matrix
    end

    % Setters
    methods

        function obj = set.AntennaMapping(obj,val)
            prop = 'AntennaMapping';
            if ~isempty(val)
                validateattributes(val,{'numeric'}, ...
                    {'vector','integer','positive'}, ...
                    [class(obj) '.' prop],prop);
                coder.internal.errorIf(~isequal(val,unique(val,'stable')), ...
                    'nr5g:nrWaveformGenerator:NonuniqueAntennaMapping')
            end
            obj.(prop) = val;
        end

        function obj = set.PrecodingMatrix(obj,val)
            prop = 'PrecodingMatrix';
            if ~isempty(val)
                validateattributes(val,{'numeric'}, ...
                    {'2d','nonnan','finite'}, ...
                    [class(obj) '.' prop],prop);
            end
            obj.(prop) = val;
        end

    end

    % Validator
    methods (Hidden = true)

        % Cross validation between AntennaMapping and PrecodingMatrix
        function validateMIMOPrecoding(obj,chStr,numPorts,chIdx)

            if ~isempty(obj.PrecodingMatrix)

                % The number of columns in PrecodingMatrix must be equal to
                % the number of logical ports
                coder.internal.errorIf(size(obj.PrecodingMatrix,2)~=numPorts, ...
                    'nr5g:nrWaveformGenerator:InvalidPrecodingMatrixNumCols', ...
                    chStr,chIdx,size(obj.PrecodingMatrix,2),numPorts);

            end

            if ~isempty(obj.AntennaMapping)

                if isempty(obj.PrecodingMatrix)

                    % The length of AntennaMapping must be equal to the
                    % number of logical ports
                    coder.internal.errorIf(numel(obj.AntennaMapping)~=numPorts, ...
                        'nr5g:nrWaveformGenerator:InvalidAntennaMappingLength', ...
                        chStr,chIdx,numel(obj.AntennaMapping),numPorts);

                else

                    % The length of AntennaMapping must be equal to the
                    % number of rows in PrecodingMatrix
                    coder.internal.errorIf(numel(obj.AntennaMapping)~=size(obj.PrecodingMatrix,1), ...
                        'nr5g:nrWaveformGenerator:IncompatiblePrecodingMatrixSize', ...
                        chStr,chIdx,numel(obj.AntennaMapping),size(obj.PrecodingMatrix,1));

                end

            end

        end
        
    end

    methods (Access = protected)

        % Calculate the matrix that should be multiplied to the generated
        % grid to perform first MIMO precoding and then mapping to output
        % columns
        function P = calculatePrecodeAndMapMatrix(obj,numPorts,W)

            % Get precoding matrix
            if nargin<3
                W = obj.PrecodingMatrix;
            end

            % Get number of output columns
            numCols = obj.NumColumns;

            % P1: precoding matrix
            if isempty(W)
                % Empty PrecodingMatrix indicates no MIMO precoding, hence
                % expand into identity matrix
                P1 = eye(numCols,numPorts);
            else
                P1 = W;
            end

            % P2: antenna mapping matrix
            if isempty(obj.AntennaMapping) || isequal(obj.AntennaMapping,(1:numCols))
                % No effective antenna mapping, hence expand into identity
                % matrix
                P2 = eye(numCols,size(P1,1),'like',P1);
            else
                % Map the i-th port (after precoding) to the column
                % indicated by the i-th element in non-empty well-defined
                % AntennaMapping vector
                P2 = zeros(numCols,size(P1,1),'like',P1);
                antInd = sub2ind(size(P2),(obj.AntennaMapping)',(1:numel(obj.AntennaMapping))');
                P2(antInd) = 1;
            end

            % P: overall operation matrix, NumColumns-by-NumPorts
            P = P2*P1;

        end

        % Get number of output columns required by the current instance
        function numCols = getNumCols(obj,numPorts)

            if isempty(obj.PrecodingMatrix)
                if isempty(obj.AntennaMapping)
                    numCols = numPorts;
                else
                    numCols = max(obj.AntennaMapping,[],'all');
                end
            else
                numCols = size(obj.PrecodingMatrix,1);
                if ~isempty(obj.AntennaMapping)
                    numCols = max(obj.AntennaMapping,[],'all');
                end
            end

        end

    end

end