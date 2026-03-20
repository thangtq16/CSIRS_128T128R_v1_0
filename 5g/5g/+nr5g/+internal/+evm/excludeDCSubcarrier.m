function [rxGrids,refGridAllRS] = excludeDCSubcarrier(dcInd,rxGrids,refGridAllRS,varargin)
%excludeDCSubcarrier Sets the content in DC subcarrier to 0
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   [grid1,grid2] = excludeDCSubcarrier(DCIND,GRID1,GRID2)
%   [grid1,grid2] = excludeDCSubcarrier(DCIND,GRID1,GRID2,FLAG) allows to
%   remove content of surrounding DC subcarriers in GRID2, when number of
%   layers is more than 1.
%
%   Note that GRID1 and GRID2 must be have same number of subcarriers.
%   DCIND is 0-based location.

% Copyright 2024-2025 The MathWorks, Inc.

%#codegen

    % Ignore DC subcarrier location, in case, the value is empty or greater
    % than largest possible resource block number (0-based) in the carrier.
    numSCs = size(rxGrids,1);
    if ~isempty(dcInd) && (dcInd(1) < size(rxGrids,1))
        % Exclude DC from received waveform and refGrid
        dcInd = dcInd(1)+1;                                % Make the dcInd 1-based
        rxGrids(dcInd,:,:,:) = 0;
        refGridAllRS(dcInd,:,:,:) = 0;
        if (size(rxGrids,3) > 1) && (nargin == 4) && varargin{1}
            % For number of layers > 1, exclude up to 24 subcarriers in
            % refGrid. These 24 subcarriers include the resource block
            % containing DC subcarrier and 6 subcarriers from the adjacent
            % resource blocks. These 24 subcarriers also account for
            % allocation edges. This is needed to avoid channel estimates
            % distortion (due to exclusion of the DC reference from
            % 'refGrid')
            rbLoc = floor((dcInd-1)/12);

            dcLow = rbLoc*12-5;
            highAdj = 0;                          % Last subcarrier adjustment
            if dcLow < 1
                highAdj = abs(dcLow);
                dcLow = 0;
            end
            dcHigh = rbLoc*12 + 12 + 6 + highAdj;

            lowAdj = 0;                           % Starting subcarrier adjustment
            if dcHigh >= numSCs
                lowAdj = (dcHigh - numSCs)+1;
                dcHigh = numSCs-1;
            end
            dcLow = dcLow - lowAdj;
            dcLoc = (dcLow:dcHigh)+1;             % 1-based indexing
            dcLoc(dcLoc < 1) = [];
            dcLoc(dcLoc > numSCs) = [];
            refGridAllRS(dcLoc,:,:,:) = 0;
        end
    end

end
