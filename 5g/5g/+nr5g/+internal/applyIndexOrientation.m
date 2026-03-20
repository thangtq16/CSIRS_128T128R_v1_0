function out = applyIndexOrientation(carrierDims,bwpDims,nslotsymb,opts,ind,varargin)
%applyIndexOrientation Orients the indices relative to the required grid
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   OUT = ...
%   nr5g.internal.applyIndexOrientation(CARRIERDIMS,BWPDIMS,NSLOTSYMB,OPTS,IND)
%   returns the indices OUT oriented with reference to carrier or bandwidth
%   part based on the field IndexOrientation in the structure OPTS,
%   provided the inputs, two-element vector carrier dimensions CARRIERDIMS
%   [NSTARTGRID, NSIZEGRID], two-element vector bandwidth part dimensions
%   BWPDIMS [NSTARTBWP, NSIZEBWP], number of OFDM symbols per slot
%   NSLOTSYMB and input indices IND with respect to carrier grid. The input
%   OPTS contains another field IndexBase to indicate whether the input IND
%   is 1-based or 0-based. IND is either a column vector or a three column
%   matrix in subscript form [k l p].
%
%   OUT = nr5g.internal.applyIndexOrientation(...,CARRIERREF) allows the
%   control of providing the input indices orientation through CARRIERREF.
%   The value of 1 indicates the input indices are with respect to carrier
%   grid and value of 0 indicates the input indices are with respect to
%   BWP grid.

%   Copyright 2019-2020 The MathWorks, Inc.

%#codegen

    % Initialize inputs
    nStartGrid = carrierDims(1);
    nSizeGrid = carrierDims(2);
    nStartBWP = bwpDims(1);
    nSizeBWP = bwpDims(2);

    % Offset of start of BWP with respect to start of carrier, in number of
    % subcarriers
    offset = (nStartBWP-nStartGrid)*12;
    base = 0;
    if strcmpi(opts.IndexBase,'0based')
        % To know if the input is 0-based or 1-based. If the input is
        % 0-based, apply the base to the input
        base = 1;
    end

    out = ind;
    if ~isempty(ind)

        % Output orientation flag
        bwpOutputOrientation = strcmpi(opts.IndexOrientation,'bwp');

        % Input index reference (Carrier or BWP)
        if nargin == 6
            carrierRef = varargin{1};
        else
            carrierRef = 1;
        end
        % Grid dimensions of the input indices IND
        if carrierRef
            gridDims = nSizeGrid;
        else
            gridDims = nSizeBWP;
        end
        l = nslotsymb;
        p = ceil(double(max(ind(:,1)+base))/(nSizeGrid*12*l));
        if ~isfinite(p)
            p = 1;
        end

        % Create a grid
        inputGrid = zeros([gridDims*12 l p]);
        if iscolumn(ind)
            % For linear indexing, assign the indices to grid
            inputGrid(ind+base) = 1;
        end
        nRBSC = 12;

        if carrierRef
            % Input indices IND is carrier oriented, so reference grid is
            % carrier
            if bwpOutputOrientation
                % To change to BWP orientation
                if iscolumn(ind)
                    % Extract the bandwidth part from carrier grid and find
                    % the indices
                    bwp = inputGrid(offset+1:offset+nSizeBWP*nRBSC,:,:);
                    out = uint32(find(bwp))-base;
                else
                    out(:,1) = out(:,1)-offset;
                end
            end
        else
            % Input indices IND is BWP oriented, so reference grid is bwp
            if ~bwpOutputOrientation
                % To change to CARRIER orientation
                if iscolumn(ind)
                    % Map the bandwidth part in the carrier grid and find
                    % the indices
                    carrier = zeros([nSizeGrid*12 l p]);
                    carrier(offset+1:offset+nSizeBWP*nRBSC,:,:) = inputGrid;
                    out = uint32(find(carrier))-base;
                else
                    out(:,1) = out(:,1)+offset;
                end
            end
        end
    end
end