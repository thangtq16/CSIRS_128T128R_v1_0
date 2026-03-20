function [bsCoordinates, ueCoordinates, ueAssociationInfo, cellRadius] = indoorHotspotLayout(floorLength, floorBreadth, numBSRow, numBSColumn, interSiteDistance, uePlacementType, numUE, propArgs)
%indoorHotspotLayout Calculate base station (BS) and user equipment (UE)
%positions for the indoor hotspot scenario
%
%   Note: This is an internal undocumented method and its API and/or
%   functionality may change in subsequent releases.
%
%   [BSCOORDINATES, UECOORDINATES, UEASSOCIATIONINFO, CELLRADIUS] =
%   indoorHotspotLayout(FLOORLENGTH, FLOORBREADTH, NUMBSROW, NUMBSCOLUMN,
%   UEPLACEMENTTYPE, NUMUE) returns the BS and UE positions, UE association
%   info, and cell radius for the given indoor hotspot configuration.
%
%   BSCOORDINATES is a matrix of size N-by-3, where N is the total number
%   of BS i.e., NUMBSROW*NUMBSCOLUMN. The 'i'th row of BSCOORDINATES
%   contains the Cartesian coordinates of a BS.
%   UECOORDINATES is a matrix of size M-by-3, where M is the total number
%   of UEs. The 'j'th row of UECOORDINATES contains the Cartesian
%   coordinates of a UE.
%   UEASSOCIATIONINFO is a vector of size M, where M is the total number of
%   UEs. The 'k'th element of UEASSOCIATIONINFO contains the BS ID to
%   which 'k'th UE is connected.
%   CELLRADIUS is the radius of each cell (in meters).
%
%   The inputs are as follows:
%   FLOORLENGTH         - Length of the indoor floor (in meters). Specify
%                         floorLength as a numeric positive scalar.
%   FLOORBREADTH        - Breadth of the indoor floor (in meters). Specify
%                         floorBreadth as a numeric positive scalar.
%   NUMBSROW            - Number of BS along X-axis. Specify numBSRow as a
%                         numeric positive integer.
%   NUMBSCOLUMN         - Number of BS along Y-axis. Specify numBSColumn
%                         as a numeric positive integer.
%   INTERSITEDISTANCE  - Distance between two adjacent BSs (in meters).
%                        interSiteDistance as a numeric positive scalar.
%   UEPLACEMENTTYPE     - Scalar value to control the distribution of UEs.
%                         Value 0 means that the total count of UEs in the
%                         scenario is numUE. Value 1 means that the count
%                         of UEs in each cell is numUE. Value 2 means that
%                         an addtional number of UEs specified by numUE,
%                         are added to an existing cell. Only Value 0 and
%                         Value 1 are supported in the default signature.
%   NUMUE               - Number of UEs in the scenario depending on the
%                         uePlacementType. uePlacementType 0 means that
%                         total count of UEs in the scenario is numUE.
%                         uePlacementType 1 means that the the total count
%                         of UEs in the scenario is numBSRow*numBSColumn*numUE.
%                         uePlacementType 2 means that the total count of
%                         UEs in the scenario is increased by numUE.
%                         Specify numUE as a numeric positive integer.
%
%   [BSCOORDINATES, UECOORDINATES, UEASSOCIATIONINFO, CELLRADIUS] =
%   indoorHotspotLayout(FLOORLENGTH, FLOORBREADTH, NUMBSROW, NUMBSCOLUMN,
%   UEPLACEMENTTYPE, NUMUE, BSCOORDINATES, BSID) creates additional UE(s) 
%   in an existing indoor hotspot layout depending upon the UEPLACEMENTTYPE.
%   If UEPLACEMENTTYPE is 0 or 1, specify all the existing BS coordinates
%   and their corresponding IDs. If UEPLACEMENTTYPE is 2, specify the
%   coordinates and ID of an existing BS, to which the additional UE(s) are
%   expected to be associated with.
%
%   BSCOORDINATES is equal to the input BSCOORDINATES.
%   UECOORDINATES is a matrix of size N-by-3, where N is additional number
%   of UE(s). The 'j'th row of UECOORDINATES contains the Cartesian
%   coordinates of a UE.
%   UEASSOCIATIONINFO is a vector of size N, where N is additional number
%   of UE(s). The 'k'th element of UEASSOCIATIONINFO contains the BS ID to
%   which 'k'th additional UE is connected.
%   CELLRADIUS is the radius of each cell (in meters).
%
%   BSPosition and BSId are optional inputs.
%   BSPosition - A matrix of size N-by-3, where N is defined as:
%           N = Total number of existing BS(s), if UEPLACEMENTTYPE is
%           0 or 1
%           N = 1, if UEPLACEMENTTYPE is 2
%   BSId  - Integer value specifying the ID(s) of the BS(s),
%           whose coordinates are specified by BSCOORDINATES.

%   Copyright 2023-2024 The MathWorks, Inc.

arguments
    floorLength
    floorBreadth
    numBSRow
    numBSColumn
    interSiteDistance
    uePlacementType
    numUE
    propArgs.BSPosition;
    propArgs.BSId;
end

% Calculate the offset from the x and y direction
maxHorizontalNumGNBs = floorLength/interSiteDistance;
maxVerticalNumGNBs = floorBreadth/interSiteDistance;
if maxVerticalNumGNBs < numBSColumn || maxHorizontalNumGNBs < numBSRow
    error("The number of gNBs exceeds" + ...
        " the maximum capacity for the given indoor hall dimensions." + ...
        " Reduce the number of gNBs or increase the hall size to maintain the ISD.")
end

% Generate the center positions around the origin
xDistOffset = -((numBSRow-1)/2)*interSiteDistance;
yDistOffset = ((numBSColumn-1)/2)*interSiteDistance;

% BS position initialization
numBS = numBSRow*numBSColumn;
if isfield(propArgs,'BSPosition') &&  isfield(propArgs,'BSId')
    bsCoordinates = round(propArgs.BSPosition,1);
    bsID = propArgs.BSId;
else % BS coordinates are not provided
    bsCoordinates = zeros(numBS, 3);
    bsID = 1:numBS;
    for rowIdx = 1:numBSColumn
        for colIdx = 1:numBSRow
            index = (rowIdx-1)*numBSRow+colIdx;
            % Assign Z-coordinate as BS antenna height (m), as per ITU-R, Guidelines for
            % evaluation of radio interface technologies for IMT-2020
            zCoordinate = 3;
            % Row-wise placement of BS starting from top left
            pos = [xDistOffset+(colIdx-1)*interSiteDistance, yDistOffset-(rowIdx-1)*interSiteDistance, zCoordinate];
            bsCoordinates(index, :) = round(pos,1);
        end
    end
end

% Generate the coordinates for UEs
cellRadius = interSiteDistance/2;
if uePlacementType==0 % If UEs are randomly placed across the floor
    % UE position initialization
    ueCoordinates = zeros(numUE, 3);
    ueAssociationInfo = zeros(numUE, 1);
    numBS = size(bsCoordinates, 1);
    for i = 1:numUE
        randBSIndex = randi([1 numBS]);
        selectedbsID = bsID(randBSIndex);
        % Calculate UE position and association information
        [ueCoordinates(i,:), ueAssociationInfo(i)] = uePlacementPerCell(bsCoordinates(randBSIndex,:), ...
            selectedbsID, cellRadius, 1);
    end
    
elseif uePlacementType==1 % If UEs distribution is per cell 
    ueCoordinates = zeros(numBS*numUE, 3);
    ueAssociationInfo = zeros(numBS*numUE, 1);
    count = 1;
    for bsIndex=1:numBS
        selectedbsID = bsID(bsIndex);
        bsCoord = bsCoordinates(bsIndex,:); % 3D BS coordinates
        % Random position generation for UEs connected to a BS
        [pos, ueAssociation] = uePlacementPerCell(bsCoord, selectedbsID, cellRadius, numUE);
        ueCoordinates(count:count+numUE-1,:) = pos;
        ueAssociationInfo(count:count+numUE-1) = ueAssociation;
        count = count+numUE;
    end
else % uePlacementType==2
    [ueCoordinates, ueAssociationInfo] = uePlacementPerCell(bsCoordinates, bsID, cellRadius, numUE);
end
end

%% Local function
function [ueCoordinates, ueAssociationInfo] = uePlacementPerCell(bsCoordinates, bsID, cellRadius, numUE)
%uePlacementPerCell Generate random UE position within a cell radius
%
%   [UECOORDINATES, UEASSOCIATIONINFO] = uePlacementPerCell(BSCOORDINATES,
%   BSID, CELLRADIUS, NUMUE) returns UE positions for the given indoor
%   hotspot scenario.
%
%   UECOORDINATES is a matrix of size N-by-3, where N is the total number
%   of UEs connected to a BS. The 'i'th row of UECOORDINATES contains the
%   Cartesian coordinates of a UE.
%   UEASSOCIATIONINFO is a vector of size N, where N is the total number of
%   UEs connected to a BS. The 'j'th element of UEASSOCIATIONINFO contains
%   the BS ID to which 'j'th UE is connected.
%
%   The inputs are as follows:
%   BSCOORDINATES  - 3D Cartesian coordinates of a BS
%   BSID           - ID of the BS having position specified by BSCOORDINATES
%   CELLRADIUS     - Radius of the given cell (in meters)
%   NUMUE          - Number of UEs connected to a BS

% UE position initialization
ueCoordinates = zeros(numUE, 3);
ueAssociationInfo = repmat(bsID, numUE, 1);
uePos = [0 0 0];

% Define the circular area for a cell
bsTheta = 0:pi/60:2*pi;
xv = cellRadius*cos(bsTheta)+bsCoordinates(1);
yv = cellRadius*sin(bsTheta)+bsCoordinates(2);
for i = 1:numUE
    flag = 1;
    % Random position of a UE
    while flag
        % Calculate UE position within the cell
        ueTheta = 360*rand;
        r = cellRadius*rand;
        uePos(1) = round(bsCoordinates(1)+r*cos(ueTheta),1);
        uePos(2) = round(bsCoordinates(2)+r*sin(ueTheta),1);
        % Assign Z-coordinate as UE antenna height (m), as per ITU-R, Guidelines for
        % evaluation of radio interface technologies for IMT-2020
        uePos(3) = 1.5;
        % Check if the random point is inside the cell
        [in,on] = inpolygon(uePos(1), uePos(2), xv, yv);
        if in && ~on && ~isequal(uePos, bsCoordinates)
            flag = 0;
        end
    end
    ueCoordinates(i,:) = uePos;
end
end