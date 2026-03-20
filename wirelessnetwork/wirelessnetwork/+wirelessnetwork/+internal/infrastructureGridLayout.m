function [bsCoordinates, ueCoordinates, ueAssociationInfo] = infrastructureGridLayout(floorLength, floorBreadth, numSite, numSectorPerSite, interSiteDistance, uePlacementType, numUE, propArgs)
%infrastructureGridLayout Calculate base station (BS) and user equipment
%(UE) positions for the infrastructure grid
%
%   Note: This is an internal undocumented method and its API and/or
%   functionality may change in subsequent releases.
%
%   [BSCOORDINATES, UECOORDINATES, UEASSOCIATIONINFO] =
%   infrastructureGridLayout(NUMSITE, NUMSECTORPERSITE, INTERSITEDISTANCE,
%   UEPLACEMENTTYPE, NUMUE) returns the BS and UE positions for the given
%   network. The base stations are assumed to be at the center of each
%   site.
%
%   BSCOORDINATES is a matrix of size M-by-3, where M is the total number
%   of BS. The 'i'th row BSCOORDINATES contains the Cartesian coordinates
%   of a BS.
%   UECOORDINATES is a matrix of size N-by-3, where N is the total number
%   of UEs. The 'j'th row of UECOORDINATES contains the Cartesian
%   coordinates of a UE.
%   UEASSOCIATIONINFO is a vector of size N, where N is the additional
%   number of UEs. The 'k'th element of UEASSOCIATIONINFO contains the BS
%   ID to which 'k'th additional UE is connected.
%
%   The inputs are as follows:
%   FLOORLENGTH        - Length of the scenario (in meters). Specify
%                        floorLength as a numeric positive scalar.
%   FLOORBREADTH       - Breadth of the scenario (in meters). Specify
%                        floorBreadth as a numeric positive scalar.
%   NUMSITE            - Number of sites in the network. Specify numSite
%                        as a numeric positive integer.
%   NUMSECTORPERSITE   - Number of sectors per site. Specify
%                        numSectorPerSite value as 1 or 3.
%   INTERSITEDISTANCE  - Distance between two adjacent BSs (in meters).
%                        interSiteDistance as a numeric positive scalar.
%   UEPLACEMENTTYPE    - Scalar value to control the distribution of UEs.
%                        Value 0 means that the total count of UEs in the
%                        scenario is numUE. Value 1 means that the count
%                        of UEs in each cell is numUE. Value 2 means that
%                        an addtional number of UEs specified by numUE,
%                        are added to an existing cell. Only Value 0 and
%                         Value 1 are supported in the default signature.
%   NUMUE              - Number of UEs in the scenario depending on the
%                        uePlacementType. uePlacementType 0 means that
%                        total count of UEs in the scenario is numUE.
%                        uePlacementType 1 means that the the total count
%                        of UEs in the scenario is numSite*numSectorPerSite*numUE.
%                        uePlacementType 2 means that the total count of
%                        UEs in the scenario is increased by numUE. Specify
%                        numUE as a numeric positive integer.
%
%   [BSCOORDINATES, UECOORDINATES, UEASSOCIATIONINFO] =
%   infrastructureGridLayout(NUMSITE, NUMSECTORPERSITE, INTERSITEDISTANCE,
%   UEPLACEMENTTYPE, NUMUE, BSCOORDINATES, BSID) creates additional UE(s)
%   in an existing infrastructure grid layout depending upon the
%   UEPLACEMENTTYPE. If UEPLACEMENTTYPE is 0 or 1, specify all the existing
%   BS coordinates and their corresponding IDs. If UEPLACEMENTTYPE is 2,
%   specify the coordinates and ID of an existing BS, to which the
%   additional UE(s) are expected to be associated with.
%
%   BSCOORDINATES is equal to the input BSCOORDINATES.
%   UECOORDINATES is a matrix of size N-by-3, where N is additional number
%   of UE(s). The 'j'th row of UECOORDINATES contains the Cartesian
%   coordinates of a UE.
%   UEASSOCIATIONINFO is a vector of size N, where N is additional number
%   of UE(s). The 'k'th element of UEASSOCIATIONINFO contains the BS ID to
%   which 'k'th additional UE is connected.
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
    numSite
    numSectorPerSite
    interSiteDistance
    uePlacementType
    numUE
    propArgs.BSPosition;
    propArgs.BSId;
    propArgs.ScenarioType = "Custom"; % Default scenario type
end

length = min(floorLength, floorBreadth);
if (numSite == 1 && length < interSiteDistance) || ... 
    (numSite == 7 && 1.5*interSiteDistance > length/2) || ...
    (numSite == 19 && 2.5*interSiteDistance > length/2)
    error("The number of gNBs exceeds" + ...
        " the maximum capacity for the given scenario canvas dimensions." + ...
        " Reduce the number of gNBs or increase the scenario canvas size to maintain the ISD.");
end 

% centerX and centerY are x and y coordinates of BSs respectively
if numSectorPerSite == 1
    centerX = [0,sqrt(3)*cosd(30:60:360),2*sqrt(3)*cosd(30:60:360),3*cosd(0:60:300)];
    centerY = [0,sqrt(3)*sind(30:60:360),2*sqrt(3)*sind(30:60:360),3*sind(0:60:300)];
    cellSide = interSiteDistance/sqrt(3);
else
    centerX = [0,3*cosd(0:60:300),6*cosd(0:60:300),3*sqrt(3)*cosd(30:60:360)];
    centerY = [0,3*sind(0:60:300),6*sind(0:60:300),3*sqrt(3)*sind(30:60:360)];
    cellSide = interSiteDistance/3;
end

% BS,UE antenna height (m) as per 3GPP TR 38.901 Table 7.4.1-1
if propArgs.ScenarioType == "UMa" % Urban macro
    bsHeight = 25;
    ueHeight = 1.5;
elseif propArgs.ScenarioType == "UMi" % Urban micro
    bsHeight = 10;
    ueHeight = 1.5;
elseif propArgs.ScenarioType == "RMa" % Rural macro
    bsHeight = 35;
    ueHeight = 1.5;
else
    ueHeight = 0;
    bsHeight = 0;
end

if isfield(propArgs,'BSPosition') &&  isfield(propArgs,'BSId')
    bsCoordinates = round(propArgs.BSPosition,1);
    bsID = propArgs.BSId;
else
    % Generate the coordinates for BS
    bsCoordinates = zeros(numSite, 3); % BS position initialization
    bsID = 1:numSite;
    for i = 1:numSite
        % Set antenna height as z-coordinate
        bsCoordinates(i,:) = round([cellSide*[centerX(i), centerY(i)] bsHeight],1);
    end
end

% Initialization
ueCoordinates = zeros(numUE, 3);
ueAssociationInfo = zeros(numUE, 1);
numBS = size(bsCoordinates, 1);

% Vertices of the polygon that forms the boundary of a cell
vx = cellSide*cosd(0:60:360);
vy = cellSide*sind(0:60:360);

% Generate the coordinates for UEs
if uePlacementType==0  % If UEs are randomly placed across the grid
    for i = 1:numUE
        randBSIndex = randi([1 numBS]);
        selectedbsID = bsID(randBSIndex);
        [ax, ay] = calculateOffsetCoordinates(numSectorPerSite, uePlacementType, cellSide);

        % Calculate the vertices coordinates for the chosen cell
        verticesXCoordinates = bsCoordinates(randBSIndex,1)+vx+ax;
        verticesYCoordinates = bsCoordinates(randBSIndex,2)+vy+ay;

        % Calculate UE's position and association information
        [ueCoordinates(i,:), ueAssociationInfo(i)] = uePlacementPerCell(bsCoordinates(randBSIndex,:), ...
            selectedbsID, verticesXCoordinates, verticesYCoordinates, cellSide, 1, ueHeight);
    end

elseif uePlacementType==1 % If UE distribution is per cell
    ueCoordinates = zeros(numSite*numSectorPerSite*numUE, 3);
    ueAssociationInfo = zeros(numSite*numSectorPerSite*numUE, 1);
    
    [a1, a2] = calculateOffsetCoordinates(numSectorPerSite, uePlacementType, cellSide);
    count = 1;
    for i = 1:numSite
        selectedbsID = bsID(i);
        bsPos = bsCoordinates(i,:); % BS coordinates
        for j = 1:numSectorPerSite
            % Center coordinates of the cell
            centerX = bsPos(1)+a1(j);
            centerY = bsPos(2)+a2(j);
            % Vertex coordinates of a particular cell
            verticesXCoordinates = centerX+vx;
            verticesYCoordinates = centerY+vy;
            % Calculate UEs position and association information
            [uePos, ueAssociation] = uePlacementPerCell(bsPos, selectedbsID, ...
                verticesXCoordinates, verticesYCoordinates, cellSide, numUE, ueHeight);
            ueCoordinates(count:count+numUE-1,:) = uePos;
            ueAssociationInfo(count:count+numUE-1) = ueAssociation;
            count = count + numUE;
        end
    end

else % uePlacementType==2
    for i = 1:numUE
        randBSIndex = randi([1 numBS]); % randBSIndex is always 1 here
        [ax, ay] = calculateOffsetCoordinates(numSectorPerSite, uePlacementType, cellSide);

        % Calculate the vertices coordinates for the chosen cell
        verticesXCoordinates = bsCoordinates(randBSIndex,1)+vx+ax;
        verticesYCoordinates = bsCoordinates(randBSIndex,2)+vy+ay;

        % Calculate UEs position and association information
        [ueCoordinates(i,:), ueAssociationInfo(i)] = uePlacementPerCell(bsCoordinates(randBSIndex,:), ...
            bsID, verticesXCoordinates, verticesYCoordinates, cellSide, 1, ueHeight);
    end
end
end

%% Local functions
function [ueCoordinates, ueAssociationInfo] = uePlacementPerCell(bsCoordinates, bsID, verticesXCoordinates, verticesYCoordinates, cellSide, numUE, ueHeight)
%uePlacementPerCell Generate random UE positions within a cell-site
%
%   [UECOORDINATES, UEASSOCIATIONINFO] = uePLACEMENTPERCELL(BSCOORDINATES,
%   BSID, VERTICESXCOODINATES, VERTICESYCOODINATES, CELLSIDE, NUMUE, UEHEIGHT)
%   returns UE positions for the given cell.
%
%   UECOORDINATES is a matrix of size N-by-3, where N is the total number
%   of UEs connected to a BS. The 'i'th row of UECOORDINATES contains the
%   Cartesian coordinates of a UE.
%   UEASSOCIATIONINFO is a vector of size N, where N is the total number of
%   UEs connected to a BS. The 'j'th element of UEASSOCIATIONINFO contains
%   the BS ID to which 'j'th UE is connected.   
%   
%   The inputs are as follows:
%   BSCOORDINATES       - 3D Cartesian coordinates of a base station
%   BSID                - ID of the BS having position specified by BSCOORDINATES
%   VERTICESXCOODINATES - Vertices' x-coordinates for the given cell
%   VERTICESYCOODINATES - Vertices' y-coordinates for the given cell
%   CELLSIDE            - Side length of the given cell (in meters)
%   NUMUE               - Number of UEs connected to a BS
%   UEHEIGHT            - Height of the UE (in meters)

% UE position initialization
ueCoordinates = zeros(numUE, 3);
ueAssociationInfo = repmat(bsID, numUE, 1);
uePos = [0 0 ueHeight]; % Set antenna height as z-coordinate
% Calculate UE's position and association information
for i = 1:numUE
    flag = 1;
    while flag
        % Calculate UE position within the cell
        r = cellSide*rand;
        theta = 360*rand;
        uePos(1) = round(r*cosd(theta)+bsCoordinates(1),1);
        uePos(2) = round(r*sind(theta)+bsCoordinates(2),1);
        % Check if the random point is inside the cell
        [in,on] = inpolygon(uePos(1), uePos(2), verticesXCoordinates, verticesYCoordinates);
        if in && ~on && ~isequal(uePos, bsCoordinates)
            flag = 0;
        end
    end
    ueCoordinates(i,:) = uePos;
end
end

function [ax, ay] = calculateOffsetCoordinates(numSectorPerSite, uePlacementType, cellSide)
%calculateOffsetCoordinates Calculates the offset coordinates based on the
%number of sectors per site and the UE placement type

if uePlacementType==1
    if numSectorPerSite == 1
        ax = zeros(1,3);
        ay = ax;
    else % numSectorPerSite==3
        % Offset coordinates, if a BS is at the meeting point of 3 sectors
        ax = cellSide*cosd(0:120:240);
        ay = cellSide*sind(0:120:240);
    end
else % uePlacementType==0 || uePlacementType==2
    if numSectorPerSite == 1
        ax = 0;
        ay = 0;
    else % numSectorPerSite==3
        % Randomly select a sector out of three sectors
        randSector = randi([1 3]);
        angle = 120*(randSector-1);

        % Offset coordinates
        ax = cellSide*cosd(angle);
        ay = cellSide*sind(angle);
    end
end
end
