function infrastructureGridLayoutAxes = infrastructureGridLayoutVisualizer(bsCoordinates, ueCoordinates, numSectorPerSite, interSiteDistance, isWrapAround, bsTag, ueTag, args)
%infrastructureGridLayoutVisualizer Visualize the network for the given
%infrastructure grid layout configuration
%
%   Note: This is an internal undocumented method and its API and/or
%   functionality may change in subsequent releases.
%
%   INFRASTRUCTUREGRIDLAYOUTAXES = infrastructureGridLayoutVisualizer
%   (BSCOORDINATES, UECOORDINATES, NUMSECTORPERSITE, INTERSITEDISTANCE,
%   WRAPAROUND, BSTAG, UETAG) creates an infrastructure grid layout in a
%   new figure according to the user specified configurations.
%
%   INFRASTRUCTUREGRIDLAYOUTAXES is the UI axes object used to plot an
%   infrastructure grid layout.
%
%   The inputs are as follows:
%   BSCOORDINATES       - 3D Cartesian coordinates of base station (BS)
%   UECOORDINATES       - 3D Cartesian coordinates of user equipment (UE)
%   NUMSECTORPERSITE    - Number of sectors per cell-site
%   INTERSITEDISTANCE   - Distance between two adjacent BSs (in meters)
%   WRAPAROUND          - Enable wrap-around modeling
%   BSTAG               - Tag values corresponding to BSs
%   UETAG               - Tag values corresponding to UEs
%
%   INFRASTRUCTUREGRIDLAYOUTAXES = infrastructureGridLayoutVisualizer(
%   BSCOORDINATES, UECOORDINATES, NUMSECTORPERSITE, INTERSITEDISTANCE,
%   WRAPAROUND, BSTAG, UETAG, NAME=VALUE) creates an infrastructure grid
%   layout according to the user specified configurations.You can specify
%   additional name-value arguments in any order as (Name1=Value1, ...,
%   NameN=ValueN).
%
%   The allowed Name=Value pairs are as follows:
%   Axes specifies the cartesian axes used to visualize the network. If not
%   provided, a new figure window will be created to visualize the network.
%
%   IsAddUE is logical scalar that specifies whether additional nodes (UEs)
%   are to be plotted in the existing network.
%
%   UserData is a structure that contains technology specific information
%   used to identify a network element in the visualization. It contains
%   the following fields:
%   Technology, Boundary, CentralNode and PeripheralNode.

%   Copyright 2023-2024 The MathWorks, Inc.

arguments
    bsCoordinates
    ueCoordinates
    numSectorPerSite
    interSiteDistance
    isWrapAround
    bsTag
    ueTag
    args.Axes
    args.IsAddUE = 0
    args.UserData
end

numBS = size(bsCoordinates,1);
numUE = size(ueCoordinates,1);

if ~isfield(args, 'Axes') % Create a new figure
    % Using the screen width and height, calculate the figure width and height
    resolution = get(0, 'ScreenSize');
    screenWidth = resolution(3);
    screenHeight = resolution(4);
    figureWidth = screenWidth * 0.75;
    figureHeight = screenHeight * 0.75;
    fig = uifigure(Name="Network Layout Visualization",Position=[screenWidth * 0.05 screenHeight * 0.05 figureWidth figureHeight]);
    % Use desktop theme to support dark theme mode
    matlab.graphics.internal.themes.figureUseDesktopTheme(fig);
    g = uigridlayout(fig, [1 1]);
    ax = uiaxes(Parent=g);
else
    ax = args.Axes;
end
if isfield(args, 'UserData')
    userData = args.UserData;
end

colors = ["--mw-graphics-colorOrder-2-primary", "--mw-borderColor-primary", "--mw-color-selected-noFocus"];
import matlab.graphics.internal.themes.specifyThemePropertyMappings

ax.DataAspectRatio = [1 1 1];
ax.XLabel.String = "X-axis (Meters)";
ax.YLabel.String = "Y-axis (Meters)";
hold (ax,'on');

if ~args.IsAddUE % Scenario creation
    numCluster = 1;

    switch numBS
        % Calculate wrap-around offsets
        case 1
            switch numSectorPerSite
                case 1
                    wrappingOffsetX = [0,sqrt(3)*cosd(30:60:360)];
                    wrappingOffsetY = [0,sqrt(3)*sind(30:60:360)];
                case 3
                    wrappingOffsetX = [0,3*cosd(0:60:300)];
                    wrappingOffsetY = [0,3*sind(0:60:300)];
            end
        case 7
            switch numSectorPerSite
                case 1
                    wrappingOffsetX = [0,3/2,4.5,3,-3/2,-9/2,-3];
                    wrappingOffsetY = [0,5*sqrt(3)/2,sqrt(3)/2,-2*sqrt(3), ...
                        -5*sqrt(3)/2,-sqrt(3)/2,2*sqrt(3)];
                case 3
                    wrappingOffsetX = [0,6,7.5,1.5,-6,-7.5,-1.5];
                    wrappingOffsetY = [0,3*sqrt(3),-3*sqrt(3)/2,-9*sqrt(3)/2, ...
                        -6*sqrt(3)/2,3*sqrt(3)/2,9*sqrt(3)/2];
            end
        case 19
            switch numSectorPerSite
                case 1
                    wrappingOffsetX = [0,9/2,15/2,3,-9/2,-15/2,-3];
                    wrappingOffsetY = [0,7*sqrt(3)/2,-sqrt(3)/2,-4*sqrt(3), ...
                        -7*sqrt(3)/2,sqrt(3)/2,4*sqrt(3)];
                case 3
                    wrappingOffsetX = [0,12,21/2,-3/2,-12,-21/2,+3/2];
                    wrappingOffsetY = [0,3*sqrt(3),-9*sqrt(3)/2,-15*sqrt(3)/2, ...
                        -3*sqrt(3),9*sqrt(3)/2,15*sqrt(3)/2];
            end
    end

    if numSectorPerSite == 1
        a1 = zeros(1,3);
        a2 = a1;
        cellSide = interSiteDistance/sqrt(3);
    else
        cellSide = interSiteDistance/3;
        % Offset coordinates, if a BS is at the meeting point of 3 sectors
        a1 = cellSide*cosd(0:120:240);
        a2 = cellSide*sind(0:120:240);
    end
    % Vertices of the polygon that forms the boundary of a cell
    vx = cellSide*cosd(0:60:360); % x coordinates
    vy = cellSide*sind(0:60:360); % y coordinates

    % Plot the network layout
    for i = 1:numCluster
        for j = 1:numBS
            bx = cellSide*wrappingOffsetX(i)+bsCoordinates(j,1); % BS X-coordinate
            by = cellSide*wrappingOffsetY(i)+bsCoordinates(j,2); % BS Y-coordinate
            for k = 1:numSectorPerSite
                % Center coordinates of hexagonal cell
                centerX = bx+a1(k);
                centerY = by+a2(k);
                % Calculate vertices of hexagonal cell
                verticesXCoordinates = centerX+vx;
                verticesYCoordinates = centerY+vy;

                % Plot cell-site
                if ~isWrapAround
                    cellBoundary = plot(ax,verticesXCoordinates,verticesYCoordinates,Tag="Cell"+((j-1)*numSectorPerSite+k));
                    if isfield(args, 'UserData')
                        cellBoundary.UserData={userData.Technology, userData.Boundary};
                    end
                    specifyThemePropertyMappings(cellBoundary,Color=colors(3));
                elseif  isWrapAround && i == 1 % Highlight center cluster
                    cellBoundary = plot(ax,verticesXCoordinates,verticesYCoordinates,LineWidth=1.5,Tag="Cell"+((j-1)*numSectorPerSite+k));
                    if isfield(args, 'UserData')
                        cellBoundary.UserData={userData.Technology, userData.Boundary};
                    end
                    specifyThemePropertyMappings(cellBoundary,Color=colors(3));
                end

                % Plot base stations and specify their color mapping for the
                % current theme
                if i>1 % When wrap-around is enabled
                    virtualBS = scatter(ax,bx,by,Marker="^",MarkerFaceAlpha=0.5,Tag="VirtualBS");
                    specifyThemePropertyMappings(virtualBS,MarkerFaceColor=colors(1));
                    specifyThemePropertyMappings(virtualBS,MarkerEdgeColor=colors(1));

                    cellBoundary = plot(ax,verticesXCoordinates,verticesYCoordinates,"k",Tag='VirtualCell');
                    specifyThemePropertyMappings(cellBoundary,Color=colors(3));
                else
                    bs = plot(ax,bx,by,Marker="^",MarkerSize=8,Tag=bsTag(j));
                    if isfield(args, 'UserData')
                        bs.UserData={userData.Technology, userData.CentralNode};
                    end
                    specifyThemePropertyMappings(bs,MarkerFaceColor=colors(1));
                    specifyThemePropertyMappings(bs,MarkerEdgeColor=colors(1));
                end
            end
        end
    end
end

% Plot UE(s)
for idx = 1:numUE
    px = ueCoordinates(idx,1); % UE X-coordinate
    py = ueCoordinates(idx,2); % UE Y-coordinate
    ue = plot(ax,px,py,Marker="o",MarkerSize=4,Tag=ueTag(idx));
    if isfield(args, 'UserData')
        ue.UserData={userData.Technology, userData.PeripheralNode};
    end
    % Specify UE color mapping for the current theme
    specifyThemePropertyMappings(ue,MarkerFaceColor=colors(2));
    specifyThemePropertyMappings(ue,MarkerEdgeColor=colors(2));
end

ax.XLimMode = 'auto';
ax.YLimMode = 'auto';
hold (ax,'off');

% Return axes
infrastructureGridLayoutAxes = ax;
end
