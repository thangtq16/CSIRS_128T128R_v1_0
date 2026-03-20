function networkLayoutAxes = networkLayoutVisualizer(bsCoordinates, ueCoordinates, cellRadius, bsTag, ueTag, args)
%networkLayoutVisualizer Visualize the network for a given indoor hotspot
% layout or random layout configuration
%
%   Note: This is an internal undocumented method and its API and/or
%   functionality may change in subsequent releases.
%
%   NETWORKLAYOUTAXES = networkLayoutVisualizer(BSCOORDINATES,
%   UECOORDINATES, CELLRADIUS, BSTAG, UETAG) creates a network layout in a
%   new figure according to the user specified configurations.
%
%   NETWORKLAYOUTAXES is the UI axes object used to plot a network layout.
%
%   The inputs are as folllows:
%   BSCOORDINATES  - 3D Cartesian coordinates of base station (BS)
%   UECOORDINATES  - 3D Cartesian coordinates of user equipment (UE)
%   CELLRADIUS     - Radius of each cell (in meters)
%   BSTAG          - Tag values corresponding to BSs
%   UETAG          - Tag values corresponding to UEs
%
%   NETWORKLAYOUTAXES = networkLayoutVisualizer(BSCOORDINATES,
%   UECOORDINATES, CELLRADIUS, BSTAG, UETAG, NAME=VALUE) creates a network
%   layout according to the user specified configurations. You can specify
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
    cellRadius
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

colors = ["--mw-graphics-colorOrder-2-primary", "--mw-borderColor-primary"];
import matlab.graphics.internal.themes.specifyThemePropertyMappings

ax.DataAspectRatio = [1 1 1];
ax.XLabel.String = "X-axis (Meters)";
ax.YLabel.String = "Y-axis (Meters)";
hold (ax,'on');

if ~args.IsAddUE % Scenario creation
    % Plot the network
    for i = 1:numBS
        bx = bsCoordinates(i,1); % BS X-coordinate
        by = bsCoordinates(i,2); % BS Y-coordinate
        % Plot the circle representing each cell
        th = 0:pi/60:2*pi;
        x = cellRadius*cos(th)+bx;
        y = cellRadius*sin(th)+by;
        cellBoundary = plot(ax,x,y,Tag="Cell"+(i));
        if isfield(args, 'UserData')
            cellBoundary.UserData={userData.Technology, userData.Boundary};
        end
        specifyThemePropertyMappings(cellBoundary,Color="--mw-color-selected-noFocus");

        % Plot BS
        bs = plot(ax,bx,by,Marker="^",MarkerSize=8,Tag=bsTag(i));
        if isfield(args, 'UserData')
            bs.UserData={userData.Technology, userData.CentralNode};
        end
        % Specify BS color mapping for the current theme
        specifyThemePropertyMappings(bs,MarkerFaceColor=colors(1));
        specifyThemePropertyMappings(bs,MarkerEdgeColor=colors(1));
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

% Set axes limits
ax.XLimMode = 'auto';
ax.YLimMode = 'auto';
hold (ax,'off');

% Return axes
networkLayoutAxes = ax;
end