classdef CodingCommon
    %CodingCommon Common wavegen configuration object for PUCCH formats 2,3,4
    %
    %   Note: This is an internal undocumented class and its API and/or
    %   functionality may change in subsequent releases.
    %
    %   CodingCommon properties (configurable):
    %
    %   Coding - Flag to enable channel coding (default true)

    %   Copyright 2021-2023 The MathWorks, Inc.

    %#codegen

    % Public, tunable properties
    properties
        %Coding Flag to enable channel coding
        % Specify Coding as a scalar logical. Setting Coding to true
        % enables channel coding for the uplink control information (UCI).
        % The default is true.
        Coding (1,1) logical = true;
    end
end
