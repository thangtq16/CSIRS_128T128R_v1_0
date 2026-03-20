function [NSlotR,NFrameR] = getRelativeNSlotAndSFN(NSlotA,NFrameA,SlotsPerFrame)
% getRelativeNSlotAndSFN returns the relative slot and frame numbers
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%   [NSLOTR,NFRAMER] = getRelativeNSlotAndSFN(NSLOTA,NFRAMEA,SLOTSPERFRAME)
%   returns the relative slot number NSLOTR and system frame number NFRAMER
%   corresponding to the absolute slot number NSLOTA and system frame
%   number NFRAMEA for a given number of slots per frame SLOTSPERFRAME.
%
%   Example:
%   SCS = 30; % Subcarrier spacing
%   SlotsPerFrame = 10*SCS/15; 
%   [nslot, sfn] = nr5g.internal.getRelativeNSlotAndSFN( SlotsPerFrame+1, 1025 ,SlotsPerFrame)

%  Copyright 2019 The MathWorks, Inc.

%#codegen

    % Calculate the appropriate frame number (0...1023) based on the
    % absolute slot number
    NFrameR = mod(NFrameA + fix(NSlotA/SlotsPerFrame),1024);
    % Relative slot number (0...slotsPerFrame-1)
    NSlotR = mod(NSlotA,SlotsPerFrame);

end