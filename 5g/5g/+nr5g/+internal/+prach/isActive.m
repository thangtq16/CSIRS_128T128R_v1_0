function active = isActive(prach)
% Establish if the PRACH is active in the current subframe and slot.
%
%    Note: This is an internal undocumented function and its API and/or
%    functionality may change in subsequent releases.
%
%   ACTIVE = nr5g.internal.prach.isActive(PRACH) returns true if the PRACH
%   is active in the current subframe and slot. This is checked according
%   to these rules:
%   1. PRACH is active in the current slot. That is, NPRACHSlot (modulo activePRACHSlotPeriod)
%      is the same as ActivePRACHSlot.
%   2. PRACH is active in the current subframe (FR1) or slot (FR2), as
%      specified by these tables in TS 38.211:
%       * Table 6.3.3.2-2 for FR1 and paired spectrum/supplementary uplink
%       * Table 6.3.3.2-3 for FR1 and unpaired spectrum
%       * Table 6.3.3.2-4 for FR2 and unpaired spectrum
%
%   PRACH is a PRACH configuration object, <a href="matlab:help('nrPRACHConfig')">nrPRACHConfig</a>.
%   Only these object properties are relevant for this function:
%
%   FrequencyRange       - Frequency range (used in combination with
%                          DuplexMode to select a configuration table from
%                          TS 38.211 Table 6.3.3.2-2 to 6.3.3.2-4)
%   DuplexMode           - Duplex mode (used in combination with
%                          FrequencyRange to select a configuration table
%                          from TS 38.211 Table 6.3.3.2-2 to 6.3.3.2-4)
%   ConfigurationIndex   - Configuration index, as defined in TS 38.211
%                          Tables 6.3.3.2-2 to 6.3.3.2-4
%   SubcarrierSpacing    - PRACH subcarrier spacing in kHz
%   LRA                  - Length of the Zadoff-Chu preamble sequence
%   ActivePRACHSlot      - Active PRACH slot number within a subframe or a
%                          60 kHz slot
%   NPRACHSlot           - PRACH slot number

%  Copyright 2019-2022 The MathWorks, Inc.

%#codegen
    
    % TS 38.211 Section 6.3.3.2 reports that, for the purpose of slot
    % numbering in the configuration tables, a subcarrier spacing of 15 kHz
    % shall be assumed for FR1, whereas a subcarrier spacing of 60 kHz
    % shall be assumed for FR2.
    if strcmpi(prach.FrequencyRange,'FR1')
        slotNumberingSCS = 15;
        slotNumberingSlotsPerFrame = 10;
    else
        slotNumberingSCS = 60;
        slotNumberingSlotsPerFrame = 40;
    end

    % Get the relative slot number and the appropriate frame number
    % based on the PRACH slot number and the subcarrier spacing of both
    % PRACH and carrier
    scs = prach.SubcarrierSpacing;
    isShortPreamble = any(prach.LRA==[139,1151,571]);
    if isShortPreamble
        nSlot = double(floor(prach.NPRACHSlot*slotNumberingSCS/scs));
    else % Long preamble
        L = nr5g.internal.prach.gridSymbolSize(prach);
        % Note that the sequence corresponding to preamble format 1 spans 2
        % OFDM symbols and thus its corresponding grid is 2 OFDM symbols
        % long. However, the length of the cyclic prefix for this format is
        % such that the overall PRACH occasion spans 3 OFDM symbols. This
        % difference is dealt with in the OFDM modulation as the grid
        % represents the preamble sequences only. However, when checking if
        % the current PRACH occasion is active, we need to consider this
        % difference between the grid size and the actual OFDM symbols
        % occupied by the preamble. This is done in the line below by
        % considering an additional OFDM symbol in the case of preamble
        % format 1.
        if strcmpi(prach.Format,'1')
            L = L + 1;
        end
        % The value of L up to this point corresponds to the number of OFDM
        % symbols occupied by the PRACH preamble. This also corresponds to
        % the number of PRACH slots occupied by the preamble in the case of
        % a 1.25 kHz subcarrier spacing. For this reason, L is normalized
        % with respect to a 1.25 kHz subcarrier spacing.
        L = L * 1.25/scs;
        % The slot number for long preambles can last for more than one
        % carrier 15 kHz slot. The number of slots occupied by the preamble
        % equals the number of OFDM symbols normalized with respect to a
        % 1.25 kHz PRACH subcarrier spacing.
        nSlot = prach.NPRACHSlot*L+(0:L-1);
    end
    [nslot,sfn] = nr5g.internal.getRelativeNSlotAndSFN(nSlot,0,slotNumberingSlotsPerFrame);
    
    % Check if PRACH is active in the current subframe (FR1) or slot (FR2)
    params = nr5g.internal.prach.getVariablesFromConfigTable(prach.FrequencyRange,prach.DuplexMode,prach.ConfigurationIndex);
    x = params.x;
    y = params.y;
    validSF = params.sfn;
    
    active = any(ismember(nslot,validSF) & mod(sfn,x(1))==repmat(y,size(sfn)));
    
    % Check if PRACH is active in the current slot, as discussed in
    % TS 38.211 Section 5.3.2 and Tables 6.3.3.2-2 to 6.3.3.2-4.
    % Note that this check is performed only when PRACH SubcarrierSpacing
    % is 30, 120, 480, or 960 kHz.
    if any(scs==[30,120,480,960])
        % ActivePRACHSlot can be:
        % * [0,1] for 30 or 120 kHz
        % * [3,7] for 480 kHz
        % * [7,15] for 960 kHz
        activePRACHSlotPeriod = scs/slotNumberingSCS;
        active = active && (prach.ActivePRACHSlot==mod(prach.NPRACHSlot,activePRACHSlotPeriod));
    end
    
end