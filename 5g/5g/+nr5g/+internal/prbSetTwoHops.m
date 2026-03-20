function out = prbSetTwoHops(prbset,freqHopping,rbStartHop2,nslot)
%prbSetTwoHops Provides the set of physical resource blocks in both hops
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   OUT = prbSetTwoHops(PRBSET,FREQHOPPING,RBSTARTHOP2,NSLOT) returns the
%   matrix, OUT, containing two rows of physical resource blocks, with each
%   row corresponding to each hop in the slot. First row corresponds to
%   first hop in the slot and second row corresponds to second hop in the
%   slot. OUT depends on the set of physical resource blocks PRBSET,
%   frequency hopping configuration FREQHOPPING, second hop starting
%   resource block RBSTARTHOP2, and the slot number NSLOT. FREQHOPPING must
%   be one of {'intraSlot', 'interSlot', 'neither'}. When FREQHOPPING is
%   'neither', both the rows in OUT have same values present in PRBSET.
%   When FREQHOPPING is 'interSlot', both the rows in OUT have same values,
%   with values in a row depending on PRBSET, NSLOT, and RBSTARTHOP2. When
%   FREQHOPPING is 'intraSlot', OUT contains different values in each row,
%   depending on PRBSET and RBSTARTHOP2.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    % Get the PRBs in each hop
    prbset = reshape(prbset,1,[]); % Reshape to row vector
    out = zeros(2,numel(prbset));  % Each row is for each hop in a slot
    secondHopStartPRB = double(rbStartHop2);
    if strcmpi(freqHopping,'interSlot') && (mod(nslot,2) == 1)
        % Update the first hop PRBs to account the odd number of slots in
        % case of inter-slot frequency hopping
        prbset = prbset - prbset(1) + secondHopStartPRB;
    end
    out(1,:) = prbset; % Assign first hop PRBs
    if strcmpi(freqHopping,'intraSlot')
        % If intra-slot frequency hopping, get the second hop PRBs
        out(2,:) = prbset - prbset(1) + secondHopStartPRB;
    else
        % Assign second hop PRBs with the PRBs in first hop
        out(2,:) = prbset;
    end

end
