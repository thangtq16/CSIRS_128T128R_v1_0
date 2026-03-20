function refSymbols = decoderReferenceSymbols(carrier,pucch,ouci,numSym,newCache)
%decoderReferenceSymbols PUCCH reference symbols for ML decoder for formats 2 and 3
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    % Persistent variables to store PUCCH reference symbols
    persistent cache cachedReferenceSymbols valid;

    % Cache the scrambling identity
    if isempty(pucch.NID)
        nid = double(carrier.NCellID);
    else
        nid = double(pucch.NID(1));
    end
    newCache.NID = nid;

    % Verify that the number of symbols is consistent with OUCI and Gd
    [Gd,qm,sf] = verifyNumSymbols(pucch,ouci,numSym);
    
    % Number of different reference sequences
    numSeqs = 2^double(ouci(1));

    % Create cache for the first time
    if isempty(cache)
        cache = newCache;
        valid = false;
        cachedReferenceSymbols = complex(zeros(Gd*sf,real(numSeqs)));
    end

    % Calculate number of RB (Mrb) and update new cache
    prbset = nr5g.internal.allocatedPhysicalResourceBlocks(carrier, pucch);
    newCache.Mrb = numel(prbset);

    % If the new and old parameter caches are not equal, generate symbols.
    if ~isequal(newCache,cache) || ~valid

        % Make cache invalid to ensure the cache is not corrupted due to a
        % problem with PUCCH reference symbol generation
        valid = false; %#ok<NASGU>

        % Initialize reference symbols for all possible OUCI-long bit
        % sequences (2^ouci)
        sym = complex(zeros(Gd*sf,real(numSeqs)));
        for ul = 0:numSeqs-1
            enc = nrUCIEncode(int2bit(ul,ouci(1),false),Gd*qm);
            sym(:,ul+1) = nrPUCCH(carrier,pucch,enc);
        end
        refSymbols = sym;
        cachedReferenceSymbols = refSymbols;

        % Update parameter cache
        cache = newCache;
        valid = true;

    else
        refSymbols = cachedReferenceSymbols;
    end

end


function [Gd,qm,sf] = verifyNumSymbols(pucch,ouci,numSym)

    interlacing = nr5g.internal.interlacing.isInterlaced(pucch);

    % Spreading factor for single-interlace transmissions
    if interlacing && numel(pucch.InterlaceIndex) == 1
        sf = double(pucch.SpreadingFactor);
    else
        sf = 1;
    end
    Gd = numSym/sf;

    % Modulation order
    pucchFormat = nr5g.internal.pucch.getPUCCHFormat(pucch);
    if pucchFormat == 2
        qm = 2;
    else % format 3
        qm = nr5g.internal.getQm(pucch.Modulation);
    end

    % Check the product of modulation order and Gd is an integer
    % multiple of spreading factor
    G = Gd*qm;
    coder.internal.errorIf(fix(G) ~= G,'nr5g:nrPUCCHDecode:InvalidG',qm,Gd,sf);

    % Check ouci and Gd values (qm*Gd must be greater than ouci)
    coder.internal.errorIf(G <= ouci(1), ...
        'nr5g:nrPUCCHDecode:InvalidSYMLenF23',qm,Gd,ouci(1));
   
end
