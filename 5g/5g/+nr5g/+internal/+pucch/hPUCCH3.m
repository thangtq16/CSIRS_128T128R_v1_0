function sym = hPUCCH3(uciCW,modulation,nid,rnti,Mrb,sf,occi,varargin)
%hPUCCH3 Physical uplink control channel format 3
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    % Scrambling, TS 38.211 Section 6.3.2.6.1
    c = nrPUCCHPRBS(nid,rnti,length(uciCW));
    btilde = xor(uciCW,c);

    % Modulation, TS 38.211 Section 6.3.2.6.2
    d = nrSymbolModulate(btilde,modulation,varargin{:});

    interlaced = ~isempty(sf);
    if ~interlaced % Direct mapping when interlacing is disabled

        y = d;

    else % Block spreading with interlaced mapping, TS 38.211 Section 6.3.2.6.3

        % Validate input size and block-spreading configuration
        nRE = 12;
        formatPUCCH = 3;
        nr5g.internal.pucch.validateSpreadingConfig(length(d),modulation,Mrb,nRE,sf,formatPUCCH);

        % Blockwise spreading TS 38.211 Section 6.3.2.6.3
        y = nr5g.internal.pucch.blockWiseSpread(d,Mrb,sf,occi);

    end

    % Transform precoding, TS 38.211 Section 6.3.2.6.4
    sym = nrTransformPrecode(y,Mrb);

end