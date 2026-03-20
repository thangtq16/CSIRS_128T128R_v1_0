function [QUCI,NumLayersUCI,QmUCI,ModUCI,NumLayersNoUCI,QmNoUCI,ModNoUCI] = getUCIMultiplexInfo(pusch,tcr)
%getUCIMultiplexInfo returns the codeword number of the codeword on which
%UCI is to be multiplexed and calculates the number of layers of this
%codeword, and also returns its modulation order and modulation scheme.
%This function also returns the number of layers and modulation scheme and
%order of the codeword without UCI, in two-codeword transmission.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2023 The MathWorks, Inc.

%#codegen

    if pusch.NumLayers <= 4 % 1 CW
        
        QUCI = 0;
        NumLayersUCI = pusch.NumLayers;
        if iscell(pusch.Modulation)
            QmUCI = nr5g.internal.getQm(pusch.Modulation{1});
            ModUCI = pusch.Modulation{1};
        else
            QmUCI = nr5g.internal.getQm(pusch.Modulation);
            ModUCI = pusch.Modulation;
        end
        NumLayersNoUCI = 0;
        QmNoUCI = 0;
        ModNoUCI = 'QPSK'; % As there is no 2nd CW, use default modulation scheme of PUSCH

    else % 2 CW

        % Get modulation orders
        fcnName = 'getUCIMultiplexInfo';
        modList = {'pi/2-BPSK','QPSK','16QAM','64QAM','256QAM'};
        modScheme = nr5g.internal.validatePXSCHModulation(fcnName,pusch.Modulation,2,modList);
        Qm1 = nr5g.internal.getQm(modScheme{1});
        Qm2 = nr5g.internal.getQm(modScheme{2});

        % Get target code rates
        if isscalar(tcr)
            tcr1 = tcr;
            tcr2 = tcr;
        else
            tcr1 = tcr(1);
            tcr2 = tcr(2);
        end

        % Compare I_MCS to get QUCI
        % Codeword with higher I_MCS has higher Qm, or if Qm are the same,
        % higher tcr
        QUCI = 0;
        QmUCI = Qm1;
        QmNoUCI = Qm2;
        if Qm2 > Qm1
            % CW q=1 has higher Qm, hence higher I_MCS
            QUCI = 1;
            QmUCI = Qm2;
            QmNoUCI = Qm1;
        elseif Qm2 == Qm1
            % Same Qm: compare tcr
            if tcr2 > tcr1
                % CW q=1 has higher tcr with same Qm, hence higher I_MCS
                QUCI = 1;
            end
        end

        % Calculate number of layers allocated to both codewords and their
        % respective modulation scheme
        layersPerCW = fix((double(pusch.NumLayers) + (0:1))/2);
        NumLayersUCI = layersPerCW(QUCI+1);
        ModUCI = modScheme{QUCI+1};
        NumLayersNoUCI = layersPerCW(2-QUCI);
        ModNoUCI = modScheme{2-QUCI};

    end

end