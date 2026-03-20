function [PL,SF] = nrPathLoss(pathLossConfig,freq,LOS,posBS,posUE)
%nrPathLoss TR 38.901 path loss and shadow fading
%   [PL,SF] = nrPathLoss(PATHLOSSCONFIG,FREQ,LOS,POSBS,POSUE) returns the
%   path loss PL (dB) between user equipment (UE) and base station (BS) at
%   frequency FREQ and the associated shadow fading standard deviation SF
%   (dB), as defined in TR 38.901 Section 7.4.1. The output path loss and
%   shadow fading are NBS-by-NUE matrices, where NBS and NUE are the number
%   of BS and UE, respectively. The input PATHLOSSCONFIG is an
%   nrPathLossConfig object specifying the scenario characteristics and
%   path loss model. POSBS and POSUE are the Cartesian coordinates of the
%   BS and UE as 3-by-NBS and 3-by-NUE matrices, respectively. The first
%   two rows of POSBS and POSUE configure the 2-D positions and the third
%   specifies the heights of BS and UE, respectively. LOS, a scalar or
%   NBS-by-NUE logical matrix, specifies the existence of line of sight
%   between each pair of BS and UE. If LOS is a scalar, all pairs of BS-UE
%   share the same line of sight condition.
%
%   Example 1:
%   % Calculate the propagation path loss between a UE and a BS at 3.5
%   % GHz in a rural macrocell scenario with an average height of buildings
%   % of 7 m and width of streets of 25 m.
%   
%   plc = nrPathLossConfig;
%   plc.Scenario = "RMa"; 
%   plc.BuildingHeight = 7;
%   plc.StreetWidth = 25;
%   
%   freq = 3.5e9;
%   los = true;
%   pbs = [0;0;30];
%   pue = [1e3;1e3;1.5];
%   
%   pl = nrPathLoss(plc,freq,los,pbs,pue);
%   disp(pl)
%
%   Example 2:
%   % Calculate the propagation path loss between 10 UEs and 2 BS at 3.5 
%   % GHz in a rural macrocell scenario with an average height of buildings
%   % of 7 m and width of streets of 25 m.
%   
%   % Path loss configuration
%   plc = nrPathLossConfig;
%   plc.Scenario = "RMa"; 
%   plc.BuildingHeight = 7;
%   plc.StreetWidth = 25;
%   
%   % Carrier frequency
%   freq = 3.5e9; 
%   
%   % Define position of BSs and UEs
%   pbs = [-500 500; 0 0; 30 50];
%   NBS = size(pbs,2);
%   NUE = 10;
%   pue = zeros(3,NUE);
%   pue(1:2,:) = 2e3*(rand(2,NUE)-0.5);
%   pue(3,:) = 1 + rand(1,NUE);
%   
%   % Line of sight between BS and UE
%   los = randi([0 1],NBS,NUE);
%   
%   pl = nrPathLoss(plc,freq,los,pbs,pue);
%   disp(pl)
%   
%   See also nrPathLossConfig.

%   Copyright 2021-2023 The MathWorks, Inc.

%#codegen    

    narginchk(5,5);
    
    [pathLossConfig,freq,posBS,posUE] = validateInputs(pathLossConfig,freq,LOS,posBS,posUE);
    
    persistent c0;
    if isempty(c0)
        c0 = physconst('lightspeed');
    end
    
    % Path loss specific configuration
    scenario = pathLossConfig.Scenario;
    hE = pathLossConfig.EnvironmentHeight;
    W = pathLossConfig.StreetWidth;
    h = pathLossConfig.BuildingHeight;
    
    % Carrier frequency in GHz
    fc = freq/1e9;  
    
    % Number of BS and UE
    nBS = size(posBS,2);
    nUE = size(posUE,2);
    
    % Initialize path loss
    PL = zeros(nBS,nUE);
    SF = zeros(nBS,nUE);
    
    % Expansion of LOS
    if isscalar(LOS)
        los = repmat(logical(LOS),nBS,nUE);
    else
        los = logical(LOS);
    end
    nlos = ~los;
    anyNLOS = any(nlos,"all");
    
    % Expansion of BS and UE positions for matrix operations
    bs = permute(repmat(posBS,1,1,nUE),[2 3 1]);
    ue = permute(repmat(posUE,1,1,nBS),[3 2 1]);
    hBS = bs(:,:,3);
    hUT = ue(:,:,3);
    
    % 3D distance between BS and UE
    d3D  = sqrt((ue(:,:,1)-bs(:,:,1)).^2 + ...
                (ue(:,:,2)-bs(:,:,2)).^2 + ...
                (ue(:,:,3)-bs(:,:,3)).^2);
    
    switch scenario
        
        case 'RMa'
            % Break point distance
            dBP = 2*pi*hBS.*hUT*freq/c0;
            d2D = sqrt(d3D.^2-(hBS-hUT).^2);
            far = d2D>dBP; % Distance beyond breakpoint
            
            % LOS path loss model before breakpoint
            PL1 = @(d3d) 20*log10(40*pi*d3d*fc/3) + min(0.03*h^1.72,10)*log10(d3d)...
                        - min(0.044*h^1.72,14.77) + 0.002*log10(h)*d3d;
            % LOS path loss model beyond breakpoint
            PL2 = @(d3d,dbp) PL1(dbp) + 40*log10(d3d./dbp);
            
            PL(~far) = PL1(d3D(~far));
            PL(far) = PL2(d3D(far),dBP(far));
            SF(~far & los) = 4;
            SF( far & los) = 6;
            
            % NLOS path loss model
            if anyNLOS
                PLp = @(d3d,hbs,hut) ...
                        161.04 - 7.1*log10(W) + 7.5*log10(h) ...
                       -(24.37-3.7*(h./hbs).^2).*log10(hbs) ...
                       +(43.42-3.1*log10(hbs)).*(log10(d3d)-3) ...
                       +20*log10(fc)-(3.2*(log10(11.75*hut)).^2-4.97);
                PL(nlos) = max(PL(nlos),PLp(d3D(nlos),hBS(nlos),hUT(nlos)));
                SF(nlos) = 8;
            end
            
        case 'UMa'
            % Equivalent heights and breakpoint distance
            hBSp = hBS-hE;
            hUTp = hUT-hE;
            dBP = 4*hBSp.*hUTp*freq/c0;
            d2D = sqrt(d3D.^2-(hBS-hUT).^2);
            far = d2D>dBP; % Distance beyond break point
            
            % LOS path loss model before breakpoint
            PL1 = @(d3d) 28.0 + 22*log10(d3d) + 20*log10(fc);
            % LOS path loss model beyond breakpoint
            PL2 = @(d3d,dbp,hbs,hut) ...
                        28.0 + 40*log10(d3d) + 20*log10(fc) ...
                      - 9*log10(dbp.^2 + (hbs-hut).^2); 
            
            PL(~far) = PL1(d3D(~far));
            PL(far)  = PL2(d3D(far),dBP(far),hBS(far),hUT(far));
            SF(los) = 4;
            
            % NLOS path loss model
            if anyNLOS
                if ~pathLossConfig.OptionalModel
                    PLp = 13.54 + 39.08*log10(d3D)...
                          + 20*log10(fc) - 0.6*(hUT-1.5);
                    SF(nlos) = 6;
                else % Optional
                    PLp = 32.4 + 20*log10(fc) + 30*log10(d3D);
                    SF(nlos) = 7.8;
                end
                PL(nlos) = max(PL(nlos),PLp(nlos));
            end
            
        case 'UMi'            
            % Equivalent heights and breakpoint distance
            hBSp = hBS-hE;
            hUTp = hUT-hE;
            dBP = 4*hBSp.*hUTp*freq/c0;
            d2D = sqrt(d3D.^2-(hBS-hUT).^2);
            far = d2D>dBP; % Distance beyond break point
            
            % LOS path loss model
            PL1 = @(d3d) 32.4 + 21*log10(d3d) + 20*log10(fc);
            PL2 = @(d3d,dbp,hbs,hut)...
                    32.4 + 40*log10(d3d) + 20*log10(fc) ...
                   - 9.5*log10(dbp.^2 + (hbs-hut).^2);
            
            PL(~far) = PL1(d3D(~far));
            PL(far) = PL2(d3D(far),dBP(far),hBS(far),hUT(far));
            
            SF(los) = 4;
            
            % NLOS path loss model
            if anyNLOS
                if ~pathLossConfig.OptionalModel
                    PLp = 35.3*log10(d3D) + 22.4...
                          + 21.3*log10(fc) - 0.3*(hUT-1.5);
                    SF(nlos) = 7.82;
                else % Optional
                    PLp = 32.4 + 20*log10(fc) + 31.9*log10(d3D);
                    SF(nlos) = 8.2;
                end
                PL(nlos) = max(PL(nlos),PLp(nlos));
            end
    
        case 'InH'            
            % LOS path loss model
            PL = 32.4 + 17.3*log10(d3D) + 20*log10(fc);
            SF(los) = 3;
            
            % NLOS path loss model
            if anyNLOS
                if ~pathLossConfig.OptionalModel
                    PLp = 38.3*log10(d3D) + 17.30 + 24.9*log10(fc);
                    sf = 8.03;
                else % Optional
                    PLp = 32.4 + 20*log10(fc) + 31.9*log10(d3D);
                    sf = 8.29;
                end
                PL(nlos) = max(PL(nlos),PLp(nlos));
                SF(nlos) = sf;
            end
            
      otherwise % Indoor Factory: InF-SL, InF-DL, InF-SH, InF-DH, InF-HH
            % LOS path loss model
            PL = 31.84 + 21.5*log10(d3D) + 19*log10(fc);
            SF(los) = 4.3;
            
            % NLOS path loss model
            if anyNLOS
                switch scenario
                    case 'InF-SH'
                        PLp = 32.40 + 23.0*log10(d3D) + 20*log10(fc);
                        sf = 5.9;
                    case 'InF-DH'
                        PLp = 33.63 + 21.9*log10(d3D) + 20*log10(fc);
                        sf = 4;
                    otherwise % 'InF-SL' or 'InF-DL', as 'InF-HH' is not valid for NLOS
                        PLp = 33.00 + 25.5*log10(d3D) + 20*log10(fc); % PL InF-SL is used in InF-DL
                        sf = 5.7; % InF-SL
                        if strcmpi(scenario,'InF-DL')
                            PLDL = 18.60 + 35.7*log10(d3D) + 20*log10(fc);
                            PLp  = max(PLDL,PLp);
                            sf = 7.2;
                        end
                end
                PL(nlos) = max(PL(nlos),PLp(nlos));
                SF(nlos) = sf;
            end            
    end
    
    verifyPathLoss(PL,fc,los,posBS,posUE);

end

function [plc,freq,posBS,posUE] = validateInputs(pathLossCfg,freq,LOS,posBS,posUE)

    fcnName = 'nrPathLoss';
    
    validateattributes(pathLossCfg,{'nrPathLossConfig'},{'scalar'},fcnName,'PLC');
    validateattributes(freq,{'numeric'},{'scalar','real','positive','nonempty','nonnan'},fcnName,'CarrierFrequency');
    validateattributes(posBS,{'numeric'},{'nrows',3,'2d','real','nonempty','nonnan'},fcnName,'PositionBS');
    validateattributes(posUE,{'numeric'},{'nrows',3,'2d','real','nonempty','nonnan'},fcnName,'PositionUE');

    nBS = size(posBS,2);
    nUE = size(posUE,2);
    
    % Validate size of EnvironmentHeight for UMa and UMi
    if any(strcmpi(pathLossCfg.Scenario,{'UMi','UMa'}))
        he = pathLossCfg.EnvironmentHeight;
        if ~isscalar(he)
            validateattributes(he,{'numeric'},{'size',[nBS nUE]},fcnName,'EnvironmentHeight');
        end
    end
    
    % Validate size of LOS
    if isscalar(LOS)
        validateattributes(LOS,{'numeric','logical'},{'nonempty','nonnan'},fcnName,'LOS');
    else
        validateattributes(LOS,{'numeric','logical'},{'size',[nBS nUE],'nonempty','nonnan'},fcnName,'LOS');
    end

    % Validate LOS value for InF-HH
    errorFlag = strcmpi(pathLossCfg.Scenario,'InF-HH') && any(~LOS,'all');
    coder.internal.errorIf(errorFlag,'nr5g:nrPathLoss:IncorrectLOSInFHH');
    
    % Create a structure with the contents of the path loss config object.
    % The numeric properties are cast to double.
    plc.Scenario = pathLossCfg.Scenario;
    plc.BuildingHeight = double(pathLossCfg.BuildingHeight);
    plc.StreetWidth = double(pathLossCfg.StreetWidth);
    plc.EnvironmentHeight = double(pathLossCfg.EnvironmentHeight);
    plc.OptionalModel = pathLossCfg.OptionalModel;
    
    % Cast numeric inputs to double
    freq = double(freq);
    posBS = double(posBS);
    posUE = double(posUE);

end

% Verify that path loss is not negative or NaN
function verifyPathLoss(PL,fc,LOS,posBS,posUE)
    [ibs,iue] = find( PL<0 | isnan(PL) | isinf(PL) );
    n = numel(ibs);
    if n == 1
        iBS = ibs(1);
        iUE = iue(1);
        pl = PL(iBS,iUE);
        los = LOS(iBS,iUE);
        bs = posBS(:,iBS)';
        ue = posUE(:,iUE)';
        strPosBS = getString(bs,';');
        strPosUE = getString(ue,';');
        coder.internal.warning('nr5g:nrPathLoss:IncorrectPathLoss', ...
                    getString(pl),strPosBS(1:end-1),strPosUE(1:end-1),...
                    sprintf('%g',fc),sprintf('%d',int8(los)));
    elseif n>1
        pairs = '';
        for i=1:n
            pairs = [pairs sprintf('(%d,',int64(ibs(i))) sprintf('%d)',int64(iue(i)))]; %#ok<AGROW>
        end
        coder.internal.warning('nr5g:nrPathLoss:IncorrectPathLossMatrix',length(ibs),pairs(:)',sprintf('%g',fc));
    end
end
    
function str = getString(x,sep)
    if nargin < 2
        sep = '';
    end
    str = '';
    for i = 1:numel(x)
        str = [str sprintf('%g%s',double(x(i)),sep)]; %#ok<AGROW> 
    end
end