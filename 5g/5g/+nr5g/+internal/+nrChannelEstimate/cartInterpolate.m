function H = cartInterpolate(nd,ipt,k,n,H0,method,varargin)
%CARTINTERPOLATE Iinterpolates channel estimate 'H0' in Cartesian
% co-ordinates (or real values), using interpolant 'ipt'. 'H' contains the
% interpolated channel estimate for subcarrier subscripts 'k' and OFDM
% symbol subscripts 'n'. 'nd' is the number of dimensions over which the
% interpolation is performed and 'method' is the interpolation method.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    % If the specified method is 'VP', this indicates that the
    % interpolation is being performed for the virtual pilots. In this
    % case, the channel estimate 'H0' is pre-processed to give a best-fit
    % linear interpolation (to remove noise) prior to linearly
    % extrapolating to calculate the VPs
    isVP = strcmp(method,'VP');
    
    % Perform interpolation
    if (nd==1)
        if (isempty(coder.target))
            % Pre-process VPs if applicable
            if (isVP)
                H0 = preprocessVPs(ipt.GridVectors{1},H0);
            end
            % Perform 1-D interpolation using griddedInterpolant
            ipt.Values = H0;
            H = ipt(k);
        else
            % Pre-process VPs if applicable
            if (isVP)
                H0 = preprocessVPs(ipt(:,1),H0);
                method1 = 'linear';
            else
                method1 = method;
            end
            % Perform 1-D interpolation using interp1
            H = interp1(ipt,H0,k,method1,'extrap');
        end
    else % nd==2
        if (isempty(coder.target) && ~isnumeric(ipt))
            % Perform 2-D gridded interpolation using griddedInterpolant 
            ipt.Values = H0;
            H = ipt({k n});
        else
            % Perform 2-D scattered interpolation
            % 'ipt' contains a sample points array for scattered
            % interpolation
            x = ipt(:,1);
            y = ipt(:,2);
            % Find the locations 'yn' of estimates for the current OFDM
            % symbol 'n'
            yn = find(y==n);
            % Pre-process VPs if applicable
            if (isVP)
                H0(yn,:,:) = preprocessVPs(x(yn),H0(yn,:,:),varargin{:});
            end
            % If there are multiple estimates for the current OFDM
            % symbol
            if (~isscalar(yn))
                % Interpolate the estimates for each subscript in 'k'
                H = interp1(x(yn),H0(yn,:,:),k,'linear','extrap');
            else
                % Repeat the single estimate for all subscripts 'k'
                H = repmat(H0(yn,:,:),[size(k) 1 1]);
            end
        end
    end
    
end

%% Local functions

% Perform a linear fit of the channel estimate 'H' whose subcarrier
% subscripts are 'k'
function Hout = preprocessVPs(k,H,varargin)

    nG = size(H,3);
    if nargin<3
        prgStarts = ones(nG,1);
    else
        prgStarts = varargin{1};
    end
    Hout = H;
    if (numel(k)>1)
        for g = 1:nG
            % When processing multiple PRGs, shift back to its absolute
            % position for accuracy
            kThis = k+prgStarts(g)-1;
            p = polyfit(kThis,H(:,1,g),1);
            Hout(:,1,g) = polyval(p,kThis);
        end
    end

end
