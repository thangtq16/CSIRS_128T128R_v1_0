function H = interpolateGrid(nd,k0,l0,H0,k,n,method,polar)
%INTERPOLATEGRID interpolates channel estimate 'H0' defined for subcarrier
% subscripts 'k0' and OFDM symbol subscripts 'l0', using 'method' as the
% interpolation method. 'nd' is the number of dimensions over which
% interpolation is performed. 'polar' determines whether interpolation is
% performed using polar (true) or Cartesian (false) co-ordinates. 'H'
% contains the interpolated channel estimate for subcarrier subscripts 'k'
% and OFDM symbol subscripts 'n'.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen
    
    if (isempty(coder.target))
        if (nd==1)
            gi = griddedInterpolant(k0,zeros(size(H0)),method);
        else % nd==2
            gi = griddedInterpolant({k0 l0},zeros(size(H0)),method);
        end
    else
        if (nd==1)
            gi = k0;
        else % nd==2
            % For code generation, interp2 is used for 2-D interpolation.
            % Unlike griddedInterpolant, interp2 does not perform
            % extrapolation. Therefore, explicit extrapolation is performed
            % here for the first and last OFDM symbol, unless they already
            % appear in 'l0'
            if (~any(n(1)==l0))
                H0 = [extrapolate(l0,H0,n(1),method) H0];
                l0 = [n(1) l0];
            end
            if (~any(n(end)==l0))
                H0 = [H0 extrapolate(l0,H0,n(end),method)];
                l0 = [l0 n(end)];
            end
            [lg,kg] = meshgrid(l0,k0);
            % For code generation, perform 2-D Cartesian interpolation.
            % This relies on the (accurate) assumption that interpolate()
            % is only called with polar=false for nd==2. The interp2 call
            % requires to be here rather than inside cartInterpolate in
            % order that coder does not attempt to generate code for
            % interp2 inside a call to createVPs, where input 'n' is scalar
            % and consequently fails size checks performed by interp2
            [yq,xq] = meshgrid(n,k);
            H = interp2(lg,kg,H0,yq,xq,method);
            return;
        end
    end
    
    if (polar)
        H = nr5g.internal.nrChannelEstimate.polarInterpolate(nd,gi,k,n,H0,method);
    else
        H = nr5g.internal.nrChannelEstimate.cartInterpolate(nd,gi,k,n,H0,method);
    end

end

%% Local functions

% Extrapolates channel estimate 'H0' in Cartesian co-ordinates. Each row of
% 'H' contains the extrapolated channel estimate for OFDM subscript 'l0',
% with the corresponding row of 'H0' containing estimates for OFDM symbol
% subscripts 'l0'. 'method' is the extrapolation method
function H = extrapolate(l0,H0,l,method)

    nk = size(H0,1);
    H = zeros([nk 1],'like',H0);
    for i = 1:nk
        H(i) = interp1(l0,H0(i,:),l,method,'extrap');
    end
    
end