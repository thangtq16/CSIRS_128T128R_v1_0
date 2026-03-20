function H = polarInterpolate(nd,ipt,k,n,H0,method,varargin)
%POLARINTERPOLATE interpolates channel estimate 'H0' in polar co-ordinates,
% using interpolant 'ipt'. 'H' contains the interpolated channel estimate
% for subcarrier subscripts 'k' and OFDM symbol subscripts 'n'. 'nd' is the
% number of dimensions over which the interpolation is performed and
% 'method' is the interpolation method.
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    [theta_in,r_in] = cart2pol(real(H0),imag(H0));
    theta_in = unwrap(theta_in);
    theta_out = nr5g.internal.nrChannelEstimate.cartInterpolate(nd,ipt,k,n,theta_in,method,varargin{:});
    r_out = nr5g.internal.nrChannelEstimate.cartInterpolate(nd,ipt,k,n,r_in,method,varargin{:});
    [x,y] = pol2cart(theta_out,r_out);
    H = complex(x,y);
    
end