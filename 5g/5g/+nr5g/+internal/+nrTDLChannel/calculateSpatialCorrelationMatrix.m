function Rspat = calculateSpatialCorrelationMatrix(P,Rt,Rr,Gamma,a)
%calculateSpatialCorrelationMatrix calculate spatial correlation matrix
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%   Copyright 2017-2024 The MathWorks, Inc.

    % Compute overall spatial correlation matrix. Rspat is of size
    % (Nt*Nr)-by-(Nt*Nr)(-by-Np)

    % Calculate Nt*Nr. Notice that P, Rt and Rr have different sizes in
    % co-polar case and cross-polar case, so Nt*Nr needs to be calcualted
    % separately
    if isscalar(P)
        % In co-polar case, P is 1. Need to extract Nt*Nr from Rt and Rr
        Ntr = size(Rt,1)*size(Rr,1);
    else
        % In cross-polar case, P is of the size (Nt*Nr)-by-(Nt*Nr)
        Ntr = size(P,1);
    end
    
    Np = max([size(Rt,3) size(Rr,3) size(Gamma,3)]);
    if (Np>1)
        % If any of (Rt, Rr, Gamma) is 3-D i.e. defines a
        % matrix per path, repeat the other matrices for
        % uniformity
        if (size(Rt,3)==1)
            Rt = repmat(Rt,[1 1 Np]);
        end
        if (size(Rr,3)==1)
            Rr = repmat(Rr,[1 1 Np]);
        end
        if (size(Gamma,3)==1)
            Gamma = repmat(Gamma,[1 1 Np]);
        end
    end
    Rspat = zeros([Ntr Ntr Np]);
    for p = 1:Np
        Rspat(:,:,p) = P*kron(kron(Rt(:,:,p),Gamma(:,:,p)),Rr(:,:,p))*P.';
    end

    % Adjustment to make Rspat positive semi-definite (if necessary) as
    % defined in TS 36.101 / TS 36.104 Annex B
    if (a~=0)
        for p = 1:Np
            Rspat(:,:,p) = (Rspat(:,:,p) + a*eye(size(Rspat)))/(1+a);
        end
    end
    
end
