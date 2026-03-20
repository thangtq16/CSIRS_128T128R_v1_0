function H = despreadTDCDM(H,un,tdCDM,K,eK,R)
%DESPREADTDCDM performs the TD-CDM despreading on LS estimates
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2024 The MathWorks, Inc.

%#codegen

    % 'm' is zero if the estimates divide evenly into TD-CDM groups,
    % otherwise 'm' is the number of estimates in the final partial group
    m = mod(numel(un),tdCDM);

    for a = 0:double(m~=0)

        if (~a)
            % whole CDM lengths (may be empty)
            unCDM = un(1:end-m);
            nlCDM = tdCDM;
        else
            % part CDM length (may be empty)
            unCDM = un(end-m+1:end);
            nlCDM = m;
        end

        for r = 1:R

            % Extract the estimates and reshape so that each row and plane
            % contains a TD-CDM group
            x = reshape(H(:,unCDM,r),K+eK,nlCDM,[]);

            % 'mx' is the mean of 'x' across the 2nd dimension (i.e. across
            % the TD-CDM group), excluding zeros
            mx = sum(x,2) ./ max(sum(x~=0,2),1);

            % Repeat the averaged value and reshape back into a matrix
            x = repmat(mx,[1 nlCDM]);
            H(:,unCDM,r) = reshape(x,[],numel(unCDM));

        end

    end

end