%precodeIsAllPlanes Establish if the RE indices refer to the same locations in all planes
%    
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.

%  Copyright 2023 The MathWorks, Inc.

%#codegen

function allplanes = precodeIsAllPlanes(siz,ind,maxp)

    % Get the number of resource element indices
    NRE = size(ind,1);

    if (NRE==0)

        % For the case of NRE=0, indicate allplanes=true in order to take
        % the most efficient code paths during precoding, and to avoid
        % issues with empty indices in the 'else' clause below. Note that
        % the value of 'allplanes' is not subsequently relevant as there
        % are no REs to process
        allplanes = true;

    elseif (prod(size(ind,[2 3]))==1)

        % If all indices are in a single column then either the indices are
        % for a single layer, or assume that the indices contain different
        % numbers of REs per layer. In the former case the value of
        % 'allplanes' is true and in the latter case 'allplanes' is false
        allplanes = (maxp==1);

    else

        % Get the number of subcarriers and OFDM symbols
        K = siz(1);
        N = siz(2);

        % Reshape 'ind' into a matrix of size NRE-by-M (the input 'ind' can
        % be 2-D or 3-D)
        ind = reshape(ind,NRE,[]);

        % 'allplanes' is true if the RE indices refer to the same locations
        % in all planes, that is, if [k,n,p] = ind2sub(siz,ind) has the
        % same value of 'k' and 'n' in each column of a given row, and this
        % is true of all rows. Considering a single row of indices 
        % r = ind(i,:), this condition will be true of that row if 'r' is
        % of the form k1 + n1*K + X(i,:)*K*N where 'k1' and 'n1' are the
        % first elements of 'k' and 'n' for that row, and X is a matrix of
        % integers.

        % For each RE (i.e. each row of 'ind'), calculate the difference
        % across the columns. 'd' is a matrix of size NRE-by-(M-1). Each
        % row of 'd' corresponds to one RE, with column 'j' indicating the
        % change in index value for that RE between plane 'j' and plane 
        % 'j'+1. 
        d = diff(ind,[],2);
        
        % 'allplanes' is true if for all elements (i,j) of 'd', 
        % mod(d(i,j),K*N)==0. This is because for a set of indices for
        % which 'allplanes' should be true, the following holds:
        % d(i,j) =   (k1(i) + n1(i)*K + X(i,j+1)*K*N))
        %          - (k1(i) + n1(i)*K + X(i,j)*K*N)
        %        = (X(i,j+1)-X(i,j))*K*N
        % => mod(d(i,j),K*N) = mod((X(i,j+1)-X(i,j))*K*N,K*N) = 0

        % Check that the condition above is satisfied for the first row of
        % 'd', d(1,:), and then check that every other row of 'd' is the
        % same as d(1,:). This is equivalent to the desired 'allplanes'
        % check plus the restriction that the rows of X are same for every
        % row of indices. The matrices of indices produced by 5G Toolbox
        % functions place all indices for a given plane in the same column,
        % that is, 'p' defined above has the same value in every row of a
        % given column. Therefore p(1,:) = p(i,:) for any row 'i'. row 'i'
        % of 'ind' can be expressed as:
        % ind(i,:) = k1 + n1*K + X(i,:)*K*N 
        %          = k1 = n1*K + p(i,:)*K*N
        %          = k1 + n1*K + p(1,:)*K*N
        % therefore X(i,:) = p(1,:) showing that the restriction "the rows
        % of X are same for every row of indices" does hold for typical 5G
        % Toolbox index sets. (In the unlikely case that an index set is
        % created for which this restriction does not hold but the index
        % set is truly 'allplanes', allplanes will be set to false and the
        % optimized allplanes=true code path during precoding will not run,
        % but the correct precoding operation will still be performed)
        allplanes = all(mod(d(1,:),K*N)==0) && all(d==d(1,:),'all');

    end

end
