function seq = PRBS(cinit,n)
%PRBS Pseudorandom binary sequence
%
%   Note: This is an internal undocumented function and its API and/or
%   functionality may change in subsequent releases.
%
%   SEQ = PRBS(CINIT,N) returns vector SEQ containing the first N elements
%   of the pseudorandom binary sequence (PRBS) generator defined in TS
%   38.211 Section 5.2.1, when initialized with 31-bit integer CINIT.

%   Copyright 2018-2021 The MathWorks, Inc.

%#codegen
    
    % Polynomials and masks for the pair of shift registers
    % 9 = [1 0 0 1], 15 = [1 1 1 1] (both arranged msb->lsb)
    poly = uint32([9 15]);
    
    % Register initialization
    reg = uint32([1 cinit]);
    
    % Pre-computed output masks for 1600 shift initialization
    masks = uint32([35263098 10031374]);
    
    % Memory to store shift register value across the sequence
    seqpair = zeros(n,2,'uint32');
    
    % Parity table
    Parity15 = uint32(bitshift(1,31-1)*[0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0]);
         
    % Run the shift registers
    for i=1:n
        
        % Store the masked register values
        seqpair(i,:) = bitand(reg,masks);
        
        % Top bit of 32 bit shift register
        feedback = Parity15(1 + bitand(reg,poly));
        
        % Shift registers down to the right
        reg = bitshift(reg,-1);
        
        % Then shift in top bit
        reg = bitxor(reg,feedback);
        
    end
    
    % Parity table for output masking
    Parity = uint32([...
        0 1 1 0 1 0 0 1 1 0 0 1 0 1 1 0 1 0 0 1 0 1 1 0 0 1 1 0 1 0 0 1 ...
        1 0 0 1 0 1 1 0 0 1 1 0 1 0 0 1 0 1 1 0 1 0 0 1 1 0 0 1 0 1 1 0 ...
        1 0 0 1 0 1 1 0 0 1 1 0 1 0 0 1 0 1 1 0 1 0 0 1 1 0 0 1 0 1 1 0 ...
        0 1 1 0 1 0 0 1 1 0 0 1 0 1 1 0 1 0 0 1 0 1 1 0 0 1 1 0 1 0 0 1 ...
        1 0 0 1 0 1 1 0 0 1 1 0 1 0 0 1 0 1 1 0 1 0 0 1 1 0 0 1 0 1 1 0 ...
        0 1 1 0 1 0 0 1 1 0 0 1 0 1 1 0 1 0 0 1 0 1 1 0 0 1 1 0 1 0 0 1 ...
        0 1 1 0 1 0 0 1 1 0 0 1 0 1 1 0 1 0 0 1 0 1 1 0 0 1 1 0 1 0 0 1 ...
        1 0 0 1 0 1 1 0 0 1 1 0 1 0 0 1 0 1 1 0 1 0 0 1 1 0 0 1 0 1 1 0]);
    
    % Output masking to implement the shift
    % Shift down 16 and combine
    ts = bitxor(seqpair,bitshift(seqpair,-16));
    
    % Shift down 8 and combine
    ts = bitxor(ts,bitshift(ts,-8));
    
    % Mask lower 8 bits
    ts = bitand(ts,255);
    
    % Look up parity value for these 8 bits
    pseq = Parity(1+ts);
    
    % Combine (xor) together the two PN sequences
    seq = xor(pseq(:,1),pseq(:,2));
    
end
