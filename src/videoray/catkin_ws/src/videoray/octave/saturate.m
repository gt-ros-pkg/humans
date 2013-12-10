function [result] = saturate(input, low_range, high_range)
% Saturates input based on low_range and high_range

if low_range > high_range
    error("incorrect config: low_range is greater than high_range");
end

if input < low_range
    input = low_range;
elseif input > high_range
    input = high_range;
end

result = input;

endfunction