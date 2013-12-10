function [result] = normDegrees(input)

if (input < 0) 
    input += 360;
elseif (input >= 360) 
    input -= 360;
end

result = input;

endfunction