function y = check_circle(x,y)
% This function checks the feasiblity of a point

if (x-xb1)^2 + (y-yb1)^2 < dmax1^2
    if (x-xb2)^2 + (y-yb2)^2 < dmax2^2
        if (x-xb3)^2 + (y-yb3)^2 < dmax3^2
            if (x-xb4)^2 + (y-yb4)^2 < dmax4^2
                y = 1;
            end
        end
    end
end
            
end

