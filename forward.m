function [x,y] = forward(A,B,T,D)
% This function gives end pointer position given the configuration of the
% robot leg

% Coded by Hemanth Manjunatha on Nov-20-2015

% Nomenclature used
% x -> x co-ordiante
% y -> y co-ordiante
% d -> extension of leg
% t -> angle of leg (orientation with respect to ground)
switch(T)
    case 'dt'% extension and angle is given
        disp('dt')
        d = A;
        t = B;
        [n,m] = leg(D);
        [xb1,yb1] = base(n);% Function to get the base point of the leg
        [xb2,yb2] = base(m);
        A = 1+ tand(t)^2;
        B = -2*xb2-2*xb1*tand(t)^2 + 2*tand(t)*(yb1-yb2);
        C = xb2^2 + xb1^2*tand(t)^2 -2*tand(t)*xb1*(yb1-yb2) + (yb1-yb2)^2-d^2;
        p = roots([A,B,C]);
        if xb1 == 0 || xb2 == 0
           x = max(p);
        else
           x = min(p);
        end
        y = tand(t)*(x-xb1)+ yb1;
            
    case 'dd'
        disp('dd')
        [n,m] = leg(D);
        d1 = A;
        d2 = B;
        [xb1,yb1] = base(n);% Function to get the base point of the leg
        [xb2,yb2] = base(m);
            D = sqrt((xb2-xb1)^2 + (yb2-yb1)^2);
            if xb2 >= xb1
            Delta= -sqrt((D+d1+d2)*(D+d1-d2)*(D-d1+d2)*(-D+d1+d2));
            else
            Delta= +sqrt((D+d1+d2)*(D+d1-d2)*(D-d1+d2)*(-D+d1+d2));
            end 
            x = (xb1+xb2)/2 + (xb2-xb1)*(d1-d1)/(2*D^2) + 2*(yb1-yb2)/(D^2)*Delta;
            y = (yb1+yb2)/2 + (yb2-yb1)*(d1-d1)/(2*D^2) + 2*(xb1-xb2)/(D^2)*Delta;
            
    case 'tt'
        disp('tt')
        [n,m] = leg(D);
        t1 = A;
        t2 = B;
        [xb1,yb1] = base(n);% Function to get the base point of the leg
        [xb2,yb2] = base(m);
            x = (yb2-yb1 + xb1*tand(t1) -xb2*tand(t2))/(tand(t1)-tand(t2));
            y = tand(t1)*(x-xb1) + yb1;
    case'td'
        disp('td')
        t = A;
        d = B;
        [n,m] = leg(D);
        [xb1,yb1] = base(m);% Function to get the base point of the leg
        [xb2,yb2] = base(n);
        A = 1 + tand(t)^2;
        B = -2*xb2-2*xb1*tand(t)^2 + 2*tand(t)*(yb1-yb2);
        C = xb2^2 + xb1^2*tand(t)^2 -2*tand(t)*xb1*(yb1-yb2) + (yb1-yb2)^2-d^2;
        p = roots([A,B,C]);
        if xb1 == 2 || xb2 == 2
            x = min(p);
        else
            x = max(p);
        end
        y = tand(t)*(x-xb2)+ yb2;
end
  

end

