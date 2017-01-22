function [x,y,t] = interpolate(xi,yi,xf,yf,Tf,L)
% This function gives the interpolation from initial to final point 

% Coded by Hemanth Manjunatha on Nov-20-2015

% Nomenclature used
% xi -> Initial x co-ordiante
% xf -> Final x co-ordiante
% yi -> Initial y co-ordiante
% yi -> Final y co-ordiante

% Variable initialisation
t = linspace(0,Tf,100);
[x,y] = deal(zeros(length(t),1));

if ~exist('L','var') 
    L = 'linear';
end
    switch(L)
        case 'linear'
            % Using standard linear algebra to find a, b

            % Finding the coefficients for x-coordiante. 
                 abx = deal([0 1;Tf 1]\[xi;xf]);  
            % Finding the coefficients for y-coordiante. 
                 aby = deal([0 1;Tf 1]\[yi;yf]);
            % changes have been made in for loop, since indexing in MATLAB starts
            % from 1
            for i = 0:length(t)-1
            % Get the co-ordinates for each time instance
                x(i+1,1) = abx(1)*t(i+1)+abx(2);% x co-ordinates
                y(i+1,1) = aby(1)*t(i+1)+aby(2);% y co-ordinates
            end
            
        case 'cubic'
            % Using standard linear algebra to find a, b, c, d, e, f
                abx = [0 0 0 1; 0 0 1 0; Tf^3 Tf^2 Tf 1 ; 3*Tf^2 2*Tf 1 0]\([xi 0 xf 0]');

                aby = [0 0 0 1; 0 0 1 0; Tf^3 Tf^2 Tf 1 ; 3*Tf^2 2*Tf 1 0]\([yi 0 yf 0]');

            for i = 0:length(t)-1
             % Get the co-ordinates for each time instance
                x(i+1,1) = (t(i+1).^(3:-1:0))*abx;% x co-ordinates
                y(i+1,1) = (t(i+1).^(3:-1:0))*aby;% y co-ordinates
            end
   
        case 'quintic'
            % Using standard linear algebra to find a, b, c, d, e, f
                abx = [0 0 0 0 0 1; 0 0 0 0 1 0; 0 0 0 2 0 0; 
                Tf^5 Tf^4 Tf^3 Tf^2 Tf 1; 5*Tf^4 4*Tf^3 3*Tf^2 2*Tf 1 0;
                20*Tf^3 12*Tf^2 6*Tf 2 0 0]\([xi 0 0 xf 0 0]');

                aby = [0 0 0 0 0 1; 0 0 0 0 1 0; 0 0 0 2 0 0; 
                Tf^5 Tf^4 Tf^3 Tf^2 Tf 1; 5*Tf^4 4*Tf^3 3*Tf^2 2*Tf 1 0;
                20*Tf^3 12*Tf^2 6*Tf 2 0 0]\([yi 0 0 yf 0 0]');

            for i = 0:length(t)-1
             % Get the co-ordinates for each time instance
                x(i+1,1) = (t(i+1).^(5:-1:0))*abx;% x co-ordinates
                y(i+1,1) = (t(i+1).^(5:-1:0))*aby;% y co-ordinates
            end
    end
    
    
end


