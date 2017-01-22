function [theta] = interpolate_theta(theta_i,theta_f,Tf,L)
% This function gives the interpolation from initial to final point 

% Coded by Hemanth Manjunatha on Nov-20-2015

% Nomenclature used
% xi -> Initial x co-ordiante
% xf -> Final x co-ordiante
% yi -> Initial y co-ordiante
% yi -> Final y co-ordiante

% Variable initialisation
t = linspace(0,Tf,100);
theta = deal(zeros(length(t),1));

if ~exist('L','var') 
    L = 'linear';
end
    switch(L)
        case 'linear'
            % Using standard linear algebra y = mx + c

            % Finding the c coefficients. 
                 c = theta_i;  
            % Finding the slope m @ t = Tf. 
                 m = (theta_f - theta_i)/Tf;
            % changes have been made in for loop, since indexing in MATLAB starts
            % from 1
            for i = 0:length(t)-1
            % Get the co-ordinates for each time instance
                theta(i+1,1) = m*t(i+1)+ c;
            end
            
        case 'cubic'
            % Using standard linear algebra to find constants
                const = [0 0 0 1; 0 0 1 0; Tf^3 Tf^2 Tf 1 ; 3*Tf^2 2*Tf 1 0]\([theta_i 0 theta_f 0]');

            for i = 0:length(t)-1
             % Get the co-ordinates for each time instance
                theta(i+1,1) = (t(i+1).^(3:-1:0))*const;
            end
   
        case 'quintic'
            % Using standard linear algebra to find constants
                const = [0 0 0 0 0 1; 0 0 0 0 1 0; 0 0 0 2 0 0; 
                Tf^5 Tf^4 Tf^3 Tf^2 Tf 1; 5*Tf^4 4*Tf^3 3*Tf^2 2*Tf 1 0;
                20*Tf^3 12*Tf^2 6*Tf 2 0 0]\([theta_i 0 0 theta_f 0 0]');
            
            for i = 0:length(t)-1
             % Get the co-ordinates for each time instance
                theta(i+1,1) = (t(i+1).^(5:-1:0))*const;
            end
    end
   
    plot(t,theta)
    grid on

end


