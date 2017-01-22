function theta_dot = theta_solve_open_loop_joint_space(t, theta, phi_dot, major_axis, minor_axis, i_dot, j_dot)

% Indipendant is d1_dot & d2_dot as of now.


% theta = [d1;      1
%          theta1;  2
%          d2;      3
%          theta2;  4
%          d3;      5
%          theta3;  6
%          d4;      7
%          theta4]  8

JA = [cosd(theta(2)) -theta(1)*sind(theta(2));
      sind(theta(2))  theta(1)*cosd(theta(2))]
 
JB = [cosd(theta(4)) -theta(3)*sind(theta(4));
      sind(theta(4))  theta(3)*cosd(theta(4))];
  
JC = [cosd(theta(6)) -theta(5)*sind(theta(6));
      sind(theta(6))  theta(5)*cosd(theta(6))];
  
JD = [cosd(theta(8)) -theta(7)*sind(theta(8));
      sind(theta(8))  theta(7)*cosd(theta(8))];
 
zeroM = zeros(2,2)  
J = [JA -JB     zeroM   zeroM;
     JA zeroM   -JC     zeroM;
     JA zeroM   zeroM   -JD]
 
switch i_dot 
    case 'd1_dot'
        iI(1) = 1
    case 'theta1_dot'
        iI(1) = 2
    case 'd2_dot'
        iI(1) = 3
	case 'theta2_dot'
        iI(1) = 4
    case 'd3_dot'
        iI(1) = 5
	case 'theta3_dot'
        iI(1) = 6
    case 'd4_dot'
        iI(1) = 7
	case 'theta4_dot'
        iI(1) = 8
end

switch j_dot 
    case 'd1_dot'
        iI(2) = 1
    case 'theta1_dot'
        iI(2) = 2
    case 'd2_dot'
        iI(2) = 3
	case 'theta2_dot'
        iI(2) = 4
    case 'd3_dot'
        iI(2) = 5
	case 'theta3_dot'
        iI(2) = 6
    case 'd4_dot'
        iI(2) = 7
	case 'theta4_dot'
        iI(2) = 8
end

iD = 1:8;
iD = iD(iD ~= iI(1))
iD = iD(iD ~= iI(2))

JI = J(:,iI)
JD = J(:,iD)

Jm = -JD\JI

S = zeros(8,2)

S(iI(1),1) = 1
S(iI(2),2) = 1

k = 1;
for i = 1:8
    if i ~= iI(1)
        if i ~=iI(2)
            S(i,:) = Jm(k,:)
            k = k+1;
        end
    end
end
    
JAI = [S(1,:);
       S(2,:)];
% Jmanipubility = JA*JAI % use this as the jacobian in the work space analysis

Jphi = [-major_axis*sind(phi_dot*t);
         minor_axis*cosd(phi_dot*t)];
     
Jinv = JA*JAI;
theta_dot = S*(Jinv\Jphi)*phi_dot