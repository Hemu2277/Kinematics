function theta_dot = theta_solve_control(t, theta, phi_dot, major_axis, minor_axis, i_dot, j_dot,control_type,k_gain,a,b,rO_V)
% Indipendant is d1_dot & d2_dot as of now.

% theta = [d1;      1
%          theta1;  2
%          d2;      3
%          theta2;  4
%          d3;      5
%          theta3;  6
%          d4;      7
%          theta4]  8


 

JA = [cos(theta(2)) -theta(1)*sin(theta(2));
      sin(theta(2))  theta(1)*cos(theta(2))];
 
JB = [cos(theta(4)) -theta(3)*sin(theta(4));
      sin(theta(4))  theta(3)*cos(theta(4))];
  
JC = [cos(theta(6)) -theta(5)*sin(theta(6));
      sin(theta(6))  theta(5)*cos(theta(6))];
  
JD = [cos(theta(8)) -theta(7)*sin(theta(8));
      sin(theta(8))  theta(7)*cos(theta(8))];
 
zeroM = zeros(2,2);
J = [JA -JB     zeroM   zeroM;
     JA zeroM   -JC     zeroM;
     JA zeroM   zeroM   -JD];
 
switch i_dot 
    case 'd1_dot'
        iI(1) = 1;
    case 'theta1_dot'
        iI(1) = 2;
    case 'd2_dot'
        iI(1) = 3;
	case 'theta2_dot'
        iI(1) = 4;
    case 'd3_dot'
        iI(1) = 5;
	case 'theta3_dot'
        iI(1) = 6;
    case 'd4_dot'
        iI(1) = 7;
	case 'theta4_dot'
        iI(1) = 8;
end

switch j_dot 
    case 'd1_dot'
        iI(2) = 1;
    case 'theta1_dot'
        iI(2) = 2;
    case 'd2_dot'
        iI(2) = 3;
	case 'theta2_dot'
        iI(2) = 4;
    case 'd3_dot'
        iI(2) = 5;
	case 'theta3_dot'
        iI(2) = 6;
    case 'd4_dot'
        iI(2) = 7;
	case 'theta4_dot'
        iI(2) = 8;
end

iD = 1:8;
iD = iD(iD ~= iI(1));
iD = iD(iD ~= iI(2));

JI = J(:,iI);
JD = J(:,iD);

Jm = -inv(JD)*JI;

S = zeros(8,2);

S(iI(1),1) = 1;
S(iI(2),2) = 1;

k = 1;
for i = 1:8
    if i ~= iI(1)
        if i ~=iI(2)
            S(i,:) = Jm(k,:);
            k = k+1;
        end
    end
end
    
JAI = [S(1,:);
       S(2,:)];
% Jmanipubility = JA*JAI % use this as the jacobian in the work space analysis


Xe_desired = [a + major_axis*cos(phi_dot*t);
              b + minor_axis*sin(phi_dot*t)];

Jphi = [-major_axis*sin(phi_dot*t);
         minor_axis*cos(phi_dot*t)];
     
 switch control_type
     case 'Open_loop'     
        theta_dot = S*inv(JA*JAI)*Jphi*phi_dot;
     case 'Closed_loop_joint'
         theta_desired = control_inverse(Xe_desired,rO_V); % returns in radians!
         theta_dot = S*inv(JA*JAI)*Jphi*phi_dot + k_gain.*(theta_desired - theta);
     case 'Closed_loop_task'
         [x,y] = RPR3_ForwardKinetics2(rad2deg(theta(2)), theta(1), rO_V);
         Xe_actual = [x;y];
         theta_dot = S*inv(JA*JAI)*(Jphi*phi_dot + k_gain.*(Xe_desired - Xe_actual));
         
 end
 

%  disp(theta_dot)
%  pause
 return
 