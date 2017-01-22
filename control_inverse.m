function theta = control_inverse(Xe,rO_V)
x = Xe(1);
y = Xe(2);
[d1, theta1] = RPR3_InverseKinetics2(x, y, rO_V(1,:));
[d2, theta2] = RPR3_InverseKinetics2(x, y, rO_V(2,:));
[d3, theta3] = RPR3_InverseKinetics2(x, y, rO_V(3,:));
[d4, theta4] = RPR3_InverseKinetics2(x, y, rO_V(4,:));

theta =  [d1;      
          deg2rad(theta1);  
          d2;      
          deg2rad(theta2);  
          d3;      
          deg2rad(theta3);  
          d4;      
          deg2rad(theta4)];