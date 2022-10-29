function [C_NtoB, C_BtoN] = NED_to_Body(Eul)

% With the Euler angles inputs, the rotation matrix between the body and
% nav frame (or geographic... simply requires an inertial frame)can be constructed. 
% This allows for accelerations in body frame to be transformed into our
% navigation frame of interest and vice versa

% phi - roll, theta - pitch, psi - yaw

% 3 step process: ZYX Euler Angles, slightly different than was done in
% class. I am matching Archit's thesis and Zipfel (neg in different
% locations)
% represents rotating abt Z then Y then X

phi = Eul(1);
theta = Eul(2);
psi = Eul(3);


T_roll = [1 0 0;
          0 cosd(phi) sind(phi);
          0 -sind(phi) cosd(phi)];
      

T_pitch = [cosd(theta) 0 -sind(theta)
          0 1 0;
          sind(theta) 0 cosd(theta)];
      
T_yaw = [cosd(psi) sind(psi) 0;
        -sind(psi) cosd(psi) 0;
        0 0 1];
      

    
C_NtoB = T_roll * T_pitch * T_yaw;

      
C_BtoN = C_NtoB.'; 
end