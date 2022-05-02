function [X, P] = constVelTU(Xin, Pin, Q, dt)
%{
    Simple constant velocity (ECEF) time update for GPS final project

    X^e = [x, y, z, xDot, yDot, zDot, clkBias, clkDrift]

    Based on 9.4.2.2 in Groves Principles of GNSS, INSS... textbook
%}

z3 = zeros(3,3);
i3 = eye(3,3);
z31 = zeros(3,1);

% Groves 9.148
F = [z3     i3      z31     z31;
     z3     z3      z31     z31;
     z31'    z31'     0       1;
     z31'    z31'     0       1];  

 % discretize state transition matrix
Phi = eye(8) + F*dt;


% update states
X = Phi*Xin;

% update covariance
P = Phi*Pin*Phi.' + Q;

end

