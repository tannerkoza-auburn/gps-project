function [p_eb_e, v_eb_e, C_b2e] = Mechanize_ECEF(f_ib_b, omega_ib_b, dt, old_PVA)

%{ 
    Mechanization of IMU accelerometer and gyro measurements. Selects
    higher fidelity update if magnitude of angular rate exceeds preset 
    maximum. Intended for GA collaborative navigation simulation. 

    Ben Jones and Will Kennedy
    3/31/21

    Source: 
    P. Groves, "Principles of GNSS Inertial and Multisensor Integrated
    Navigation Systems." 2nd Edition.
%}

%______________________ Parameters and initialization ____________________%

omega_ie = [0; 0; 7.292115*10^(-5)]; % (rad/s) rotation rate of Earth
Omega_ie = Skew(omega_ie); % Skew of Earth Rotation

% deconstruct struct (haha)
pos_in = old_PVA.pos(:,end);
vel_in = old_PVA.vel(:,end);
C_b2e_in = old_PVA.C_b2e;

% gravity model:
g_ECEF = GravModel_ECEF(pos_in);




%____________________________Atttitude Update_____________________________%
% Precision update:  Eq. 5.75
% C_b2e(+) = C_b2e(-) * C_b(+)^b(-)  -  Omega_ie*C_b(+)^b(-)*dt
% Where C_(b+)^(b-) relates previous body att to current

alpha_ib = omega_ib_b*dt; % attitude increment of body gyro rates

if norm(alpha_ib) > 1.E-7
% Rodrigues' Formula for precision with large attitude increments 
% Equation 5.73 in Groves
    C_b2oldb = eye(3) + ... 
             (sin(norm(alpha_ib))/norm(alpha_ib))*Skew(alpha_ib)+ ...
   ((1 - cos(norm(alpha_ib)))/norm(alpha_ib)^2)*Skew(alpha_ib)*Skew(alpha_ib);
                
else
    C_b2oldb = eye(3) + Skew(omega_ib_b*dt);
end 

% new attitude
C_b2e = C_b2e_in*C_b2oldb - Omega_ie*C_b2e_in.*dt;


%______________________Specific Force Transformation______________________%
% Precision update utilizes average rotation matrix. Eqn 5.85
Avg_C_b2e = C_b2e_in*C_b2oldb - 0.5*Omega_ie*C_b2e_in*dt; 

f_ib_e = Avg_C_b2e*f_ib_b;



%__________________________Velocity Update________________________________%
% Eqn. 5.36 
acc_ECEF = f_ib_e + g_ECEF - 2*Omega_ie*vel_in;
v_eb_e = vel_in + (f_ib_e + g_ECEF - 2*Omega_ie*vel_in).*dt;



%__________________________Position Update________________________________%
% Eqn 5.38  (average of velocity over interval)
p_eb_e = pos_in + ((vel_in + v_eb_e)*dt)/2;
end 
