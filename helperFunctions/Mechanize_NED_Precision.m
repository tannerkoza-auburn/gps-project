function [lat, lon, h, p_new, v_new, C_b2n] = Mechanize_NED_Precision(f_ib_b, omega_ib_b, lat_in, lon_in, h_in, pos_in, vel_in, C_b2n_in, dt)

%{
    Conversion of old NED mechanization to a more precise form to eliminate
    possible error sources in the transfer alignment process

    Written for GA Transfer Alignment project
    
    Ben Jones
    Original: 9/25/2020
    Updated to precision: 4/2/2021
    
    Source: 
    P. Groves, "Principles of GNSS Inertial and Multisensor Integrated
    Navigation Systems." 2nd Edition.


    Precision update performs attitude update last
%}


    % constants:
    R_o = 6.378137*10^6; % (m) - equatorial radius of Earth
    e = 0.0818; % eccentricity of Earth

    % Calculate Earth Radii as function of previous Lat:
    R_E = R_o / (sqrt(1 - e^2*(sind(lat_in))^2)); % meridian 
    R_N = ((1 - e^2)*R_o)/((1 - e^2*sind(lat_in)^2)^(3/2)); % transverse
    

    % rotation rate of Earth in nav frame: (Eq. 2.123)
    omega_ie_n = 7.292115*10^(-5)*[cosd(lat_in); 0; -sind(lat_in)];

    % rotation rate of nav frame wrt Earth (Eq. 5.44)
    omega_en_n = [vel_in(2)/(R_E + h_in); -vel_in(1)/(R_N + h_in); -tand(lat_in)*vel_in(2)/(R_E + h_in)];

    % create skew symmetric matrices (all still in rad / s)
    Omega_ie_n = Skew(omega_ie_n); % rotation rate of earth
    Omega_en_n = Skew(omega_en_n); % rotation rate of nav wrt earth

    % attitude updated by gyro measurements, current rot rate of body wrt e, and rot of earth
    %C_b2n = C_b2n_in*(eye(3) + Omega_ib_b*dt) - (Omega_ie_n + Omega_en_n)*C_b2n_in*dt;
       
    
    
    %% Step 1 transform specific force:
    
    
    alpha_ib = omega_ib_b*dt; % attitude increment of body gyro rates
    
    if norm(alpha_ib) > 1.E-7
        % Formula for precision with large attitude increments: Eq. 5.84 
        C_b2oldb = eye(3) + (1 - cos(norm(alpha_ib))) / norm(alpha_ib)^2 ...
        * Skew(alpha_ib) + (1 - sin(norm(alpha_ib)) /norm(alpha_ib)) /(norm(alpha_ib))^2 ...
        * Skew(alpha_ib)^2;
        
    else
        C_b2oldb = eye(3) + Skew(alpha_ib);
    end
        
    % average attitude over interval for specific force transformation:
     Avg_C_b2n = C_b2n_in*C_b2oldb - 0.5 * (Omega_en_n + Omega_ie_n) * C_b2n_in;
    
     f_ib_nav = Avg_C_b2n*f_ib_b;

    
    %% Step 2 Update Velocity: (precision velocity update is recursive so its avoided here)
    
    % calculate gravity vector from Groves ECEF cartesian model:
    g_nav = Gravity_NED(lat_in, h_in);
    
    % Eqn 5.54
    v_new = vel_in + dt*(f_ib_nav + g_nav - (Omega_en_n + 2*Omega_ie_n)*vel_in);
    
    
    
    %% Step 3: Update Position
    
    % More precise position update (Grove p.180):
    h = h_in - 0.5*dt*(vel_in(3) + v_new(3));
    lat = lat_in + rad2deg(0.5*dt*( vel_in(1)/(R_N + h_in)  + v_new(1)/(R_N + h)));
    
%     if h < 0 
%         error('height became negative')
%     end 
%    
    % update R_E and R_N for lon calculation
    R_E_new = R_o / (sqrt(1 - e^2*(sind(lat))^2));
    R_N_new = ((1 - e^2)*R_o)/((1 - e^2*sind(lat)^2)^(3/2));
    
    lon = lon_in + rad2deg(0.5*dt*( vel_in(2)/((R_E + h_in)*cosd(lat_in))  + v_new(2)/((R_E_new + h)*cosd(lat)) ));

% 
%     lon = lon_in + 0.5 * dt * (vel_in(2) / ((R_E + h_in) * cos(lat_in)) + v_new(2) / ((R_E_new + h_new) * cos(lat_new))); 
%     
    
    % NED position (m)
    p_new = pos_in + ((v_new + vel_in)/2)*dt;
    
    
    
    %% Step 4: Update attitude
    
    % recalculate rot rate of NED wrt to Earth
     new_omega_en = [v_new(2)/(R_E_new + h); 
                    -v_new(1)/(R_N_new + h); 
                    -tand(lat)*v_new(2)/(R_E_new + h)];
                
     new_Omega_en = Skew(omega_en_n);
     
     if norm(alpha_ib) > 1.E-7
        % Formula for precision with large attitude increments: Eq. 5.84 
        C_b2oldb = eye(3) + ... 
             (sin(norm(alpha_ib))/norm(alpha_ib))*Skew(alpha_ib)+ ...
         ((1 - cos(norm(alpha_ib)))/norm(alpha_ib)^2)*Skew(alpha_ib)*Skew(alpha_ib);
        
    else
        C_b2oldb = eye(3) + Skew(alpha_ib);
     end
     
    C_b2n = (eye(3) - (Omega_ie_n + 0.5*(Omega_en_n + new_Omega_en))*dt)*C_b2n_in*C_b2oldb;
     
    
    %% Additional Step: Keep the rotation matrices orthonormal:
    C_b2n = C_b2n*(C_b2n.'*C_b2n)^(-1/2);

end 