function [lat, lon, h, p_new, v_new, C_b2n] = Mechanize_NED(f_ib_b, omega_ib_b, lat_in, lon_in, h_in, pos_in, vel_in, C_b2n_in, dt)

% Mechanization of IMU acc and gyro data into the Nav NED frame:
% based on lecture from fundamentals of navigation and guidance course (Fall 2020)

% Ben Jones
% 9/25/2020

    % constants:
    R_o = 6.378137*10^6; % (m) - equatorial radius of Earth
    e = 0.0818; % eccentricity of Earth

    % Calculate Earth Radii as function of Lat:
    R_E = R_o / (sqrt(1 - e^2*(sind(lat_in))^2));
    R_N = ((1 - e^2)*R_o)/((1 - e^2*sind(lat_in)^2)^(3/2));


    %% Step 1 Update attitude:

    % rotation rate of Earth in nav frame:
    omega_ie_n = 7.292115*10^(-5)*[cosd(lat_in); 0; -sind(lat_in)];

    % rotation rate of nav frame wrt Earth
    omega_en_n = [vel_in(2)/(R_E + h_in); -vel_in(1)/(R_N + h_in); -tand(lat_in)*vel_in(2)/(R_E + h_in)];

    % create skew symmetric matrices (all still in rad / s)
    Omega_ib_b = Skew(omega_ib_b); % gyro rates of body
    Omega_ie_n = Skew(omega_ie_n); % rotation rate of earth
    Omega_en_n = Skew(omega_en_n); % rotation rate of nav wrt earth

    % attitude updated by gyro measurements, current rot rate of body wrt e, and rot of earth
    C_b2n = C_b2n_in*(eye(3) + Omega_ib_b*dt) - (Omega_ie_n + Omega_en_n)*C_b2n_in*dt;

    % Dan's method forces orthonormality
    % C_b2n = ((2*eye(3) + Omega_ib_b*dt)/(2*eye(3) - Omega_ib_b*dt))*C_b2n_in;
        
    

    %% Step 2 transform force outputs:
    f_ib_nav = 0.5*(C_b2n_in + C_b2n)*f_ib_b;
    
    
    
    %% Step 3: Update Velocity
   
    % calculate gravity vector from Dr. Martin's ECEF cartesian model: 
    g_nav = Gravity_NED(lat_in, h_in);
    
    % integrate for new velocity
    v_new = vel_in + dt*(f_ib_nav + g_nav - (Omega_en_n + 2*Omega_ie_n)*vel_in);
    
    
    %% Step 4: Update position
    
    % modification to position update (Grove p.180):
    h = h_in - 0.5*dt*(vel_in(3) + v_new(3));
    lat = lat_in + rad2deg(0.5*dt*( vel_in(1)/(R_N + h_in)  + v_new(1)/(R_N + h)));

    
    % update R_E :
    R_E_new = R_o / (sqrt(1 - e^2*(sind(lat))^2));
    
    lon = lon_in + rad2deg(0.5*dt*( vel_in(2)/(R_E*cosd(lat_in))  + v_new(2)/(R_E_new*cosd(lat)) ));
    
    % NED position 
    p_new = pos_in + ((v_new + vel_in)/2)*dt;

end 