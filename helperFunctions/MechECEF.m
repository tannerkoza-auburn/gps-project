classdef MechECEF < matlab.mixin.Copyable
    
    % This class institutes a precision ECEF-Frame IMU Mechanization based on
    % Groves, Paul "Principles of GNSS, Inertial, and Multisensor
    % Integrated Navigation Systems, 2nd Edition"
    
    % Ben Jones
    % 5/3/21
    
    properties
        
        % PVA
        pos % (m) - position in ECEF frame 
        vel % (m/s) - velocity in ECEF frame
        C_b2e % attitude - rotm from body to ECEF frame
        Eul % (deg) - attitude in Euler angles
        C_b2n % attitude for extracting Euler angles
        
        gamma % (m/s^2) gravitational acceleration
        r_eS_e % (m) geocentric radius at surface
        
        dt % measurement rate of IMU and update rate of mechanization
        
       
    end
    
    methods
        function self = MechECEF(frame, pos, vel, rotm, dt, n_steps)
            
            % Initialization of Mechanization class
            
            %{
            
            Two methods of initialization: ECEF or NED inputs
                
                NED inputs: 
                    pos = lla = [lat; lon; height] - (deg | deg | m) 
                    vel - (m/s) NED resolved velocity 
                    rotm - body-to-NED rotation matrix
            
                ECEF inputs:
                    pos - (m) ECEF position
                    vel - (m/s) ECEF resolved velocity
                    rotm - body-to-ECEF rotation matrix
            
            
            Within intialization... conversions to ECEF properties which
            will be propogated in mechanization.
            
            %}
            
            
            % pre-allocate
            self.pos = NaN(3,n_steps);
            self.vel = NaN(3,n_steps);
            self.Eul = NaN(3,n_steps);
            self.C_b2e = NaN(3,3,n_steps);
            
            self.dt = dt; % (s) - update rate 
            
            switch frame
                
                case 'NED'
                 
                % unpack NED parameters 
                    C_b2n = rotm; % input rotm is body to NED 
                    lat = pos(1);
                    lon = pos(2);
                    h = pos(3);
                    v_NED = vel;
                    
                    
                    
                    R_o = 6378137; % (m) equatorial radius of the Earth in meters WGS84
                    e = 0.0818191908425; % eccentricity of the Earth WGS84
                    
                    % transverse radius of curvature (Groves eqn 2.105)
                    R_E = R_o / (sqrt(1 - e^2*sind(lat)^2));
                    
                    % ECEF position
                    self.pos(:,1) = [(R_E + h) * cosd(lat) * cosd(lon);...
                        (R_E + h) * cosd(lat) * sind(lon);...
                        ((1 - e^2) * R_E + h) * sind(lat)];
                    
                    
                    % ECEF attitude/NED Euler angles:
                    C_n2e = NED_to_ECEF(lat,lon);
                    self.C_b2e(:,:,1) = C_n2e*C_b2n;
                    self.C_b2n(:,:,1) = C_b2n; % local frame rotm for eul
                    eul = rad2deg(rotm2eul(self.C_b2n(:,:,1)));
                    self.Eul(:,1) = [eul(3); eul(2); eul(1)];
                    
                    
                    % ECEF velocity:
                    self.vel(:,1) = C_n2e*v_NED;
                    
                    
                    
                case 'ECEF'
                    
                % no conversions necessary, just assign properties:
                    self.pos(:,1) = pos;
                    self.vel(:,1) = vel;                
                    self.C_b2e(:,:,1) = rotm; % intput rotm is body to ECEF
                    
                    % construct C_b2n for Euler angle extraction:
                    lla = ecef2lla([self.pos(1,1) self.pos(2,1) self.pos(3,1)]);
                    lat = lla(1); lon = lla(2); % extract lat and lon
                    C_n2e = NED_to_ECEF(lat,lon); 
                    self.C_b2n(:,:,1) = C_n2e.'*rotm; % local frame rotm for eul
                    eul = rad2deg(rotm2eul(self.C_b2n(:,:,1)));
                    self.Eul(:,1) = [eul(3); eul(2); eul(1)]; % Euler angles
                    
            end
            
        end
        
        function self = Mechanize(self, f_ib_b, omega_ib_b, n)
            %{
                Mechanization of IMU accelerometer and gyro measurements. Selects
                higher fidelity update if magnitude of angular rate exceeds preset
                maximum. Intended for GA collaborative navigation simulation.
            
                Inputs:
                   - f_ib_b: accelerometer measurements (m/s^2)
                   - omega_ib_b: gyroscope measurements (rad/s)
                   - n: iteration variable... outputs stored to n + 1 index
            %}
            
            
            %_____ Parameters and initialization _____%
            Omega_ie = Skew([0; 0; 7.292115*10^(-5)]); % (rad/s) Skew of Earth Rotation
                                 
            % gravity model:
            [g_ECEF, self.gamma(:,n)] = GravModel_ECEF(self.pos(:,n));
            
            
            
            % construct C_b2n for Euler angle extraction:
            lla = ecef2lla([self.pos(1,n) self.pos(2,n) self.pos(3,n)]);
            lat = lla(1); lon = lla(2); % extract lat and lon
            C_n2e = NED_to_ECEF(lat,lon); % temp variable
            self.C_b2n(:,:,n) = C_n2e.'*self.C_b2e(:,:,n); % local frame rotm for eul
            eul = rad2deg(rotm2eul(self.C_b2n(:,:,n)));
            self.Eul(:,n) = [eul(3); eul(2); eul(1)]; % Euler angles
            
            
            
            %_____ Atttitude Update _____%
            
            % Precision update:  Eq. 5.75
            % C_b2e(+) = C_b2e(-) * C_b(+)^b(-)  -  Omega_ie*C_b(+)^b(-)*dt
            % Where C_(b+)^(b-) relates previous body att to current
            
            alpha_ib = omega_ib_b*self.dt; % attitude increment of body gyro rates
            
            if norm(alpha_ib) > 1.E-7
                % Rodrigues' Formula for precision with large attitude increments
                % Equation 5.73 in Groves
                C_b2oldb = eye(3) + ...
                    (sin(norm(alpha_ib))/norm(alpha_ib))*Skew(alpha_ib)+ ...
                    ((1 - cos(norm(alpha_ib)))/norm(alpha_ib)^2)*Skew(alpha_ib)*Skew(alpha_ib);
                
            else
                C_b2oldb = eye(3) + Skew(omega_ib_b*self.dt);
            end
            
            % new attitude
            C_b2e_new = self.C_b2e(:,:,n)*C_b2oldb - Omega_ie*self.C_b2e(:,:,n).*self.dt;
            
            
            %_____ Specific Force Transformation _____%
            % Precision update utilizes average rotation matrix. Eqn 5.85
            Avg_C_b2e = self.C_b2e(:,:,n)*C_b2oldb - 0.5*Omega_ie*self.C_b2e(:,:,n)*self.dt;
            
            f_ib_e = Avg_C_b2e*f_ib_b;
            
            
            
            %_____ Velocity Update _____%
            % Eqn. 5.36
            % acc_ECEF = f_ib_e + g_ECEF - 2*Omega_ie*self.vel;
            v_new = self.vel(:,n) + (f_ib_e + g_ECEF - 2*Omega_ie*self.vel(:,n)).*self.dt;
            
            
            %_____ Position Update _____%
            % Eqn 5.38  (average of velocity over interval)
            self.pos(:,n+1) = self.pos(:,n) + ((self.vel(:,n) + v_new)*self.dt)/2;
            
            
            %_____ Replace old values with new _____%
            self.C_b2e(:,:,n+1) = C_b2e_new;
            self.vel(:,n+1) = v_new;
            
            
            % construct C_b2n for Euler angle extraction:
            lla = ecef2lla([self.pos(1,n+1) self.pos(2,n+1) self.pos(3,n+1)]);
            lat = lla(1); lon = lla(2); % extract lat and lon
            C_n2e = NED_to_ECEF(lat,lon); % temp variable
            self.C_b2n(:,:,n+1) = C_n2e.'*self.C_b2e(:,:,n+1); % local frame rotm for eul
            eul = rad2deg(rotm2eul(self.C_b2n(:,:,n+1)));
            self.Eul(:,n+1) = [eul(3); eul(2); eul(1)]; % Euler angles
            
        end
    end
end

