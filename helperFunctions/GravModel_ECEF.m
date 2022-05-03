function [grav_ECEF, gamma] = GravModel_ECEF(pos_ECEF)

 %{
    ECEF-Frame Gravity model for GA Collaborative guidance project.
    Utilizes ECEF position to return an ECEF-resolved gravity vector.

    Ben Jones and Will Kennedy
    3/31/21

    Source: 
    P. Groves, "Principles of GNSS Inertial and Multisensor Integrated
    Navigation Systems." 2nd Edition.
%}

% Earth Parameters:
mu = 3.986004418 * 10^(14); % (m^3/s^2) WGS 84 gravitational constant
J2 = 1.082627*10^(-3); % Earth's second gravitational constant WGS 84
R_o = 6378137; % (m) WGS 84 equatorial radius of Earth
omega_ie = 7.292115*10^(-5); % (rad/s) rotation rate of Earth


% gamma Groves Eqn. 2.142
    % last term is nasty so I calculated it separately
last_term = [ (1 - 5*(pos_ECEF(3)/norm(pos_ECEF))^2)*pos_ECEF(1);
              (1 - 5*(pos_ECEF(3)/norm(pos_ECEF))^2)*pos_ECEF(2);
              (3 - 5*(pos_ECEF(3)/norm(pos_ECEF))^2)*pos_ECEF(3) ];
  
gamma = (-mu/norm(pos_ECEF)^3)*(pos_ECEF + ...
            1.5*J2*(R_o^2/(norm(pos_ECEF)^2))*last_term);


% centrifugal acceleration term
cent_terms = omega_ie^2*[1 0 0; 0 1 0; 0 0 0]*pos_ECEF;

% gravitational acceleration:
grav_ECEF = gamma + cent_terms;


end 