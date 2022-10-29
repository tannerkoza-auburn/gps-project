function [g_nav] = Gravity_nav(lat, h)

% Utilize NED gravity model to calculate the gravity vector given the
% current position.
% Based on Paul Groves' textbook


% Constants 
R_0 = 6.378137*10^6; % WGS84 Equatorial radius in meters
R_P = 6.3568*10^6; %  Polar radius in meters
e = 0.0818191908425; % eccentricity
f = 1 / 298.257223563; %  flattening
mu = 3.9860*10^14; % Earth gravitational constant (m^3 s^-2)
omega_ie = 7.292115*10^(-5);  % rotation rate (rad/s)

% Calculate surface gravity (2.134)
g_0 = 9.7803253359 * (1 + 0.001931853 * sind(lat)^2) / sqrt(1 - e^2 * sind(lat)^2);

% north gravity (2.140)
g_nav(1,1) = -8.08E-9 * h * sind(2 * lat);

% East gravity is zero
g_nav(2,1) = 0;

% Calculate down gravity using (2.139)
g_nav(3,1) = g_0 * (1 - (2 / R_0) * (1 + f * (1 - 2 * sind(lat)^2) +...
    (omega_ie^2 * R_0^2 * R_P / mu)) * h + (3 * h^2 / R_0^2));


end 