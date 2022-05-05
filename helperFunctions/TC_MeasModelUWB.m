function [dZ,H, R] = TC_MeasModelUWB(Xin, SVpsr, psrSigma, SVpos)
%{
    Slight changes to be used with UWBs instead
%}

% number of measurements (assuming psr and dopp for each SV/pseudolite)
nMeas = length(SVpsr);

% --- Constants
C = physconst('LightSpeed'); % Speed of light in m/s
omega_ie = 7.292115E-5;  % Earth rotation rate in rad/s
R_0 = 6378137; %WGS84 Equatorial radius in meters
e = 0.0818191908425; %WGS84 eccentricity
Omega_ie = Skew([0; 0; omega_ie]);

% extract states
estPos = Xin(1:3);
estVel = Xin(4:6);
estClkBias = Xin(7);
estClkDrift = Xin(8);


% Loop through available measurements at this epoch
R = zeros(nMeas, nMeas);
for i = 1:nMeas

    % approx range 
    rngHat = norm(SVpos(i,:)' - estPos);

    % frame rotation during measurement transit time using ( Groves 8.36 )
    C_e_I = [1, omega_ie * rngHat / C, 0;...
             -omega_ie * rngHat / C, 1, 0;...
             0, 0, 1];

    % Predict pseudo-range using (9.165)
    deltaECEF = (C_e_I * SVpos(i,:)' - estPos); % vector
    psrHat(i) = norm(deltaECEF); 
        
    % Predict line of sight
    unitVecs(i,1:3) = deltaECEF' / norm(deltaECEF); % exclude clkbias here
    
    % --- construct innovation vector
    dZ(i,1) = SVpsr(i) - psrHat(i); % psr
    
    % --- populate R matrix
    R(i,i) = psrSigma(i)^2;    
end 


% recall: organizing the measurements as: [psr1 psr2 psr3... dopp1 dopp2 dopp3]

% --- construct state-to-measurement mapping matrix, H, using (9.163)
H = zeros(nMeas,8);
H(1:nMeas,1:3) = -unitVecs;
H(1:nMeas,7) = zeros(nMeas,1);

end



function [skew] = Skew(omega)
%{
    Creates skew-symmetric form of input vector
%}

omega_x = omega(1);
omega_y = omega(2);
omega_z = omega(3);

skew = [0 -omega_z omega_y;
        omega_z 0 -omega_x;
        -omega_y omega_x 0];

end

