function [dZ,H, R] = TC_MeasModel(Xin, SVpsr, psrSigma, SVdopp, doppSigma, SVpos, SVvel)
%{
    Take in available measurements and current state estimate to construct
    the measurement update components for a tightly-coupled integration.

    Can be used with traditional satellite measurements or pseudolites (UWB
    base stations). For pseudolites or SVs without a dopp measurement, just 
    pass in zeros for the Dopp measurements along with a near infinite R 
    value so they're simply ignored.

    !!! IMPORTANT NOTES FOR USAGE:
    - input vectors MUST be in the same order... eg: SV4 psr must have
    the same index as SV4 position 
    - pseudoranges passed in should be already corrected!!
    - all measurements must be synced in time. perform separate measurement
    updates for non-synced measurements
    - organizing the measurements as: [psr1 psr2 psr3... dopp1 dopp2 dopp3]

    Inputs:
    - Xin: current state estimate (ECEF)
    - SVpsr: vector (n x 1) of pseudoranges
    - psrSigma: uncertainty in pseudoranges
    - SVdopp: vector (n x 1) of doppler measurements  ( !!! in HZ !!! )
    - doppSigma: uncertainty in doppler measurements
    - SVpos: matrix (n x 3) of SV positions (ECEF) 
        ^ will be repeated for SVs with psr and dopp
    - SVvel: matrix (n x 3) of SV velocities (ECEF)

    Outputs:
    - dZ: innovation between state-measurement estimate and measurements
    - H: linearized measurement matrix
    - R: matrix of measurement uncertainty

%}

% number of measurements (assuming psr and dopp for each SV/pseudolite)
nMeas = 2*length(SVpsr);

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

% convert dopps from Hz to m/s
L1_freq = 1575.42e6;
SVdopp = SVdopp * -( C/L1_freq );

% Loop through available measurements at this epoch
R = zeros(nMeas, nMeas);
for i = 1:nMeas/2

    % approx range 
    rngHat = norm(SVpos(i,:)' - estPos);

    % frame rotation during measurement transit time using ( Groves 8.36 )
    C_e_I = [1, omega_ie * rngHat / C, 0;...
             -omega_ie * rngHat / C, 1, 0;...
             0, 0, 1];

    % Predict pseudo-range using (9.165)
    deltaECEF = (C_e_I * SVpos(i,:)' - estPos); % vector
    psrHat(i) = norm(deltaECEF) + estClkBias; 
        
    % Predict line of sight
    unitVecs(i,1:3) = deltaECEF' / norm(deltaECEF); % exclude clkbias here
        
    % Predict pseudo-range rate using (9.165)
    doppHat(i) = unitVecs(i,1:3) * (C_e_I * (SVvel(i,:)' +...
        Omega_ie * SVpos(i,:)') - (estVel + Omega_ie * estPos)) + estClkDrift;        
    
    % --- construct innovation vector
    dZ(i,1) = SVpsr(i) - psrHat(i); % psr
    dZ(i + nMeas/2,1) = SVdopp(i) - doppHat(i); % dopp
    
    % --- populate R matrix
    R(i,i) = psrSigma(i);
    R(i + nMeas/2, i + nMeas/2) = doppSigma(i);
    
end 


% recall: organizing the measurements as: [psr1 psr2 psr3... dopp1 dopp2 dopp3]

% --- construct state-to-measurement mapping matrix, H, using (9.163)
H = zeros(nMeas,8);
H(1:nMeas/2,1:3) = -unitVecs;
H(1:nMeas/2,7) = ones(nMeas/2,1);
H(nMeas/2 + 1:end,4:6) = -unitVecs;
H(nMeas/2 + 1:end,8) = ones(nMeas/2,1);


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

