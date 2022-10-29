function ephemerisOut = readMultiGNSSRinex(filename,TOW)

% ephemerisOut = readMultiGNSSRinex(filename,TOW)
%
% This function takes in a multi-gnss observation file and reads the gnss
% observations into structures for the appropriate constellations
%
% Data files can be found at: (for 2020 - change directory for other years)
% ftp://cddis.gsfc.nasa.gov/pub/gnss/data/daily/2020/brdc/
% *Note - Anonymous FTP will be disabled Dec. 31, 2020 - this link won't
% work anymore but files will be in the same directory in the logged in
% version
%
% Currently supported constellations are: GPS, Galileo, SBAS
% Coming soon: GLONASS, others as needed
%
% RINEX 3.03 format is used as the primary reference for the formatting of
% this parser
%
% Constellation abbreviations:
% G - GPS
% E - Galileo
% R - GLONASS
% S - SBAS
% I - IRNSS
% C - Beidou
% J - QZSS
%
% Inputs:
%   filename - relative or absolute filepath of RINEX file
%   % TOW    - GPS time of week at the start of the time of interest. Used
%   to determine which rinex entries to save to ensure that they are valid
%
% Outputs: 
%   ephemerisOut - struct containing parsed ephemeris data from the
%   provided file


%% Setup
% Setting up number of lines for each navigation message type
constellationType = {'G','E','R','S','I','C','J'};
numLines = [8, 8, 4, 4, 8, 8, 8];

linesPerObs = containers.Map(constellationType,numLines);

% Opening file
fid = fopen(filename,'r');

% Initializing ephem struct
ephem.gps = [];
ephem.galileo = [];
ephem.waas = [];
ephem.glonass = [];
constants.firstGlonass = 0;
constants.firstIRNSS = 0;
constants.firstQZSS = 0;
constants.firstBeidou = 0;

constants.TOW = TOW;
constants.glonassTow= 22;

%% Getting number of lines
eoh = false;    % Set end of header to false until end is detected
headerCount = 0;

while eoh == false
    headerCount = headerCount + 1;
    data = fgetl(fid);
    header(headerCount) = {data};
    
    % TODO - process GNSS time offsets and save eventually
    
    if contains(data,"END OF HEADER")
        eoh = true;
    end
end

% Get first observation to prime parser
data = fgetl(fid);
while ~feof(fid)
    linecount = 1;
    constellation = find(strcmp(data(1),constellationType));
    linesMax = linesPerObs(data(1));
    constants.saveEphem = 0;
    
    switch constellation
        case 1 % GPS
            while linecount <= linesMax
                [ephem,constants] = processGPSLine(ephem,data,...
                    linecount,constants);
                data = fgetl(fid);
                linecount = linecount + 1;
                if constants.saveEphem == 1
                    ephemerisOut.gps(constants.prn,:) = ephem.gps(:);
                end
            end
            
        case 2 % Galileo
            while linecount <= linesMax
                [ephem,constants] = processGalileoLine(ephem,data,...
                    linecount,constants);
                data = fgetl(fid);
                linecount = linecount + 1;
                if constants.saveEphem == 1
                    ephemerisOut.galileo(constants.prn,:) = ...
                        ephem.galileo(:);
                end
            end
            
        case 3 % GLONASS
            while linecount <= linesMax
                [ephem,constants] = processGlonassLine(ephem,data,...
                    linecount,constants);                
                data = fgetl(fid);
                linecount = linecount + 1;                
                if constants.saveEphem == 1
                    ephemerisOut.glonass(constants.prn,:) = ...
                        ephem.glonass(:);
                end
            end
            
            
        case 4 % SBAS
            while linecount <= linesMax
                [ephem,constants] = processSBASLine(ephem,data,...
                    linecount,constants);
                if constants.saveEphem == 1
                    ephemerisOut.sbas(constants.prn,:) = ...
                        ephem.sbas(:);
                end
                data = fgetl(fid);
                linecount = linecount + 1;
            end
            
            
        case 5 % IRNSS
            while linecount <= linesMax
                if constants.firstIRNSS == 0
%                     warning('IRNSS Parser not completed yet!')
                    constants.firstIRNSS = 1;
                end
                data = fgetl(fid);
                linecount = linecount + 1;
            end
            
            
        case 6 % BeiDou
            while linecount <= linesMax
                if constants.firstBeidou == 0
%                     warning('BeiDou Parser not completed yet!')
                    constants.firstBeidou = 1;
                end
                data = fgetl(fid);
                linecount = linecount + 1;
            end
            
            
        case 7 % QZSS
            while linecount <= linesMax
                if constants.firstQZSS == 0
%                     warning('QZSS Parser not completed yet!')
                    constants.firstQZSS = 1; 
                end
                data = fgetl(fid);
                linecount = linecount + 1;
            end
            
            
        otherwise
            warnString = ['Unrecognized constellation in RINEX file!'...
                ' Constellation ID is: ', data(1), '\n'];
            fprintf(2, warnString)
    end
    
end

fclose(fid);
% Function end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Supporting functions
% Process GPS lines
function [ephem,constants] = processGPSLine(ephem,line,linecount,constants)
switch linecount
    case 1
        prn = str2double(line(2:3));
        ephem.gps.prn = prn;
        constants.prn = prn;            % Used for bookkeeping
        ephem.gps.a_f0 = str2double(line(24:42));
        ephem.gps.a_f1 = str2double(line(43:61));
        ephem.gps.a_f2 = str2double(line(62:end));
        
    case 2
        ephem.gps.IODE_sf2 = str2double(line(1:23));
        ephem.gps.C_rs =  str2double(line(24:42));
        ephem.gps.deltan =  str2double(line(43:61));
        ephem.gps.M_0 =  str2double(line(62:end));
        
    case 3
        ephem.gps.C_uc = str2double(line(1:23));
        ephem.gps.e = str2double(line(24:42));
        ephem.gps.C_us = str2double(line(43:61));
        ephem.gps.A = str2double(line(62:end))^2;
        
    case 4
        ephem.gps.t_oe = str2double(line(1:23));
        ephem.gps.C_ic = str2double(line(24:42));
        ephem.gps.omega_0 = str2double(line(43:61));
        ephem.gps.C_is = str2double(line(62:end));
        
    case 5
        ephem.gps.i_0 = str2double(line(1:23));
        ephem.gps.C_rc = str2double(line(24:42));
        ephem.gps.omega = str2double(line(43:61));
        ephem.gps.omegaDot = str2double(line(62:end));
        
    case 6
        ephem.gps.iDot = str2double(line(1:23));
        ephem.gps.L2Code = str2double(line(24:42));
        ephem.gps.gpsWeek = str2double(line(44:61));
        ephem.gps.L2P = str2double(line(62:end));
        
    case 7
        ephem.gps.Accuracy = str2double(line(1:23));
        ephem.gps.health = str2double(line(1:23));
        ephem.gps.T_GD = str2double(line(43:61));
        ephem.gps.IODC = str2double(line(62:end));
        
    case 8
        ephem.gps.t_oc = str2double(line(1:23));
        
        if ephem.gps.t_oe <= constants.TOW
            constants.saveEphem = 1;
        end
end
end

% Process Galileo lines
function [ephem,constants] = processGalileoLine(ephem,line,...
    linecount,constants)
switch linecount
    case 1
        prn = str2double(line(2:3));
        ephem.galileo.prn = prn;
        constants.prn = prn;            % Used for bookkeeping
        ephem.galileo.a_f0 = str2double(line(24:42));
        ephem.galileo.a_f1 = str2double(line(43:61));
        ephem.galileo.a_f2 = str2double(line(62:end));
        
    case 2
        ephem.galileo.IODE_sf2 = str2double(line(1:23));
        ephem.galileo.C_rs =  str2double(line(24:42));
        ephem.galileo.deltan =  str2double(line(43:61));
        ephem.galileo.M_0 =  str2double(line(62:end));
        
    case 3
        ephem.galileo.C_uc = str2double(line(1:23));
        ephem.galileo.e = str2double(line(24:42));
        ephem.galileo.C_us = str2double(line(43:61));
        ephem.galileo.A = str2double(line(62:end))^2;
        
    case 4
        ephem.galileo.t_oe = str2double(line(1:23));
        ephem.galileo.C_ic = str2double(line(24:42));
        ephem.galileo.omega_0 = str2double(line(43:61));
        ephem.galileo.C_is = str2double(line(62:end));
        
    case 5
        ephem.galileo.i_0 = str2double(line(1:23));
        ephem.galileo.C_rc = str2double(line(24:42));
        ephem.galileo.omega = str2double(line(43:61));
        ephem.galileo.omegaDot = str2double(line(62:end));
        
    case 6
        ephem.galileo.iDot = str2double(line(1:23));
        ephem.galileo.L2Code = str2double(line(24:42));
        ephem.galileo.gpsWeek = str2double(line(43:61));
        ephem.galileo.L2P = str2double(line(62:end));
        
    case 7
        ephem.galileo.Accuracy = str2double(line(1:23));
        ephem.galileo.health = str2double(line(1:23));
        ephem.galileo.T_GD = str2double(line(43:61));
        ephem.galileo.IODC = str2double(line(62:end));
        
    case 8
        ephem.galileo.t_oc = str2double(line(1:23));
        
        if ephem.galileo.t_oe <= constants.TOW
            constants.saveEphem = 1;
        end
end
end

% Process GLONASS Lines
function [ephem,constants] = processGlonassLine(ephem,line,...
    linecount,constants)
switch linecount
    case 1
        prn = str2double(line(2:3));
        ephem.glonass.prn = prn;
        constants.prn = prn;            % Used for bookkeeping
                
        % Lets store everything for the heck of it.
        ephem.glonass.time.year = str2double(line(5:8)); % year
        ephem.glonass.time.month = str2double(line(10:11));
        ephem.glonass.time.day = str2double(line(13:14));
        ephem.glonass.time.minute = str2double(line(16:17));
        ephem.glonass.time.second = str2double(line(19:20));
        ephem.glonass.svClkBias = str2double(line(24:42)); 
        ephem.glonass.svRelativeFrequencyBias = str2double(line(43:62));
        ephem.glonass.messageFrameTime = str2double(line(62:end));
        
    case 2
        ephem.glonass.svPosX = str2double(line(5:23));
        ephem.glonass.svVelX =  str2double(line(24:42));
        ephem.glonass.svAccelX =  str2double(line(43:61));
        ephem.glonass.health =  str2double(line(62:end));
        
    case 3
        ephem.glonass.svPosY = str2double(line(5:23));
        ephem.glonass.svVelY = str2double(line(24:42));
        ephem.glonass.svAccelY = str2double(line(43:61));
        ephem.glonass.FrequencyNumber = str2double(line(62:end));
        
    case 4
        ephem.glonass.svPosZ = str2double(line(5:23));
        ephem.glonass.svVelZ = str2double(line(24:42));
        ephem.glonass.svAccelZ = str2double(line(43:61));
        ephem.glonass.ageOfOperation = str2double(line(62:end));       
        
        if ephem.glonass.time.minute <= constants.glonassTow
            constants.saveEphem = 1;
        end
end
end

% Process SBAS lines
function [ephem,constants] = processSBASLine(ephem,line,...
    linecount,constants)
switch linecount
    case 1
        prn = str2double(line(2:3))-20; % True PRN = prn+120 (done for simplicity)
        ephem.sbas.prn = prn + 120;
        constants.prn = prn;            % Used for bookkeeping
        ephem.sbas.a0 = str2double(line(25:42));
        ephem.sbas.a1 = str2double(line(43:61));
        ephem.sbas.t_ot = str2double(line(63:end));
        
    case 2
        ephem.sbas.xPos = str2double(line(1:23))*1e3;
        ephem.sbas.xVel =  str2double(line(24:42))*1e3;
        ephem.sbas.xAcc =  str2double(line(44:61))*1e3;
        ephem.sbas.health =  str2double(line(63:end));
        
    case 3
        ephem.sbas.yPos = str2double(line(1:23))*1e3;
        ephem.sbas.yVel = str2double(line(24:42))*1e3;
        ephem.sbas.yAcc = str2double(line(44:61))*1e3;
        ephem.sbas.accuracy = str2double(line(63:end));
        
    case 4
        ephem.sbas.zPos = str2double(line(1:23))*1e3;
        ephem.sbas.zVel = str2double(line(24:42))*1e3;
        ephem.sbas.zAcc = str2double(line(44:61))*1e3;
        ephem.sbas.IODN = str2double(line(63:end));
        
        if ephem.sbas.t_ot <= constants.TOW
            constants.saveEphem = 1;
        end
end
end