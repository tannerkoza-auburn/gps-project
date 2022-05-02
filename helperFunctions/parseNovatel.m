function out = parseNovatel(bagName, ref_lla, topicPrefix)
           
    if isempty(ref_lla)
        ref_lla = [32.605363688183651, -85.486936034004174, 201.7449376863390];
    end

    gpsPi = 3.1415926535898;  % Pi used in the GPS coordinate system

    bag = rosbag(bagName);
    topic = select(bag, 'Topic',topicPrefix + "/gpsEphemerisTagged");
    structure = readMessages(topic,'DataFormat', 'struct');
    out.gpsEphemerisTagged.time = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1e-9,structure)';
    out.gpsEphemerisTagged.gpsWeek = cellfun(@(m) double(m.Tags.GpsTime.GpsWeek),structure)';
    out.gpsEphemerisTagged.gpsSeconds = cellfun(@(m) double(m.Tags.GpsTime.GpsSeconds),structure)';
    ephemCount = 1;
    k = 1;

for n = 1:length(structure)
    %     if n == 1
    %         gpsSec = msgStructs{n}.Tags.GpsTime.GpsSeconds;
    %         currentTime = gpsSec;
    %     else
    %         gpsSec = msgStructs{n}.Tags.GpsTime.GpsSeconds;
    %     end
    %
    %     if currentTime ~= gpsSec
    %         ephemCount = ephemCount + 1;
    %         currentTime = gpsSec;
    %         k = 1;
    %     end
    
    gpsSec = structure{n}.Tags.GpsTime.GpsSeconds;
    
    ephemOn = false;
    prn = double(structure{n}.Ephemeris.Prn);
    if n == 1 && ephemOn
        rosSec = structure{n}.Header.Stamp.Sec;
        
        out.Ephemeris(k,:).prn = double(prn);
        out.Ephemeris(k,:).rosSec = double(rosSec);
        out.Ephemeris(k,:).gpsSec = double(gpsSec);
        
        % Decoding ephemeris subframes
        %%%%%%%%%%%%%%%%%%% Subframe 1 %%%%%%%%%%%%%%%%%%%%%%%
        % The reported message strucuture says the output is in hex... Yet
        % the values are represented in decimal??? Maybe ask JD?
        subframe1 = structure{n}.Ephemeris.Subframe1;
        sf1Bin = dec2bin(subframe1);
        binArray = [sf1Bin(1,:) sf1Bin(2,:) sf1Bin(3,:) sf1Bin(4,:) sf1Bin(5,:) sf1Bin(6,:) ...
            sf1Bin(7,:) sf1Bin(8,:) sf1Bin(9,:) sf1Bin(10,:) sf1Bin(11,:) sf1Bin(12,:) ...
            sf1Bin(13,:) sf1Bin(14,:) sf1Bin(15,:) sf1Bin(16,:) sf1Bin(17,:) sf1Bin(18,:) ...
            sf1Bin(19,:) sf1Bin(20,:) sf1Bin(21,:) sf1Bin(22,:) sf1Bin(23,:) sf1Bin(24,:) ...
            sf1Bin(25,:) sf1Bin(26,:) sf1Bin(27,:) sf1Bin(28,:) sf1Bin(29,:) sf1Bin(30,:)];
        
        
        % Parity Bits are removeed from the subframe information
        
        % TLM Word
        out.Ephem_V2(ephemCount,:).TLM = binArray(1,1:22) 
        % Prebamle should be the first 8 bits and should aways be the same
        out.Ephem_V2(ephemCount,:).preamble = out.Ephem_V2(ephemCount,:).TLM(1:8)  % Should be =====> '10001011'
        % Bit 23 of each TLM word is the Integrity Status Flag (ISF). A "0" in bit position 23 indicates that the
        % conveying signal is provided with the legacy level of integrity assurance. That is, the probability
        % that the instantaneous URE of the conveying signal exceeds 4.42 times the upper bound value of
        % the current broadcast URA index, for more than 5.2 seconds, without an accompanying alert, is
        % less than 1E-5 per hour. A "1" in bit-position 23 indicates that the conveying signal is provided with
        % an enhanced level of integrity assurance. That is, the probability that the instantaneous URE of
        % the conveying signal exceeds 5.73 times the upper bound value of the current broadcast URA
        % index, for more than 5.2 seconds, without an accompanying alert, is less than 1E-8 per hour. 
        out.Ephem_V2(ephemCount,:).integrityStatusFlag = binArray(1,23);
%         out.Ephem_V2(ephemCount,:).reserved =  binArray(1,24); % Rserved for parity computation (not used for anything)
        
        % HOW Word
        out.Ephem_V2(ephemCount,:).HOW = binArray(1,25:46) ;
        out.Ephem_V2(ephemCount,:).TOW = bin2dec(out.Ephem_V2(ephemCount,:).HOW(1:17))*4;
        out.Ephem_V2(ephemCount,:).alertFlag = out.Ephem_V2(ephemCount,:).HOW (18);
        %Bit 18 is an "alert" flag. When this flag is raised (bit 18 = "1"), it shall indicate to the standard
        % positioning service (SPS) user (unauthorized user) that the signal URA may be worse than
        % indicated in subframe 1 and that he shall use that SV at his own risk.
        out.Ephem_V2(ephemCount,:).antiSpoofFlag = out.Ephem_V2(ephemCount,:).HOW(19);
        % Bit 19 is an anti-spoof (A-S) flag. A "1" in bit-position 19 indicates that the A-S mode is ON in that
        % SV.
        
        idCode = out.Ephem_V2(ephemCount,:).HOW(20:22);                
        %out.Ephem_V2(ephemCount,:).subFrameID = idCode2SubFrameId(idCode); % This looks good!!! 
%         out.Ephem_V2(ephemCount,:).bitSolver4Parity = binArray(1,47:48);
        % 2 NONINFORMATION BEARING BITS USED FOR PARITY COMPUTATION
        
        % Hmm.... not sure why we have to 1024 twice??? Looks good though
        out.Ephem_V2(ephemCount,:).weekNumber = bin2dec(binArray(1,49:58)) + 2*1024
        codes4L2Id = binArray(1,59:60);
        %out.Ephem_V2(ephemCount,:).code4L2 = l2CodeIdentifier(codes4L2Id);
        
        % The nominal URA value (X) is suitable for use as a conservative prediction of the RMS 
        % signal-inspace (SIS) range errors for accuracy-related purposes in the pseudorange domain
        out.Ephem_V2(ephemCount,:).uraIndex = bin2dec(binArray(1,61:64));
        
        % The MSB indicates a summary of the health of the lNAV data
        % 0 = all data are OK
        % 1 = some or all of the LNAV data are bad
        % We will be ingoring the LSB of these word portion..... :)
        % The full svHealth description can be foudn via binArray(1,65:70);
        out.Ephem_V2(ephemCount,:).svHealth = binArray(1,65);
        
        % When bit 1 of word four is a "1", it shall indicate that the LNAV data stream was commanded OFF
        %on the P-code of the in-phase component of the L2 channel.
        out.Ephem_V2(ephemCount,:).l2PDataFlag = binArray(1,73);
        
%         out.Ephem_V2(ephemCount,:).reserved1 = binArray(1,74:96);
%         out.Ephem_V2(ephemCount,:).reserved2 = binArray(1,97:120);
%         out.Ephem_V2(ephemCount,:).reserved3 = binArray(1,121:144);
%         out.Ephem_V2(ephemCount,:).reserved4 = binArray(1,145:160);
        
        out.Ephem_V2(ephemCount,:).T_GD = twoscomp(binArray(1,161:168)) * 2^(-31);        
        out.Ephem_V2(ephemCount,:).IODC = bin2dec([binArray(1,71:72) binArray(1,169:176)]);
        out.Ephem_V2(ephemCount,:).t_oc = bin2dec(binArray(1,177:192)) * 2^(4);
        out.Ephem_V2(ephemCount,:).a_f2 = twoscomp(binArray(193:200)) * 2^(-55);
        out.Ephem_V2(ephemCount,:).a_f1 = twoscomp(binArray(201:216)) * 2^(-43);
        out.Ephem_V2(ephemCount,:).a_f0 = twoscomp(binArray(217:238)) * 2^(-31);
        
        out.Ephemeris(ephemCount,:).TOW = bin2dec(binArray(1,21:37))*4;
        out.Ephemeris(ephemCount,:).SubframeID = bin2dec(binArray(1,40:42));
        out.Ephemeris(ephemCount,:).L2Code = bin2dec(binArray(1,59:60));
        out.Ephemeris(ephemCount,:).weekNumber = bin2dec(binArray(1,49:58)) + 1024;
        out.Ephemeris(ephemCount,:).Accuracy = bin2dec(binArray(1,61:64));
        out.Ephemeris(ephemCount,:).health = bin2dec(binArray(1,65:70));
        out.Ephemeris(ephemCount,:).T_GD = twoscomp(binArray(1,161:168)) * 2^(-31);
        out.Ephemeris(ephemCount,:).IODC = bin2dec([binArray(1,71:72) binArray(1,169:176)]);
        out.Ephemeris(ephemCount,:).t_oc = bin2dec(binArray(1,177:192)) * 2^(4)
        out.Ephemeris(ephemCount,:).a_f2 = twoscomp(binArray(193:200)) * 2^(-55);
        out.Ephemeris(ephemCount,:).a_f1 = twoscomp(binArray(201:216)) * 2^(-43);
        out.Ephemeris(ephemCount,:).a_f0 = twoscomp(binArray(217:238)) * 2^(-31);
        
% q = quantizer([4 2]);
% b = [binArray(217:238)];        
        
        %%%%%%%%%%%%%%%%%%% Subframe 2 %%%%%%%%%%%%%%%%%%%%%%%
        subframe2 = structure{n}.Ephemeris.Subframe2;
        sf2Bin = dec2bin(subframe2);
        binArray = [sf2Bin(1,:) sf2Bin(2,:) sf2Bin(3,:) sf2Bin(4,:) sf2Bin(5,:) sf2Bin(6,:) ...
            sf2Bin(7,:) sf2Bin(8,:) sf2Bin(9,:) sf2Bin(10,:) sf2Bin(11,:) sf2Bin(12,:) ...
            sf2Bin(13,:) sf2Bin(14,:) sf2Bin(15,:) sf2Bin(16,:) sf2Bin(17,:) sf2Bin(18,:) ...
            sf2Bin(19,:) sf2Bin(20,:) sf2Bin(21,:) sf2Bin(22,:) sf2Bin(23,:) sf2Bin(24,:) ...
            sf2Bin(25,:) sf2Bin(26,:) sf2Bin(27,:) sf2Bin(28,:) sf2Bin(29,:) sf2Bin(30,:)];
        
        out.Ephemeris(ephemCount,:).IODE_sf2 = bin2dec(binArray(1,49:56));
        out.Ephemeris(ephemCount,:).C_rs = twoscomp(binArray(1,57:72)) * 2^(-5);
        out.Ephemeris(ephemCount,:).deltan = twoscomp(binArray(1,73:88)) * 2^(-43) * gpsPi;
        out.Ephemeris(ephemCount,:).M_0 = twoscomp(binArray(1,89:120)) * 2^(-31) * gpsPi;
        out.Ephemeris(ephemCount,:).C_uc = twoscomp(binArray(1,121:136)) * 2^(-29);
        out.Ephemeris(ephemCount,:).e = bin2dec(binArray(1,137:168)) * 2^(-33);
        out.Ephemeris(ephemCount,:).C_us = twoscomp(binArray(1,169:184)) * 2^(-29);
        out.Ephemeris(ephemCount,:).sqrtA = bin2dec(binArray(1,185:216)) * 2^(-19);
        out.Ephemeris(ephemCount,:).t_oe = bin2dec(binArray(1,217:232)) * 2^(4);
        
        %%%%%%%%%%%%%%%%%%% Subframe 3 %%%%%%%%%%%%%%%%%%%%%%%
        subframe3 = structure{n}.Ephemeris.Subframe3;
        sf3Bin = dec2bin(subframe3);
        binArray = [sf3Bin(1,:) sf3Bin(2,:) sf3Bin(3,:) sf3Bin(4,:) sf3Bin(5,:) sf3Bin(6,:) ...
            sf3Bin(7,:) sf3Bin(8,:) sf3Bin(9,:) sf3Bin(10,:) sf3Bin(11,:) sf3Bin(12,:) ...
            sf3Bin(13,:) sf3Bin(14,:) sf3Bin(15,:) sf3Bin(16,:) sf3Bin(17,:) sf3Bin(18,:) ...
            sf3Bin(19,:) sf3Bin(20,:) sf3Bin(21,:) sf3Bin(22,:) sf3Bin(23,:) sf3Bin(24,:) ...
            sf3Bin(25,:) sf3Bin(26,:) sf3Bin(27,:) sf3Bin(28,:) sf3Bin(29,:) sf3Bin(30,:)];
        
        out.Ephemeris(ephemCount,:).C_ic = twoscomp(binArray(1,49:64)) * 2^(-29);
        out.Ephemeris(ephemCount,:).omega_0 = twoscomp(binArray(1,65:96)) * 2^(-31) * gpsPi;
        out.Ephemeris(ephemCount,:).C_is = twoscomp(binArray(1,97:112)) * 2^(-29);
        out.Ephemeris(ephemCount,:).i_0 = twoscomp(binArray(1,113:144)) * 2^(-31) * gpsPi;
        out.Ephemeris(ephemCount,:).C_rc = twoscomp(binArray(1,145:160)) * 2^(-5);
        out.Ephemeris(ephemCount,:).omega = twoscomp(binArray(1,161:192)) * 2^(-31) * gpsPi;
        out.Ephemeris(ephemCount,:).omegaDot = twoscomp(binArray(1,193:216)) * 2^(-43) * gpsPi;
        out.Ephemeris(ephemCount,:).IODE_sf3 = bin2dec(binArray(1,217:224));
        out.Ephemeris(ephemCount,:).iDot = twoscomp(binArray(1,225:238)) * 2^(-43) * gpsPi;
              
        prnSaved(1) = prn;
        ephemCount = ephemCount + 1;
    end
    
    prnSaved = [];
    prn = 1;
    if ~any(ismember(prnSaved,prn)) && ephemOn
        
        rosSec = structure{n}.Header.Stamp.Sec;
        
        out.Ephemeris(ephemCount,:).prn = double(prn);
        out.Ephemeris(ephemCount,:).rosSec = double(rosSec);
        out.Ephemeris(ephemCount,:).gpsSec = double(gpsSec);
        
        % Decoding ephemeris subframes
        %%%%%%%%%%%%%%%%%%% Subframe 1 %%%%%%%%%%%%%%%%%%%%%%%
        subframe1 = structure{n}.Ephemeris.Subframe1;
        sf1Bin = dec2bin(subframe1);
        binArray = [sf1Bin(1,:) sf1Bin(2,:) sf1Bin(3,:) sf1Bin(4,:) sf1Bin(5,:) sf1Bin(6,:) ...
            sf1Bin(7,:) sf1Bin(8,:) sf1Bin(9,:) sf1Bin(10,:) sf1Bin(11,:) sf1Bin(12,:) ...
            sf1Bin(13,:) sf1Bin(14,:) sf1Bin(15,:) sf1Bin(16,:) sf1Bin(17,:) sf1Bin(18,:) ...
            sf1Bin(19,:) sf1Bin(20,:) sf1Bin(21,:) sf1Bin(22,:) sf1Bin(23,:) sf1Bin(24,:) ...
            sf1Bin(25,:) sf1Bin(26,:) sf1Bin(27,:) sf1Bin(28,:) sf1Bin(29,:) sf1Bin(30,:)];
        
        out.Ephemeris(ephemCount,:).TOW = bin2dec(binArray(1,21:37))*4;
        out.Ephemeris(ephemCount,:).SubframeID = bin2dec(binArray(1,40:42));
        out.Ephemeris(ephemCount,:).L2Code = bin2dec(binArray(1,59:60));
        out.Ephemeris(ephemCount,:).weekNumber = bin2dec(binArray(1,49:58)) + 1024;
        out.Ephemeris(ephemCount,:).Accuracy = bin2dec(binArray(1,61:64));
        out.Ephemeris(ephemCount,:).health = bin2dec(binArray(1,65:70));
        out.Ephemeris(ephemCount,:).T_GD = twoscomp(binArray(1,161:168)) * 2^(-31);
        out.Ephemeris(ephemCount,:).IODC = bin2dec([binArray(1,71:72) binArray(1,169:176)]);
        out.Ephemeris(ephemCount,:).t_oc = bin2dec(binArray(1,177:192)) * 2^(4);
        out.Ephemeris(ephemCount,:).a_f2 = twoscomp(binArray(193:200)) * 2^(-55);
        out.Ephemeris(ephemCount,:).a_f1 = twoscomp(binArray(201:216)) * 2^(-43);
        out.Ephemeris(ephemCount,:).a_f0 = twoscomp(binArray(217:238)) * 2^(-31);
        
        %%%%%%%%%%%%%%%%%%% Subframe 2 %%%%%%%%%%%%%%%%%%%%%%%
        subframe2 = structure{n}.Ephemeris.Subframe2;
        sf2Bin = dec2bin(subframe2);
        binArray = [sf2Bin(1,:) sf2Bin(2,:) sf2Bin(3,:) sf2Bin(4,:) sf2Bin(5,:) sf2Bin(6,:) ...
            sf2Bin(7,:) sf2Bin(8,:) sf2Bin(9,:) sf2Bin(10,:) sf2Bin(11,:) sf2Bin(12,:) ...
            sf2Bin(13,:) sf2Bin(14,:) sf2Bin(15,:) sf2Bin(16,:) sf2Bin(17,:) sf2Bin(18,:) ...
            sf2Bin(19,:) sf2Bin(20,:) sf2Bin(21,:) sf2Bin(22,:) sf2Bin(23,:) sf2Bin(24,:) ...
            sf2Bin(25,:) sf2Bin(26,:) sf2Bin(27,:) sf2Bin(28,:) sf2Bin(29,:) sf2Bin(30,:)];
        
        out.Ephemeris(ephemCount,:).IODE_sf2 = bin2dec(binArray(1,49:56));
        out.Ephemeris(ephemCount,:).C_rs = twoscomp(binArray(1,57:72)) * 2^(-5);
        out.Ephemeris(ephemCount,:).deltan = twoscomp(binArray(1,73:88)) * 2^(-43) * gpsPi;
        out.Ephemeris(ephemCount,:).M_0 = twoscomp(binArray(1,89:120)) * 2^(-31) * gpsPi;
        out.Ephemeris(ephemCount,:).C_uc = twoscomp(binArray(1,121:136)) * 2^(-29);
        out.Ephemeris(ephemCount,:).e = bin2dec(binArray(1,137:168)) * 2^(-33);
        out.Ephemeris(ephemCount,:).C_us = twoscomp(binArray(1,169:184)) * 2^(-29);
        out.Ephemeris(ephemCount,:).sqrtA = bin2dec(binArray(1,185:216)) * 2^(-19);
        out.Ephemeris(ephemCount,:).t_oe = bin2dec(binArray(1,217:232)) * 2^(4);
        
        %%%%%%%%%%%%%%%%%%% Subframe 3 %%%%%%%%%%%%%%%%%%%%%%%
        subframe3 = structure{n}.Ephemeris.Subframe3;
        sf3Bin = dec2bin(subframe3);
        binArray = [sf3Bin(1,:) sf3Bin(2,:) sf3Bin(3,:) sf3Bin(4,:) sf3Bin(5,:) sf3Bin(6,:) ...
            sf3Bin(7,:) sf3Bin(8,:) sf3Bin(9,:) sf3Bin(10,:) sf3Bin(11,:) sf3Bin(12,:) ...
            sf3Bin(13,:) sf3Bin(14,:) sf3Bin(15,:) sf3Bin(16,:) sf3Bin(17,:) sf3Bin(18,:) ...
            sf3Bin(19,:) sf3Bin(20,:) sf3Bin(21,:) sf3Bin(22,:) sf3Bin(23,:) sf3Bin(24,:) ...
            sf3Bin(25,:) sf3Bin(26,:) sf3Bin(27,:) sf3Bin(28,:) sf3Bin(29,:) sf3Bin(30,:)];
        
        out.Ephemeris(ephemCount,:).C_ic = twoscomp(binArray(1,49:64)) * 2^(-29);
        out.Ephemeris(ephemCount,:).omega_0 = twoscomp(binArray(1,65:96)) * 2^(-31) * gpsPi;
        out.Ephemeris(ephemCount,:).C_is = twoscomp(binArray(1,97:112)) * 2^(-29);
        out.Ephemeris(ephemCount,:).i_0 = twoscomp(binArray(1,113:144)) * 2^(-31) * gpsPi;
        out.Ephemeris(ephemCount,:).C_rc = twoscomp(binArray(1,145:160)) * 2^(-5);
        out.Ephemeris(ephemCount,:).omega = twoscomp(binArray(1,161:192)) * 2^(-31) * gpsPi;
        out.Ephemeris(ephemCount,:).omegaDot = twoscomp(binArray(1,193:216)) * 2^(-43) * gpsPi;
        out.Ephemeris(ephemCount,:).IODE_sf3 = bin2dec(binArray(1,217:224));
        out.Ephemeris(ephemCount,:).iDot = twoscomp(binArray(1,225:238)) * 2^(-43) * gpsPi;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        prnSaved(n) = prn;
        ephemCount = ephemCount + 1;

    end
    k = k + 1;
end

% chk = out.Ephemeris(:,:,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Novatel pseudorange
bag_sel = select(bag, 'Topic',topicPrefix + '/rawMeasurementsTagged');
structure = readMessages(bag_sel,'DataFormat','struct');

for n = 1:length(structure)
    gpsSec(n) = structure{n}.Tags.GpsTime.GpsSeconds;
    rosSec = structure{n}.Header.Stamp.Sec;
    
    l1counter = 1;
    l2counter = 1;
    for k = 1:length(structure{n}.Measurements)
        
        if double(structure{n}.Measurements(k).Frequency.Type) == 1
            
        out.range_L1(n).prn(l1counter) = double(structure{n}.Measurements(k).Prn);
        out.range_L1(n).gpsSec(l1counter) = gpsSec(n);
        out.range_L1(n).rosSec(l1counter) = rosSec;
        out.range_L1(n).Pseudorange(l1counter)  = double(structure{n}.Measurements(k).Pseudorange);          
        out.range_L1(n).PseudorangeVariance(l1counter) = double(structure{n}.Measurements(k).PseudorangeVariance);
        out.range_L1(n).CarrierPhase(l1counter) = double(structure{n}.Measurements(k).CarrierPhase);
        out.range_L1(n).carrierPhaseVariance(l1counter) = double(structure{n}.Measurements(k).CarrierPhaseVariance);
        out.range_L1(n).doppler(l1counter)  = double(structure{n}.Measurements(k).Doppler);
        out.range_L1(n).CnO(l1counter) = double(structure{n}.Measurements(k).CarrierToNoise);
        out.range_L1(n).lossOLock(l1counter) = double(structure{n}.Measurements(k).LossOfLock);
        
        
        l1counter = l1counter + 1;
        
        end
        
        if double(structure{n}.Measurements(k).Frequency.Type) == 2
            
        out.range_L2(n).prn(l2counter) = double(structure{n}.Measurements(k).Prn);
        out.range_L2(n).gpsSec(l2counter) = gpsSec(n);
        out.range_L2(n).rosSec(l2counter) = rosSec;
        out.range_L2(n).Pseudorange(l2counter)  = double(structure{n}.Measurements(k).Pseudorange);          
        out.range_L2(n).PseudorangeVariance(l2counter) = double(structure{n}.Measurements(k).PseudorangeVariance);
        out.range_L2(n).CarrierPhase(l2counter) = double(structure{n}.Measurements(k).CarrierPhase);
        out.range_L2(n).carrierPhaseVariance(l2counter) = double(structure{n}.Measurements(k).CarrierPhaseVariance);
        out.range_L2(n).doppler(l2counter)  = double(structure{n}.Measurements(k).Doppler);
        out.range_L2(n).CnO(l2counter) = double(structure{n}.Measurements(k).CarrierToNoise);
        
        l2counter = l2counter + 1;
        
        end
        
        if l1counter - l2counter > 1
            
            ty = 1;
            
        end
        
        check(k,n) = double(structure{n}.Measurements(k).Pseudorange);
    end
end

%%
%%%%%%%%%%
    
    topic = select(bag, 'Topic',topicPrefix + '/gpsTimeTagged');
    structure = readMessages(topic,'DataFormat', 'struct');
    out.gpsTimeTagged.time = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1e-9,structure)';
    out.gpsTimeTagged.gpsWeek = cellfun(@(m) double(m.GpsTime.GpsWeek),structure)';
    out.gpsTimeTagged.gpsSeconds = cellfun(@(m) double(m.GpsTime.GpsSeconds),structure)';
   
    topic = select(bag, 'Topic',topicPrefix + '/llhPositionTagged');
    structure = readMessages(topic,'DataFormat', 'struct');
    out.llhPositionTagged.time = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1e-9,structure)';
    out.llhPositionTagged.gpsWeek = cellfun(@(m) double(m.Tags.GpsTime.GpsWeek),structure)';
    out.llhPositionTagged.gpsSeconds = cellfun(@(m) double(m.Tags.GpsTime.GpsSeconds),structure)';
    out.llhPositionTagged.latitude = cellfun(@(m) double(m.LlhPosition.Latitude),structure)';
    out.llhPositionTagged.longitude = cellfun(@(m) double(m.LlhPosition.Longitude),structure)';
    out.llhPositionTagged.altitude = cellfun(@(m) double(m.LlhPosition.Altitude),structure)';
    out.llhPositionTagged.horizontalAccuracy = cellfun(@(m) double(m.LlhPosition.HorizontalAccuracy),structure)';
    out.llhPositionTagged.verticalAccuracy = cellfun(@(m) double(m.LlhPosition.VerticalAccuracy),structure)';

    %TODO: Status, Covariances
    topic = select(bag, 'Topic',topicPrefix + '/navSatFix');
    structure = readMessages(topic,'DataFormat', 'struct');
    out.navSatFix.time = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1e-9,structure)';
    out.navSatFix.latitude = cellfun(@(m) double(m.Latitude),structure)';
    out.navSatFix.longitude = cellfun(@(m) double(m.Longitude),structure)';
    out.navSatFix.altiude = cellfun(@(m) double(m.Altitude),structure)';
    
    % SV Positions
    topic = select(bag, 'Topic',topicPrefix + '/svStateTagged');
    structure = readMessages(topic,'DataFormat', 'struct');
    prnCounter = 1;   % This should reset to one after every sampling period
    epochCounter = 1; % This should incrrease monotoincally via sampling period
    previousTimeStamp = [];
    for kk = 1:length(structure)
        
        currentTimeStamp = double(structure{kk}.Tags.GpsTime.GpsSeconds);
        if ~isempty(previousTimeStamp)
            % Check for non-zero dt
            dt = currentTimeStamp - previousTimeStamp;
            
            if dt ~= 0
                % A new observation period has occured!!!
                epochCounter = epochCounter + 1;
                prnCounter = 1;
                
            end
            
        end
            
        
        out.svPosition(epochCounter).gpsSec(prnCounter) =  double(structure{kk}.Tags.GpsTime.GpsSeconds);
        out.svPosition(epochCounter).Prn(prnCounter) = double(structure{kk}.SvState.Prn);
        out.svPosition(epochCounter).positionX(prnCounter) = double(structure{kk}.SvState.Position.X);
        out.svPosition(epochCounter).positionY(prnCounter) = double(structure{kk}.SvState.Position.Y);
        out.svPosition(epochCounter).positionZ(prnCounter) = double(structure{kk}.SvState.Position.Z);
        out.svPosition(epochCounter).svClockCorrection(prnCounter) = double(structure{kk}.SvState.SvClockCorrection);
        
        % Finshing extracting information from current index...Lets update
        previousTimeStamp = out.svPosition(epochCounter).gpsSec(prnCounter);
        prnCounter = prnCounter +1;
        
    end

   

    %TODO: covariances and rest of topics
    topic = select(bag, 'Topic',topicPrefix + '/odom');
    structure = readMessages(topic,'DataFormat', 'struct');
    
    out.odom.Pose.X = cellfun(@(m) double(m.Pose.Pose.Position.X),structure)';
    out.odom.Pose.Y = cellfun(@(m) double(m.Pose.Pose.Position.Y),structure)';
    out.odom.Pose.Z = cellfun(@(m) double(m.Pose.Pose.Position.Z),structure)';
    out.odom.Twist.X = cellfun(@(m) double(m.Twist.Twist.Linear.X),structure)';
    out.odom.Twist.Y = cellfun(@(m) double(m.Twist.Twist.Linear.Y),structure)';
    out.odom.Twist.Z = cellfun(@(m) double(m.Twist.Twist.Linear.Z),structure)';
    
    % RR custom message
%     bag_sel = select(bag, 'Topic','novatel_novatel/novatelGnssRawMessageTagged');
%     structure = readMessages(bag_sel,'DataFormat','struct');
    
    

end



function out = parseNovatelOdom(structure,ref_lla)

    out.time = cellfun(@(m) double(m.Header.Stamp.Sec) + double(m.Header.Stamp.Nsec)*1e-9,structure)';
    out.positionEcef = [cellfun(@(m) double(m.Pose.Pose.Position.X),structure)';
                                       cellfun(@(m) double(m.Pose.Pose.Position.Y),structure)';
                                       cellfun(@(m) double(m.Pose.Pose.Position.Z),structure)'];
    out.velocityEcef = [cellfun(@(m) double(m.Twist.Twist.Linear.X),structure)';
                                       cellfun(@(m) double(m.Twist.Twist.Linear.Y),structure)';
                                       cellfun(@(m) double(m.Twist.Twist.Linear.Z),structure)'];
                                   
    out.speed = sum(out.velocityEcef.^2).^0.5;
    
    
    % --- Get reference lat long alt for transforming to local level frame
    reflat = ref_lla(1);
    reflon = ref_lla(2);
    refalt = ref_lla(3);
    
    % --- transform ECEF XYZ
    for k = 1:length(out.time)
        % --- position to LLA
        if exist('wgsxyz2lla')
            [pos_lat,pos_lon,pos_alt] = wgsxyz2lla(out.positionEcef(:,k)); % LLA
            out.positionLla(:,k) = [pos_lat;pos_lon;pos_alt];
        end
        
        % --- position to NED
        if exist('wgsxyz2enu')
            pos_enu=wgsxyz2enu(out.positionEcef(:,k),reflat,reflon,refalt);
            out.positionNed(:,k) = [pos_enu(2);pos_enu(1);-pos_enu(3)];
        end
        
        % --- velocity to NED
        if exist('ecef2enuv')
            [vE,vN,vU] = ecef2enuv(out.velocityEcef(1,k), ...
                                   out.velocityEcef(2,k), ...
                                   out.velocityEcef(3,k), ...
                                   reflat, reflon);
                               
            out.velocityNed(:,k) = [vN;vE;-vU];
            
            if out.speed(k) > 0.1
                out.course(:,k) = atan2(out.velocityNed(2,k),out.velocityNed(1,k));
            else
                out.course(:,k) = nan;
            end
        end
    end

    

end

