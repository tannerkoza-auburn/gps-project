function eph = decodeEphem(eph)
%% Initialization
gpsPi = 3.1415926535898;

%% Subframe 1
sub1 = dec2bin(eph.subframe1);
subframe1 = char(strjoin(string(sub1(1:30,:)),''));

eph.TOW         = bin2dec(subframe1(25:41)) * 6 - 30;
eph.weekNumber  = bin2dec(subframe1(49:58)) + 2*1024;
eph.accuracy    = bin2dec(subframe1(61:64));
eph.health      = bin2dec(subframe1(65:70));
eph.T_GD        = twosComp2dec(subframe1(161:168)) * 2^(-31);
eph.IODC        = bin2dec([subframe1(71:72) subframe1(169:176)]);
eph.t_oc        = bin2dec(subframe1(177:192)) * 2^4;
eph.a_f2        = twosComp2dec(subframe1(193:200)) * 2^(-55);
eph.a_f1        = twosComp2dec(subframe1(201:216)) * 2^(-43);
eph.a_f0        = twosComp2dec(subframe1(217:238)) * 2^(-31);

%% Subframe 2
sub2 = dec2bin(eph.subframe2);
subframe2 = char(strjoin(string(sub2(1:30,:)),''));

eph.IODE_sf2    = bin2dec(subframe2(49:56));
eph.C_rs        = twosComp2dec(subframe2(57:72)) * 2^(-5);
eph.deltan      = ...
    twosComp2dec(subframe2(73:88)) * 2^(-43) * gpsPi;
eph.M_0         = ...
    twosComp2dec(subframe2(89:120)) ...
    * 2^(-31) * gpsPi;
eph.C_uc        = twosComp2dec(subframe2(121:136)) * 2^(-29);
eph.e           = ...
    bin2dec(subframe2(137:168)) ...
    * 2^(-33);
eph.C_us        = twosComp2dec(subframe2(169:184)) * 2^(-29);
eph.sqrtA       = ...
    bin2dec(subframe2(185:216)) ...
    * 2^(-19);
eph.t_oe        = bin2dec(subframe2(217:232)) * 2^4;

%% Subframe 3
sub3 = dec2bin(eph.subframe3);
subframe3 = char(strjoin(string(sub3(1:30,:)),''));

eph.C_ic        = twosComp2dec(subframe3(49:64)) * 2^(-29);
eph.omega_0     = ...
    twosComp2dec(subframe3(65:96)) ...
    * 2^(-31) * gpsPi;
eph.C_is        = twosComp2dec(subframe3(97:112)) * 2^(-29);
eph.i_0         = ...
    twosComp2dec(subframe3(113:144)) ...
    * 2^(-31) * gpsPi;
eph.C_rc        = twosComp2dec(subframe3(145:160)) * 2^(-5);
eph.omega       = ...
    twosComp2dec(subframe3(161:192)) ...
    * 2^(-31) * gpsPi;
eph.omegaDot    = twosComp2dec(subframe3(193:216)) * 2^(-43) * gpsPi;
eph.IODE_sf3    = bin2dec(subframe3(217:224));
eph.iDot        = twosComp2dec(subframe3(225:238)) * 2^(-43) * gpsPi;

end