function [C_n2e] = NED_to_ECEF(lat, lon)


%{
    Simple function to calculate NED to ECEF rotation matrix. Useful since
    this transformation allows to define initial attitude in roll,pitch,yaw
    wrt local navigation (NED) and then propagate in ECEF. Can also be used
    to extract NED attitude easily

    Ben Jones
    3/31/21
%}

s_lt = sind(lat);  s_ln = sind(lon);
c_lt = cosd(lat);  c_ln = cosd(lon);
    
C_n2e = [-s_lt*c_ln     -s_ln       -c_lt*c_ln;
         -s_lt*s_ln      c_ln       -c_lt*s_ln;
         c_lt            0          -s_lt];
     
     
cos_lat = cosd(lat);
sin_lat = sind(lat);
cos_long = cosd(lon);
sin_long = sind(lon);
     
     
C_e_n = [-sin_lat * cos_long, -sin_lat * sin_long,  cos_lat;...
                   -sin_long,            cos_long,        0;...
         -cos_lat * cos_long, -cos_lat * sin_long, -sin_lat];     
     
     
end 