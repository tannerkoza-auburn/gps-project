function [Meas_Matrix] = combineMats(imuMat,sv_UWB_Mat)
%{
  Take in UWB mats, satellite mats, and IMU mats and combine into a
  single measurement matrix

  Inputs: 
    AS STRINGS:
    - 'imuMat'
    - 'svMat'
    - 'UWBmat'

    All associated matrices must have the associated ROS time tagged in
    their first column!!

   Output:
    - Meas_Matrix = [rosTime 
%}

IMU = load(imuMat);
svObs = load(sv_UWB_Mat);

end

