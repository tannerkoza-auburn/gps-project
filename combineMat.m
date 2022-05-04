%{
Take in UWB mats, satellite mats, and IMU mats and combine into a
single measurement matrix
%}


IMUmat = kvh.IMU;


 



save('data/combinedMats/figure8_CombinedMat', 'MeasMatrix');