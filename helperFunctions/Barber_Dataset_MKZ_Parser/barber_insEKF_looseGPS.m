clear; clc; close all hidden;

%% RUN SETTINGS
    bag_filename = 'barber_run4.bag';

    % DEFINE SETTINGS
        % sensor message use settings
            % etalin
                settings.use_etalin_imu = true;
            
            % novatelMKZ
                settings.use_novatelMKZ_pos = false; 
        
            % septentrio
                settings.use_septentrio_att = false;
                settings.use_septentrio_vel = true;
                count = 0;
                settings.use_septentrio_pos = false;
        
            % vehicle
                settings.use_vehicle_imu = false;
                settings.use_vehicle_pos = false;
                settings.use_vehicle_vel = false;

        % plotting settings
            settings.plot_ekf_info = true;

        % measurement analysis settings
            settings.plot_imu_analysis = true;

        % run preference settings
            settings.endFactor = 0.1;
            settings.startFactor = 0;


%% VEHICLE MODEL PARAMETERS 

% geometric
params.vdm.a = 1.257;              % [meters] 
params.vdm.b = 1.593;              % [meters] 
params.vdm.L = 2.850;              % [meters] 
params.vdm.trackw_f = 1.594;       % [meters]  
params.vdm.trackw_r = 1.594;       % [meters]

% inertial
params.vdm.m = 1542;               % [kg]          % total mass
params.vdm.Izz = 1000;             % [kg*m^2]      % z axis moment of inertia

% tire
params.vdm.Caf = 120000;           % [N/rad]       % front tire lateral slip
params.vdm.Car = 184600;           % [N/rad]       % rear tire lateral slip
params.vdm.tire_radius = [];       % [meters]

% steering
params.vdm.steer_ratio = [];       % [unitless]
params.vdm.steer_offset = [];      % [degrees?]

%% SENSOR PARAMETERS

% etalin IMU
params.etalin.imu = [];

% vehicle IMU
params.vehicle.imu = [];

%% CONSTRUCT AND INITIALIZE CLASS 
    ekf = insEKF;

% select operative settings
    ekf.settings.gaussMarkov = false;
    ekf.settings.zeroVelUpdt = false;
    ekf.settings.zeroRateUpdt = false;

    % temporary values
    ekf.lla_ = [33.5337563823342	-86.6187961345039	166.269330364572]';
    ekf.pos_ = lla2ecef(ekf.lla_')';
    ekf.vel_ = zeros(3,1);

    temp1 = [-0.9998    0.0150   -0.0119;
             -0.0144   -0.9988   -0.0473;
             -0.0126   -0.0471    0.9988]; % from accel wahba

    temp2 = [-0.9055    0.4202   -0.0593;
             -0.4238   -0.9024    0.0774;
             -0.0210    0.0952    0.9952]; % from gyro wahba
    temp2 = rotx(-2)*temp2;


    r = 0.7;
    Retalin = r.*temp1 + (1-r).*temp2;

    Rb2n = Rz(135);
%     Rb2n = Rx(2)*Ry(-0.5)*Rz(122.9036);
%         Rb2n = Rx(0.5)*Ry(-0.5)*Rz(135);
    
    ekf.att_ = ekf.ned2ecef_rotMat(ekf.lla_)*Rb2n;
    clearvars acc rsx rsy rsz roll pitch Rb2n

    % initialize IMU
    % TEMPORARY VALUES
        % accelerometer bias stability and measurement noise in body frame
        ekf.imu_params.accel_bias_s = [0 0 0]'; %[-0.1874   0.3297   -0.0286]'; %[-0.1874    0.35   -0.0286]'; %0.3297
        ekf.imu_params.accel_var = [2.5 2.5 2.5]'*10^-2;
%         ekf.imu_params.accel_bias_s = [0 0 0]'; 
%         ekf.imu_params.accel_bias_is = [0 0 0]'; 
%         ekf.imu_params.accel_var = (10^0)*[2.5 2.5 2.5]';
                       
        % gyro bias stability and measurement noise in sensor(body?) frame
        ekf.imu_params.gyro_bias_s = [0 0 0]'; %[-0.0006 0.0065 -0.006]'; 
        ekf.imu_params.gyro_var = [1.6129 1.6129 1.6129]'*10^-2;
%         ekf.imu_params.gyro_bias_s = [0 0 0]'; 
%         ekf.imu_params.gyro_bias_is = [0 0 0]'; 
%         ekf.imu_params.gyro_var = [0.008 0.008 0.1701]' * 10^-2;

%% CALLBACK LOOP

    % LOAD AND PREP BAG
%         run_topics = {'/vectornav/IMU','/etalin/Imu','/vehicle/imu/data_raw','/novatelMKZ/llhPositionTagged','/septentrio/attitude_euler'}; %,...
        run_topics = {'/etalin/Imu',...
                      '/vehicle/imu/data_raw',...
                      '/vectornav/IMU',...
                      '/novatelMKZ/llhPositionTagged',...
                      '/septentrio/attitude_cov_euler','/septentrio/attitude_euler',...
                      '/septentrio/pos_cov_cartesian','/septentrio/vel_cov_cartesian','/septentrio/pvt_cartesian',...
                      '/vehicle/gps/fix', '/vehicle/gps/vel',...
                      '/vehicle/joint_states'};
    
        bag = rosbag(bag_filename);
        bagInfo = rosbag('info',bag_filename);
        bsel = select(bag,'Topic',run_topics);
        fprintf('Reading Ros Messages into Structure ... ')
        msgStruct = readMessages(bsel,'DataFormat','struct');
        fprintf('Done \n')

    % DECLARE CONDITIONAL BOOLS
        bool.initialized = false;
        bool.first_timeUpdt = false;
        bool.runKalmanUpdt = false;
    
    % DETERMINE START AND END
        startNum = floor(settings.startFactor*bsel.NumMessages) + 1;
        endNum = floor(settings.endFactor*bsel.NumMessages); % 0.15% gets to the end of the second turn
        
    % BEGIN CALLBACK LOOP
    fprintf('Begin Callback Loop \n')
    for k = startNum:endNum
        i = k - startNum + 1;

%         %TEMPORARY: for debug, if car is moving greater than 100m/s exit the for loop
%         if norm(ekf.vel_) > 1.2*10^2 
%             warning('velocity over 100 meters per second')
%             disp(i);
%             break
%         end

        % RESET LOOP SPECIFIC CONDITIONAL BOOLS
        bool.runKalmanUpdt = false;

%         % CHECK RUN SPECIFIC CONDITIONAL BOOLS
%         if all(ekf.pos_ ~= zeros(3,1)) && all(ekf.vel_ ~= zeros(3,1)) && any(any(ekf.att_ ~= eye(3)))
            bool.initialized = true;
%         end


        % INPUT A CHECK TO SEE IF TIME HAS CHANGED
%         time = bsel.MessageList.Time(i);
%         if i > 1
%             if time == old_time
%                 fprintf(strcat(string(topic),' and ',string(bsel.MessageList.Topic(i)),' have the same time. \n'));
%                 pause
%             end
%         end
%         old_time = time;


        % READ IN CURRENT ROS MESSAGE
        loop.topic = bsel.MessageList.Topic(k);
        loop.msgType = bsel.MessageList.MessageType(k);
%         loop.msgStruct = readMessages(bsel,i,'DataFormat','struct');
        time = bsel.MessageList.Time(k);
        data = parseRosMsgs(msgStruct(k),loop.msgType,0,0);

        % SAVE MESSAGE TO STRUCTURE
        measField = char(loop.topic);
        measField(strfind(measField,'/')) = '_';
        measField = measField(2:end);
        if ~exist('meas',"var")
            meas.(measField) = data;
        end
        if ~isfield(meas,measField)
            meas.(measField) = data;
        else
            meas.(measField)(end+1) = data;
        end
        clearvars measField
        
        % RUN APPROPRIATE CALLBACK
        swapXZ = [0 0 1; 0 1 0; 1 0 0];  % swap x and z axes
        swapXY = [0 1 0; 1 0 0; 0 0 1];  % swap x and y axes
        lh2rh = [1 0 0; 0 1 0; 0 0 -1];  % left hand to right hand
        turnAround = [-1 0 0; 0 -1 0; 0 0 1]; % turn 180 about z axis 

        switch loop.topic
        % account for lever arms later
        
            % ETALIN CALLBACKS
            case '/etalin/Imu'
                if settings.use_etalin_imu && bool.initialized
                    
                    f_ib_b = Retalin*data.linear_acceleration'; 
                    w_ib_b = Retalin*data.angular_velocity';
                    if ~bool.first_timeUpdt
                        ekf.time_ = time - 0; % temporary
                        bool.first_timeUpdt = true;
                    end
                    ekf.time_update_ins(ekf,f_ib_b,w_ib_b,time)
                    clearvars f_ib_b w_ib_b
                end

            % VEHICLE CALLBACKS
            case '/vehicle/imu/data_raw'
                if settings.use_vehicle_imu && bool.initialized
                    f_ib_b =  data.linear_acceleration'; 
                    w_ib_b =  [1 0 0; 0 1 0; 0 0 -1]  * data.angular_velocity';
                    if ~bool.first_timeUpdt
                        ekf.time_ = time - 0.01; %temporary
                        bool.first_timeUpdt = true;
                    end
                    ekf.time_update_ins(ekf,f_ib_b,w_ib_b,time)
                    clearvars f_ib_b w_ib_b
                end
                
            case '/vehicle/gps/fix'
                if settings.use_vehicle_pos && bool.first_timeUpdt
                    pos = lla2ecef(double(data.lla))';
                    posCov = [];
                    % TODO
                    ekf.position_updt(ekf,pos,posCov,time) 
                    bool.runKalmanUpdt = true;
                    clearvars pos posCov
                end
                
            case '/vehicle/gps/vel'
                if settings.use_vehicle_vel && bool.first_timeUpdt
                    count = count+1;
                    if mod(count,20) == 0
                        vel = double(data.linear); % check if ecef
                        velCov = eye(3).*1.5; % TODO fix accuracy
                        % TODO
                        ekf.position_updt(ekf,vel,velCov,time) 
                        bool.runKalmanUpdt = true;
                        clearvars vel velCov
                    end
                end
                    
            case '/vehicle/joint_states'
                meas.vehicle_joint_states(end+1) = data;
                % TODO
                %{"wheel_fl"	"wheel_fr"	"wheel_rl"	"wheel_rr"	"steer_fl"	"steer_fr"}
                steer_ang = data.Position(5:6);
                ws = data.Velocity(1:3);
                
            % NOVATELMKZ CALLBACKS
            case '/novatelMKZ/llhPositionTagged'
                if settings.use_novatelMKZ_pos && bool.first_timeUpdt
                    pos = lla2ecef(double(data.lla))';
                    posCov = eye(3).*max(double([data.horzAccuracy data.vertAccuracy])); % TODO fix accuracy
                    ekf.position_updt(ekf,pos,posCov,time) 
                    bool.runKalmanUpdt = true;
                    clearvars pos posCov
                end
                
            % SEPTENTRIO CALLBACKS
        
            case '/septentrio/attitude_euler'
                % TODO determine ekf source for Rn2e
                if settings.use_septentrio_att
                    Rn2e = ekf.ned2ecef_rotMat(ekf.lla_);
                    if abs(data.rpy(1)) > 360
                        data.rpy(1) = 0;
                    end
                    att = double(Rn2e * rotz(data.rpy(3))*roty(data.rpy(2))*rotx(data.rpy(1)));
                    if isfield(meas,'septentrio_attitude_cov_euler')
                        attCov = double(deg2rad(meas.septentrio_attitude_cov_euler(end).attCov)); % need to fix for cartesian
                        attCov(attCov < 0) = 0;
                    else
                        attCov = deg2rad([0,0,0;0,114.32732,0;0,0,34.275791]);
                    end
                    ekf.attitude_updt(ekf,att,attCov,time)
                    bool.runKalmanUpdt = true;
                    clearvars att attCov
                end
                
            case '/septentrio/pvt_cartesian'
                if settings.use_septentrio_pos && bool.first_timeUpdt
                    pos = double(data.position)';
                    posCov = double(meas.septentrio_pos_cov_cartesian(end).posCov(1:3,1:3));
                    ekf.position_updt(ekf,pos,posCov,time) 
                    bool.runKalmanUpdt = true;
                    clearvars pos posCov
                end
        
                if settings.use_septentrio_vel && bool.first_timeUpdt
                    vel = double(data.velocity)';
                    velCov = double(meas.septentrio_vel_cov_cartesian(end).velCov(1:3,1:3));
                    ekf.velocity_updt(ekf,vel,velCov,time)
                    bool.runKalmanUpdt = true;
                    clearvars vel velCov
                end
             
        end % switch case
        
        % RUN KALMAN UPDATE IF APPROPRIATE MEASUREMENT UPDATE OCCURED
        if bool.runKalmanUpdt
            ekf.kalmanUpdate(ekf);
            ekf.feedbackErrorStates(ekf);
        end
    

        % POST PROGRESS UPDATE
        loop.progress = floor(i./(0.05.*(endNum-startNum)));

        if i == 1
            loop.percent_count = 0;
            fprintf('Total Progress ... ');
            update = fprintf('0%% \n');
        end

        if  loop.progress > loop.percent_count
            loop.percent_count = loop.progress;
            fprintf(repmat('\b',1,update))
            update = fprintf('%d%%', loop.percent_count*5);
        end

    end % loop

    % CONVERT MEASUREMENT STRUCTURES TO TABLES
    fields = fieldnames(meas);
    for i = 1:length(fields)
        meas.(fields{i}) = struct2table(meas.(fields{i}),'AsArray',true);
    end
    fprintf(repmat('\b',1,update))
    fprintf('Done \n')

%% PLOTTING
    
if settings.plot_ekf_info
    fprintf('Plotting Results \n')
    % PLOT RESULTS
    warning('off','all')
    ekf.plot_EKF_info(ekf,50)
    warning('on','all')

    figure(9)
    geoplot(meas.novatelMKZ_llhPositionTagged.lla(:,1),meas.novatelMKZ_llhPositionTagged.lla(:,2),'b.')
    legend('EKF Position Solution', 'DGPS Measured Position')

    figure()
    ned = ekf.data.ekf_states.pos_ned;
    plot3(ned(2,:),ned(1,:),-ned(3,:),'.')
    hold on
    plot3(ned(2,1),ned(1,1),-ned(3,1),'g.','MarkerSize',10)
    nedgps = lla2ned(meas.novatelMKZ_llhPositionTagged.lla,meas.novatelMKZ_llhPositionTagged.lla(1,:),'ellipsoid');
    plot3(nedgps(:,2),nedgps(:,1),-nedgps(:,3),'r.','MarkerSize',10)
    legend('solution','start','gps')
    xlabel('east')
    ylabel('north')
    zlabel('up')
    axis equal
end

if settings.plot_imu_analysis
    fprintf('Plotting IMU Measurement Comparison \n')
    rotData = @(A,x) (A*x(1:height(x),:)')';

    init_time = min([meas.etalin_Imu.time;meas.vehicle_imu_data_raw.time;meas.vectornav_IMU.time]);

%     Retalin = eye(3);
    Rvehicle = eye(3);
    Rvectornav = eye(3);
    
    meas.etalin_Imu.angular_velocity =  rotData(Retalin,meas.etalin_Imu.angular_velocity);
    meas.etalin_Imu.linear_acceleration =  rotData(Retalin,meas.etalin_Imu.linear_acceleration);
    
    meas.vehicle_imu_data_raw.angular_velocity =  rotData(Rvehicle,meas.vehicle_imu_data_raw.angular_velocity);
    meas.vehicle_imu_data_raw.linear_acceleration =  rotData(Rvehicle,meas.vehicle_imu_data_raw.linear_acceleration);

    meas.vectornav_IMU.angular_velocity =  rotData(Rvectornav,meas.vectornav_IMU.angular_velocity);
    meas.vectornav_IMU.linear_acceleration =  rotData(Rvectornav,meas.vectornav_IMU.linear_acceleration);

    figure()
    for i = 1:3
            sp(i) = subplot(3,1,i); hold on; grid on; grid minor;
                plot(meas.etalin_Imu.time - init_time,             meas.etalin_Imu.angular_velocity(:,i),'.')
%                 plot(meas.vehicle_imu_data_raw.time - init_time,   meas.vehicle_imu_data_raw.angular_velocity(:,i),'.')
%                 plot(meas.vectornav_IMU.time - init_time,          meas.vectornav_IMU.angular_velocity(:,i),'.')
    end
%         legend('Etalin','MKZ','Vectornav')
        sgtitle('IMU Angular Velocities')



    figure()
    for i = 1:3
            sp(i) = subplot(3,1,i); hold on; grid on; grid minor;
                plot(meas.etalin_Imu.time - init_time,             meas.etalin_Imu.linear_acceleration(:,i),'.')
                plot(meas.vehicle_imu_data_raw.time - init_time,   meas.vehicle_imu_data_raw.linear_acceleration(:,i),'.')
                plot(meas.vectornav_IMU.time - init_time,          meas.vectornav_IMU.linear_acceleration(:,i),'.')
    end
        legend('Etalin','MKZ','Vectornav')
        sgtitle('IMU Linear Accelerations')
end

%% SUPPORTING FUNCTIONS

function R = Rx(theta)
    c = cosd(theta); s = sind(theta);

    R = [ 1  0  0 ;...
          0  c -s ;...
          0  s  c ];
end

function R = Ry(theta)
    c = cosd(theta); s = sind(theta);

    R = [ c  0  s ;...
          0  1  0 ;...
         -s  0  c ];
end

function R = Rz(theta)
    c = cosd(theta); s = sind(theta);

    R = [ c -s  0 ;...
          s  c  0 ;...
          0  0  1 ];
end


