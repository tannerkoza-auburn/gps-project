%% ROS MESSAGE PARSER

function [data]= parseRosMsgs(msgStruct,type,startSec,startNsec) 
% STANDARDS
% covariances are formatted as matrices
% vectors are horizontal and stacked chronologically
% scalars and strings are stacked chronologically

    switch type
        case 'sensor_msgs/FluidPressure' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).pres = msg.FluidPressure_;
                data(i).presCov = msg.Variance;
            end
            %data = struct2table(data);
            
        case 'sensor_msgs/Imu' 
            for i=1:length(msgStruct)  
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).quaternion_orientation = rosVecToDouble(msg.Orientation);
                data(i).angular_velocity = rosVecToDouble(msg.AngularVelocity);
                data(i).linear_acceleration = rosVecToDouble(msg.LinearAcceleration);

                data(i).orientation_covariance = reshape(msg.OrientationCovariance,3,3);
                data(i).angular_velocity_covariance = reshape(msg.AngularVelocityCovariance,3,3);
                data(i).linear_acceleration_covariance = reshape(msg.LinearAccelerationCovariance,3,3);
                
            end
            %data = struct2table(data);
            
        case 'sensor_msgs/MagneticField' 
            for i=1:length(msgStruct)
                
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).MagField = rosVecToDouble(msg.MagneticField_);
                data(i).MagFieldCov = reshape(msg.MagneticFieldCovariance,3,3);
            end
            %data = struct2table(data);

        case 'sensor_msgs/NavSatFix' 
            for i = 1:length(msgStruct)
                
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).lla = [msg.Latitude msg.Longitude msg.Altitude]; 
                data(i).posCov = reshape(msg.PositionCovariance,3,3);
                
                data(i).COVARIANCETYPEUNKNOWN = msg.COVARIANCETYPEUNKNOWN;
                data(i).COVARIANCETYPEAPPROXIMATED = msg.COVARIANCETYPEAPPROXIMATED;
                data(i).COVARIANCETYPEDIAGONALKNOWN = msg.COVARIANCETYPEDIAGONALKNOWN;
                data(i).COVARIANCETYPEKNOWN = msg.COVARIANCETYPEKNOWN;
                data(i).PositionCovarianceType = msg.PositionCovarianceType;
            end
            %data = struct2table(data);
            
        case 'sensor_msgs/Temperature' 
            for i=1:length(msgStruct)

                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).temp = msg.Temperature_;
                data(i).tempCov = msg.Variance;
            end
            %data = struct2table(data);
            
        case 'sensor_msgs/JointState' 
            for i=1:length(msgStruct)

                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).name = string(msg.Name');
                data(i).Position = msg.Position';
                data(i).Velocity = msg.Velocity';
                data(i).Effort = msg.Effort';
            end
            %data = struct2table(data);
            
        case 'sensor_msgs/Joy' 
            for i=1:length(msgStruct)

                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).Axes = msg.Axes';
                data(i).Buttons = double(msg.Buttons');
            end
            %data = struct2table(data);
            
        case 'sensor_msgs/PointCloud2' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).Data = {msg.Data'};
                data(i).Height = msg.Height;
                data(i).Width = msg.Width;
                data(i).PointStep = msg.PointStep;
                data(i).RowStep = msg.RowStep;
                
                data(i).IsBigendian = msg.IsBigendian;
                data(i).IsDense = msg.IsDense;
                
            end
            %data = struct2table(data);
            warning('TODO: figure out use/parsing for fields ...')
            
        case 'sensor_msgs/TimeReference' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).TimeRef_sec = msg.TimeRef.Sec;
                data(i).TimeRef_n_sec = msg.TimeRef.Nsec;
                data(i).source = string(msg.Source);
            end
            %data = struct2table(data);

        case 'can_msgs/Frame' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
            
                data(i).Data = msg.Data';
                
                data(i).Id = msg.Id;
                data(i).IsRtr = msg.IsRtr;
                data(i).IsExtended = msg.IsExtended;
                data(i).IsError = msg.IsError;
                data(i).Dlc = msg.Dlc;
            end
            %data = struct2table(data);
            
        case 'dataspeed_ulc_msgs/UlcReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).PedalsEnabled = msg.PedalsEnabled;
                data(i).SteeringEnabled = msg.SteeringEnabled;
                data(i).SpeedPreempted = msg.SpeedPreempted;
                data(i).SteeringPreempted = msg.SteeringPreempted;
                data(i).OverrideLatched = msg.OverrideLatched;
                data(i).Timeout = msg.Timeout;
                data(i).YAWRATEMODE = msg.YAWRATEMODE;
                data(i).CURVATUREMODE = msg.CURVATUREMODE;
                data(i).SteeringMode = msg.SteeringMode;
                data(i).LOOSETRACKINGMODE = msg.LOOSETRACKINGMODE;
                data(i).TIGHTTRACKINGMODE = msg.TIGHTTRACKINGMODE;
                data(i).TrackingMode = msg.TrackingMode;
                data(i).SpeedRef = msg.SpeedRef;
                data(i).SpeedMeas = msg.SpeedMeas;
                data(i).AccelRef = msg.AccelRef;
                data(i).AccelMeas = msg.AccelMeas;
                data(i).MaxSteeringAngle = msg.MaxSteeringAngle;
                data(i).MaxSteeringVel = msg.MaxSteeringVel;
            end
            %data = struct2table(data);
            
        case 'dbw_mkz_msgs/BrakeInfoReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).BrakeTorqueRequest = msg.BrakeTorqueRequest;
                data(i).BrakeTorqueActual = msg.BrakeTorqueActual;
                data(i).WheelTorqueActual = msg.WheelTorqueActual;
                data(i).AccelOverGround = msg.AccelOverGround;
                
                % brake pedal quality factor not parsed
                % hill start assist not parsed
                
                data(i).AbsActive = msg.AbsActive;
                data(i).AbsEnabled = msg.AbsEnabled;
                data(i).StabActive = msg.StabActive;
                data(i).StabEnabled = msg.StabEnabled;
                data(i).TracActive = msg.TracActive;
                data(i).TracEnabled = msg.TracEnabled;
                
                data(i).ParkingBrakeStatus = msg.ParkingBrake.Status;
                % rest of parking brake not parsed
                
                data(i).Stationary = msg.Stationary;
            end
            %data = struct2table(data);
            
        case 'dbw_mkz_msgs/BrakeReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).PedalInput = msg.PedalInput;
                data(i).PedalCmd = msg.PedalCmd;
                data(i).PedalOutput = msg.PedalOutput;
                data(i).TorqueInput = msg.TorqueInput;
                data(i).TorqueCmd = msg.TorqueCmd;
                data(i).TorqueOutput = msg.TorqueOutput;
                data(i).DecelCmd = msg.DecelCmd;
                data(i).DecelOutput = msg.DecelOutput;
                
                data(i).BooInput = msg.BooInput;
                data(i).BooCmd = msg.BooCmd;
                data(i).BooOutput = msg.BooOutput;
                data(i).Enabled = msg.Enabled;
                data(i).Override = msg.Override;
                data(i).Driver = msg.Driver;
                data(i).Timeout = msg.Timeout;
                
                data(i).OTHERBRAKE_counter = msg.WatchdogCounter.OTHERBRAKE;
                data(i).OTHERTHROTTLE_counter = msg.WatchdogCounter.OTHERTHROTTLE;
                data(i).OTHERSTEERING_counter = msg.WatchdogCounter.OTHERSTEERING;
                data(i).BRAKECOUNTER_counter = msg.WatchdogCounter.BRAKECOUNTER;
                data(i).BRAKEDISABLED_counter = msg.WatchdogCounter.BRAKEDISABLED;
                data(i).BRAKECOMMAND_counter = msg.WatchdogCounter.BRAKECOMMAND;
                data(i).BRAKEREPORT_counter = msg.WatchdogCounter.BRAKEREPORT;
                data(i).THROTTLECOUNTER_counter = msg.WatchdogCounter.THROTTLECOUNTER;
                data(i).THROTTLEDISABLED_counter = msg.WatchdogCounter.THROTTLEDISABLED;
                data(i).THROTTLECOMMAND_counter = msg.WatchdogCounter.THROTTLECOMMAND;
                data(i).THROTTLEREPORT_counter = msg.WatchdogCounter.THROTTLEREPORT;
                data(i).STEERINGCOUNTER_counter = msg.WatchdogCounter.STEERINGCOUNTER;
                data(i).STEERINGDISABLED_counter = msg.WatchdogCounter.STEERINGDISABLED;
                data(i).STEERINGCOMMAND_counter = msg.WatchdogCounter.STEERINGCOMMAND;
                data(i).STEERINGREPORT_counter = msg.WatchdogCounter.STEERINGREPORT;
                
                data(i).WatchdogBraking = msg.WatchdogBraking;
                data(i).FaultWdc = msg.FaultWdc;
                data(i).FaultCh1 = msg.FaultCh1;
                data(i).FaultCh2 = msg.FaultCh2;
                data(i).FaultPower = msg.FaultPower;
            end
            %data = struct2table(data);
            
        case 'dbw_mkz_msgs/DriverAssistReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                
                
                data(i).Decel = msg.Decel;
                data(i).DecelSrc = msg.DecelSrc;
                data(i).DECELNONE = msg.DECELNONE;
                data(i).DECELAEB = msg.DECELAEB;
                data(i).DECELACC = msg.DECELACC;
                
                data(i).FcwEnabled = msg.FcwEnabled;
                data(i).FcwActive = msg.FcwActive;
                data(i).AebEnabled = msg.AebEnabled;
                data(i).AebPrecharge = msg.AebPrecharge;
                data(i).AebBraking = msg.AebBraking;
                data(i).AccEnabled = msg.AccEnabled;
                data(i).AccBraking = msg.AccBraking;
            end
            %data = struct2table(data);
            
        case 'dbw_mkz_msgs/FuelLevelReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                
                
                data(i).FuelLevel = msg.FuelLevel;
                data(i).Battery12v = msg.Battery12v;
                data(i).BatteryHev = msg.BatteryHev;
                data(i).Odometer = msg.Odometer;
            end
            %data = struct2table(data);
            
        case 'dbw_mkz_msgs/GearReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                % 'dbw_mkz_msgs/Gear'
                data(i).Gear_state = msg.State.Gear_;
                data(i).NONE_state = msg.State.NONE;
                data(i).PARK_state = msg.State.PARK;
                data(i).REVERSE_state = msg.State.REVERSE;
                data(i).NEUTRAL_state = msg.State.NEUTRAL;
                data(i).DRIVE_state = msg.State.DRIVE;
                data(i).LOW_state = msg.State.LOW;
                
                % 'dbw_mkz_msgs/Gear'
                data(i).Gear_cmd = msg.Cmd.Gear_;
                data(i).NONE_cmd = msg.Cmd.NONE;
                data(i).PARK_cmd = msg.Cmd.PARK;
                data(i).REVERSE_cmd = msg.Cmd.REVERSE;
                data(i).NEUTRAL_cmd = msg.Cmd.NEUTRAL;
                data(i).DRIVE_cmd = msg.Cmd.DRIVE;
                data(i).LOW_cmd = msg.Cmd.LOW;
                
                % 'dbw_mkz_msgs/GearReject'
                data(i).Value_reject = msg.Reject.Value;
                data(i).NONE_reject = msg.Reject.NONE;
                data(i).SHIFTINPROGRESS_reject = msg.Reject.SHIFTINPROGRESS;
                data(i).OVERRIDE_reject = msg.Reject.OVERRIDE;
                data(i).ROTARYLOW_reject = msg.Reject.ROTARYLOW;
                data(i).ROTARYPARK_reject = msg.Reject.ROTARYPARK;
                data(i).VEHICLE_reject = msg.Reject.VEHICLE;
                data(i).UNSUPPORTED_reject = msg.Reject.UNSUPPORTED;
                data(i).FAULT_reject = msg.Reject.FAULT;
                
                data(i).Override = msg.Override;
                data(i).FaultBus = msg.FaultBus;
            end
            %data = struct2table(data);
            
        case 'dbw_mkz_msgs/Misc1Report' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
            end
            %data = struct2table(data);
            warning('TODO: decide importnant info. finish parser...')
            % Parser Incomplete. Contains Miscellaneous information regarding media/doors/airbags/bluetooth/lights/wipers/etc. Does contain Outside temperature reading... ');
            
        case 'dbw_mkz_msgs/SteeringReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).SteeringWheelAngle = msg.SteeringWheelAngle;
                data(i).SteeringWheelCmd = msg.SteeringWheelCmd;
                data(i).SteeringWheelTorque = msg.SteeringWheelTorque;
                data(i).SteeringWheelCmdType = msg.SteeringWheelCmdType;
                
                data(i).CMDANGLE = msg.CMDANGLE;
                data(i).CMDTORQUE = msg.CMDTORQUE;
                data(i).Speed = msg.Speed;
                
                data(i).Enabled = msg.Enabled;
                data(i).Override = msg.Override;
                data(i).Timeout = msg.Timeout;
                data(i).FaultWdc = msg.FaultWdc;
                data(i).FaultBus1 = msg.FaultBus1;
                data(i).FaultBus2 = msg.FaultBus2;
                data(i).FaultCalibration = msg.FaultCalibration;
                data(i).FaultPower = msg.FaultPower;
            end
            %data = struct2table(data);
            
        case 'dbw_mkz_msgs/SurroundReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
            end
            %data = struct2table(data);
            warning('TODO: parse radar info')
%             fprintf(' NOTE: Parser Incomplete. Contains Sonar information. To determine how to best parse... ')
            
        case 'dbw_mkz_msgs/ThrottleInfoReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).ThrottlePc = msg.ThrottlePc;
                data(i).ThrottleRate = msg.ThrottleRate;
                data(i).EngineRpm = msg.EngineRpm;
                
                % throttle pedal quality factor not parsed
                % GearNum not parsed
            end
            %data = struct2table(data);
            
        case 'dbw_mkz_msgs/ThrottleReport' 
              for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).PedalInput = msg.PedalInput;
                data(i).PedalCmd = msg.PedalCmd;
                data(i).PedalOutput = msg.PedalOutput;
                
                data(i).Enabled = msg.Enabled;
                data(i).Override = msg.Override;
                data(i).Driver = msg.Driver;
                data(i).Timeout = msg.Timeout;
                
                data(i).OTHERBRAKE_counter = msg.WatchdogCounter.OTHERBRAKE;
                data(i).OTHERTHROTTLE_counter = msg.WatchdogCounter.OTHERTHROTTLE;
                data(i).OTHERSTEERING_counter = msg.WatchdogCounter.OTHERSTEERING;
                data(i).BRAKECOUNTER_counter = msg.WatchdogCounter.BRAKECOUNTER;
                data(i).BRAKEDISABLED_counter = msg.WatchdogCounter.BRAKEDISABLED;
                data(i).BRAKECOMMAND_counter = msg.WatchdogCounter.BRAKECOMMAND;
                data(i).BRAKEREPORT_counter = msg.WatchdogCounter.BRAKEREPORT;
                data(i).THROTTLECOUNTER_counter = msg.WatchdogCounter.THROTTLECOUNTER;
                data(i).THROTTLEDISABLED_counter = msg.WatchdogCounter.THROTTLEDISABLED;
                data(i).THROTTLECOMMAND_counter = msg.WatchdogCounter.THROTTLECOMMAND;
                data(i).THROTTLEREPORT_counter = msg.WatchdogCounter.THROTTLEREPORT;
                data(i).STEERINGCOUNTER_counter = msg.WatchdogCounter.STEERINGCOUNTER;
                data(i).STEERINGDISABLED_counter = msg.WatchdogCounter.STEERINGDISABLED;
                data(i).STEERINGCOMMAND_counter = msg.WatchdogCounter.STEERINGCOMMAND;
                data(i).STEERINGREPORT_counter = msg.WatchdogCounter.STEERINGREPORT;
                
                data(i).FaultWdc = msg.FaultWdc;
                data(i).FaultCh1 = msg.FaultCh1;
                data(i).FaultCh2 = msg.FaultCh2;
                msg.FaultPower = msg.FaultPower;
              end
              %data = struct2table(data);
            
        case 'dbw_mkz_msgs/TirePressureReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).FrontLeft = msg.FrontLeft;
                data(i).FrontRight = msg.FrontRight;
                data(i).RearLeft = msg.RearLeft;
                data(i).RearRight = msg.RearRight;
            end
            %data = struct2table(data);
            
        case 'dbw_mkz_msgs/WheelPositionReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).FrontLeft = msg.FrontLeft;
                data(i).FrontRight = msg.FrontRight;
                data(i).RearLeft = msg.RearLeft;
                data(i).RearRight = msg.RearRight;
                data(i).COUNTSPERREV = msg.COUNTSPERREV;
            end
            %data = struct2table(data);
            
        case 'dbw_mkz_msgs/WheelSpeedReport' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).FrontLeft = msg.FrontLeft;
                data(i).FrontRight = msg.FrontRight;
                data(i).RearLeft = msg.RearLeft;
                data(i).RearRight = msg.RearRight;
            end
            %data = struct2table(data);
            
        case 'diagnostic_msgs/DiagnosticArray' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
%                 
%                 data.OK(i,:) = msg.Status.OK;
%                 data.WARN(i,:) = msg.Status.WARN;
%                 data.ERROR(i,:) = msg.Status.ERROR;
%                 data.STALE(i,:) = msg.Status.STALE;
%                 data.Level(i,:) = msg.Status.Level;
%                 data.Name(i,:) = string(msg.Status.Name);
%                 data.Message(i,:) = string(msg.Status.Message);
%                 data.HardwareId(i,:) = string(msg.Status.HardwareId);
%                 
%                 data.Values(i,:) = {msg.Status.Values};
%                 
            end
            %data = struct2table(data);
            % warning('TODO: fix broken parser')
            
        case 'etalin/PAR_sol_stat' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).PosSolStatus = msg.PosSolStatus;
                data(i).AttBdySolStatus = msg.AttBdySolStatus;
                data(i).AttUsrSolStatus = msg.AttUsrSolStatus;
                data(i).AzimuthEstimatedError = msg.AzimuthEstimatedError;
                data(i).INVALID = msg.INVALID;
                data(i).DEGRADEDPERFORMANCE = msg.DEGRADEDPERFORMANCE;
                data(i).FULLPERFORMANCE = msg.FULLPERFORMANCE;
            end
            %data = struct2table(data);
            
        case 'etalin/SOS' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).INURDY = msg.INURDY;
                data(i).GPSRDY = msg.GPSRDY;
                data(i).EXTAIDRDY = msg.EXTAIDRDY;
                data(i).OBITRDY = msg.OBITRDY;
                data(i).ALRTPRS = msg.ALRTPRS;
                data(i).GPSAIDON = msg.GPSAIDON;
                data(i).VMSAIDON = msg.VMSAIDON;
                data(i).ZUPTAIDON = msg.ZUPTAIDON;
                data(i).POSUPDTRQST = msg.POSUPDTRQST;
                data(i).RDYTOMOVE = msg.RDYTOMOVE;
            end
            %data = struct2table(data);
            
        case 'etalin/acc_bdy_frm' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).timeAcc = msg.TimeAcc;
                data(i).accel = rosVecToDouble(msg.Acceleration);
                data(i).accErr = msg.AccErr;
            end
            %data = struct2table(data);
            
        case 'etalin/ang_acc_sol' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                
                
                data(i).timeAngAcc = msg.TimeAngAcc;
                data(i).ang_accel = rosVecToDouble(msg.AngularAcceleration);
            end
            %data = struct2table(data);
            
        case 'etalin/ang_vel_sol' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).timeAngVel = msg.TimeAngVel;
                data(i).ang_vel = rosVecToDouble(msg.AngularVelocity);
            end
            %data = struct2table(data);
            
        case 'etalin/att_sol' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).bodyAttitude = rosVecToDouble(msg.BodyAttitude);
                data(i).bodyAttErr = rosVecToDouble(msg.BodyAttErr);
                data(i).userAttitude = rosVecToDouble(msg.UserAttitude);
            end
            %data = struct2table(data);
            
        case 'etalin/vel_bdy_frm' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).TimeVel = msg.TimeVel;
                data(i).velocity = rosVecToDouble(msg.Velocity);
                data(i).velErr = msg.VelErr;
            end
            %data = struct2table(data);
            
        case 'geometry_msgs/TwistStamped'
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).linear = rosVecToDouble(msg.Twist.Linear); 
                data(i).angular = rosVecToDouble(msg.Twist.Angular); 
            end
            %data = struct2table(data);
            
        case 'nav_msgs/Odometry' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).position = rosVecToDouble(msg.Pose.Pose.Position);
                data(i).orientation_quat = rosVecToDouble(msg.Pose.Pose.Orientation);

                data(i).pose_covariance = reshape(msg.Pose.Covariance,6,6);

                data(i).linear_twist = rosVecToDouble(msg.Twist.Twist.Linear);
                data(i).angular_twist = rosVecToDouble(msg.Twist.Twist.Angular);

                data(i).twist_covariance = reshape(msg.Twist.Covariance,6,6);
            end
            %data = struct2table(data);
            
        case 'ros_sensor_msgs/AtmosphericCorrectionTagged'
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                [data(i).prn,ind] = sort(extractfield(msg.Corrections,'Prn'));
                [~,ind] = sort(ind);
                data(i).ionoL1_correction(ind) = extractfield(msg.Corrections,'IonosphereCorrectionL1');
                data(i).ionoL2_correction(ind) = extractfield(msg.Corrections,'IonosphereCorrectionL2');
                data(i).tropo_correction(ind) = extractfield(msg.Corrections,'TroposphereCorrection');
            end
            %data = struct2table(data);
            
        case 'ros_sensor_msgs/GpsEphemerisTagged' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).prn = msg.Ephemeris.Prn;
                data(i).subframe1 = msg.Ephemeris.Subframe1;
                data(i).subframe2 = msg.Ephemeris.Subframe2;
                data(i).subframe3 = msg.Ephemeris.Subframe3;
                
            end
            %data = struct2table(data);
            
        case 'ros_sensor_msgs/GpsTimeTagged' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).GpsWeek = msg.GpsTime.GpsWeek;
                data(i).GpsSeconds = msg.GpsTime.GpsSeconds;
            end
            %data = struct2table(data);
            
        case 'ros_sensor_msgs/LlhPositionTagged' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).lla = [msg.LlhPosition.Latitude msg.LlhPosition.Longitude msg.LlhPosition.Altitude];
                data(i).horzAccuracy = double(msg.LlhPosition.HorizontalAccuracy);
                data(i).vertAccuracy = double(msg.LlhPosition.VerticalAccuracy);
            
            end
            %data = struct2table(data);
            
        case 'ros_sensor_msgs/RawMeasurementsTagged' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).prn = extractfield(msg.Measurements,'Prn');
                data(i).pseudorange = extractfield(msg.Measurements,'Pseudorange');
                data(i).pseudorange_variance = extractfield(msg.Measurements,'PseudorangeVariance');
                
                for j=1:length(data(i).prn)
                    fields = fieldnames(msg.Measurements(j).Frequency);
                    type = msg.Measurements(j).Frequency.Type;
                
                    data(i).frequencyType(j) = string(fields{type + 2});
                end
                
                data(i).carrier_phase = extractfield(msg.Measurements,'CarrierPhase');
                data(i).carrier_phase_variance = extractfield(msg.Measurements,'CarrierPhaseVariance');
                data(i).doppler = extractfield(msg.Measurements,'Doppler');
                data(i).doppler_variance = extractfield(msg.Measurements,'DopplerVariance');
                data(i).cn0 = extractfield(msg.Measurements,'CarrierToNoise');
                data(i).loss_of_lock = cell2mat(extractfield(msg.Measurements,'LossOfLock'));
                
            end
            %data = struct2table(data);
            
        case 'ros_sensor_msgs/SvStateTagged'
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).prn = msg.SvState.Prn;
                data(i).svPos = rosVecToDouble(msg.SvState.Position);
                data(i).svVel = rosVecToDouble(msg.SvState.Velocity.Linear);
                data(i).svAngVel = rosVecToDouble(msg.SvState.Velocity.Angular);
                data(i).svClockCorr = msg.SvState.SvClockCorrection;
            end
            %data = struct2table(data);
            
        case 'rosgraph_msgs/Log'
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};

                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
            end
%             %data = struct2table(data);
            data =[];
            % thing for subscribing to rosout. who cares?
            
        case 'septentrio/AttitudeCovEulerMsg'
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).GpsWeek = msg.WeekNumber;
                data(i).GpsSeconds = msg.GPSMs.*10^-3;

                data(i).attCov = diag([msg.VarRoll msg.VarPitch msg.VarHeading ]);
            end
            %data = struct2table(data);
            
        case 'septentrio/AttitudeEulerMsg'
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).GpsWeek = msg.WeekNumber;
                data(i).GpsSeconds = msg.GPSMs.*10^-3;
                
                data(i).numSat = msg.NumSat;
                data(i).rpy = double([msg.Roll msg.Pitch msg.Heading]);
                data(i).Omega = double([msg.XOmega msg.YOmega msg.ZOmega]);
            end
            %data = struct2table(data);
            
        case 'septentrio/PosCovCartesianMsg'
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
               
                data(i).GpsWeek = msg.WeekNumber;
                data(i).GpsSeconds = msg.GPSMs.*10^-3;

                data(i).posCov = [msg.CovXx msg.CovXy msg.CovXz msg.CovXb;...
                                  msg.CovXy msg.CovYy msg.CovYz msg.CovYb;...
                                  msg.CovXz msg.CovYz msg.CovZz msg.CovZb;...
                                  msg.CovXb msg.CovYb msg.CovZb msg.CovBb];
                    
            end
            %data = struct2table(data);
            
        case 'septentrio/PvtCartesianMsg'
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).GpsWeek = msg.WeekNumber;
                data(i).GpsSeconds = msg.GPSMs.*10^-3;

                data(i).numSat = msg.NumSat;

                data(i).position = [msg.XPosition msg.YPosition msg.ZPosition];
                data(i).velocity = [msg.XVelocity msg.YVelocity msg.ZVelocity];
                data(i).clockBias = msg.RxClkBias;
                data(i).clockDrift = msg.RxClkDrift;
                data(i).course = msg.Course;
            end
            %data = struct2table(data);
            
        case 'septentrio/VelCovCartesianMsg'
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Header.Stamp.Sec) - startSec) + (double(msg.Header.Stamp.Nsec) - startNsec)*10^-9;
                
                data(i).GpsWeek = msg.WeekNumber;
                data(i).GpsSeconds = msg.GPSMs.*10^-3;

                data(i).velCov = [msg.CovVxVx msg.CovVxVy msg.CovVxVz msg.CovVxd;...
                                  msg.CovVxVy msg.CovVyVy msg.CovVyVz msg.CovVyd;...
                                  msg.CovVxVz msg.CovVyVz msg.CovVzVz msg.CovVzd;...
                                  msg.CovVxd  msg.CovVyd  msg.CovVzd  msg.CovDd];
            end
            %data = struct2table(data);
            
        case 'std_msgs/Bool' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).bool = msg.Data;
            end
            %data = struct2table(data);
            
        case 'std_msgs/String' 
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).string = msg.Data;
            end
            %data = struct2table(data);
            
        case 'tf2_msgs/TFMessage'
            for i=1:length(msgStruct)
                msg = msgStruct{i,1};
                data(i).time = (double(msg.Transforms(1).Header.Stamp.Sec) - startSec) + (double(msg.Transforms(1).Header.Stamp.Nsec) - startNsec)*10^-9;
                data(i).seq = msg.Transforms(1).Header.Seq;
            end
%             %data = struct2table(data);
            data = [];
            % fuck this
            
        otherwise 
            warning(strcat('Unrecognized Message Type: ',type))
            data = NaN;
    end
            
end

%% SUPPORTING FUNCTIONS 

function out = rosVecToDouble(in) 
    
    fields = fieldnames(in);
    
    out = [];
    for i = 2:length(fields)
        out = [out in.(fields{i})];
    end
end 
