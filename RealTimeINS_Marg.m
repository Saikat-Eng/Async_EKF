clear all;
close all;
pause on;

file_data = readmatrix('navsys_1hr.csv');

Accelerometer = file_data(:,1:3);
Gyroscope = file_data(:,4:6);
Magnetometer = file_data(:,7:9);
GPSPosition = file_data(:,10:12);
GPSVelocity = file_data(:,13:15);
sample = file_data(:,16);


refloc = [23.67108327593469 86.96900856134305 11.4831720000000];
fusionfilt = insfilterMARG;
fusionfilt.IMUSampleRate = 100;
fusionfilt.ReferenceLocation = refloc;


[XYZ, H, D, I, F] = wrldmagm(refloc(3), refloc(1), refloc(2), decyear(2020,7,4),'2020');
mag_field = XYZ/1000;

fusionfilt.State(1:4) = [1 0 0 0];
fusionfilt.State(17:19) = mag_field;
fusionfilt.StateCovariance = diag(1e-4 * ones(22,1));


%--------------------Noise insfilterMarg---------------------------

mn_t = tunernoise('insfilterMARG');

mn_t.MagnetometerNoise = 0.0225;
mn_t.GPSPositionNoise = 5.169;
mn_t.GPSVelocityNoise = 0.0051;

%Rmag = 0.0862; % Magnetometer measurement noise
%Rvel = 0.0051; % GPS Velocity measurement noise
%Rpos = 5.169;

% Process noises
fusionfilt.AccelerometerBiasNoise =  0.0000010716; 
fusionfilt.AccelerometerNoise = 0.0061; 
fusionfilt.GyroscopeBiasNoise = 1.3436e-14; 
fusionfilt.GyroscopeNoise =  3.0462e-6; 
fusionfilt.MagnetometerBiasNoise = 2.189e-11;
fusionfilt.GeomagneticVectorNoise = 7.67e-6;

%-----------------------------------AHRS filt------------------------------
GyroscopeNoiseMPU9250 = 3.0462e-06; % GyroscopeNoise (variance value) in units of rad/s
AccelerometerNoiseMPU9250 = 0.0061; % AccelerometerNoise(variance value)in units of m/s^2

fuse = ahrsfilter('SampleRate',100,'GyroscopeNoise',GyroscopeNoiseMPU9250,'AccelerometerNoise',AccelerometerNoiseMPU9250);

Orientation = fuse(Accelerometer,Gyroscope,Magnetometer);
ahrs_eul  = euler(quaternion(Orientation),"YXZ","frame");
ahrs_eul = rad2deg(ahrs_eul);
Position = GPSPosition;
sensorData = timetable(Accelerometer,Gyroscope,Magnetometer,GPSPosition,GPSVelocity,'SampleRate',100);
%groundTruth = timetable(Position,Orientation,'SampleRate',100);
plot_data(sensorData.Time, ahrs_eul,1,'AHRS Eular');

%-----------------------------------------insfilterMarg----------------------------------------%
        %pos, orient, eul
        %       State                           Units        Index
        %   Orientation (quaternion parts)                   S(1:4)
        %   Position (NAV)                      m            S(5:7)
        %   Velocity (NAV)                      m/s          S(8:10)
        %   Delta Angle Bias (XYZ)              rad/s        S(11:13)
        %   Delta Velocity Bias (XYZ)           m/s          S(14:16)
        %   Geomagnetic Field Vector (NAV)      uT           S(17:19)
        %   Magnetometer Bias (XYZ)             uT           S(20:22)

[posTunedEst, orientTunedEst, eul_untune] = fuse_m(fusionfilt,sensorData,mn_t,sample);
eul_untune = rad2deg(eul_untune);
plot_data(sensorData.Time, eul_untune, 2, 'Estimated Eular');

%plot_data(sensorData.Time, accl,3,'Accel NED');
%plot_data(sensorData.Time, est_vel,4,'Vel NED');

%plot_data(sensorData.Time, accl_ino,3,'Accel innovation');
%plot_data(sensorData.Time, gyro_ino,4,'Gyro innovation');
%plot_data(sensorData.Time, mag_ino,5,'Mag innovation');




