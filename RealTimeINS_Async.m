clear all;
pause on;

file_data = readmatrix('navsys_1hr.csv');

Accelerometer = file_data(:,1:3);
Gyroscope = file_data(:,4:6);
Magnetometer = file_data(:,7:9);
GPSPosition = file_data(:,10:12);
GPSVelocity = file_data(:,13:15);
sample = file_data(:,16);

refloc = [23.67108327593469 86.96900856134305 11.4831720000000];
fusionfilt = insfilterAsync('ReferenceLocation', refloc);

[XYZ, H, D, I, F] = wrldmagm(refloc(3), refloc(1), refloc(2), decyear(2020,7,4),'2020');
mag_field = XYZ/1000;

fusionfilt.State(1:4) = [1 0 0 0];
fusionfilt.State(23:25) = mag_field;
fusionfilt.State(20:22) = deg2rad([1.125 1.125 1.125]); 
fusionfilt.StateCovariance = diag(1e-4 * ones(28,1));

%--------------------Noise insfilterAsync---------------------------
rmag = sqrt(mag_field(1,1)^2+mag_field(2,1)^2+mag_field(3,1)^2);
mn_t = tunernoise('insfilterAsync');
   
mn_t.AccelerometerNoise = 0.0061;
mn_t.GyroscopeNoise = 3.0462e-6;
mn_t.MagnetometerNoise = rmag;
mn_t.GPSPositionNoise = 1.4^2;
mn_t.GPSVelocityNoise = 0.01^2;

% Process noises
fusionfilt.AngularVelocityNoise = 0.05;     
fusionfilt.AccelerationNoise = 50.2;
fusionfilt.VelocityNoise = 1.0e-6;
fusionfilt.PositionNoise = 1.0e-4;
fusionfilt.QuaternionNoise = 1.0e-2; 
fusionfilt.MagnetometerBiasNoise = 1.5e-2;    %const, tracking the course well
fusionfilt.AccelerometerBiasNoise = 1.5e-10; 
fusionfilt.GyroscopeBiasNoise = 1.5e-14;                
fusionfilt.GeomagneticVectorNoise = 1.0e-6;

%-----------------------------------AHRS filt------------------------------
GyroscopeNoiseMPU9250 = 3.0462e-06; %GyroscopeNoise (variance value) in units of rad/s
AccelerometerNoiseMPU9250 = 0.0061; %AccelerometerNoise(variance value)in units of m/s^2

fuse = ahrsfilter('SampleRate',100,'GyroscopeNoise',GyroscopeNoiseMPU9250,'AccelerometerNoise',AccelerometerNoiseMPU9250,'MagnetometerNoise',0.8);

Orientation = fuse(Accelerometer,Gyroscope,Magnetometer);
ahrs_eul  = euler(quaternion(Orientation),"YXZ","point");
ahrs_eul = rad2deg(ahrs_eul);
Position = GPSPosition;
sensorData = timetable(Accelerometer,Gyroscope,Magnetometer,GPSPosition,GPSVelocity,'SampleRate',100);
groundTruth = timetable(Position,Orientation,'SampleRate',100);
plot_data(sensorData.Time, ahrs_eul,1,'AHRS Eular');

%lin_acc = linaccel(Orientation,Accelerometer);
%plot_data(sensorData.Time, lin_acc,11,'lin accel');

%-----------------------------------------insfilterAsync----------------------------------------%
[posTunedEst,est_vel, orientTunedEst, eul_untune, accl, angR, accl_ino, gyro_ino, mag_ino] = fuse_t(fusionfilt,sensorData,mn_t,sample);
eul_untune = rad2deg(eul_untune);

plot_data(sensorData.Time, eul_untune,2,'Estimated Eular');
plot_data(sensorData.Time, accl,3,'Accel NED');
%plot_data(sensorData.Time, est_vel,4,'Vel NED');
%plot_data(sensorData.Time, posTunedEst,5,'Pos NED');

%plot_data(sensorData.Time, accl_ino,3,'Accel innovation');
%plot_data(sensorData.Time, gyro_ino,4,'Gyro innovation');
%plot_data(sensorData.Time, mag_ino,5,'Mag innovation');


