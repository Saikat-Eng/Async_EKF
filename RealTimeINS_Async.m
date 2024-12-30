clear all;
pause on;

file_data = readmatrix('ins_data.csv');

Accelerometer = file_data(:,1:3);
Gyroscope = file_data(:,4:6);
Magnetometer = file_data(:,7:9);
GPSPosition = file_data(:,10:12);
GPSVelocity = file_data(:,13:15);
sample = file_data(:,16);

noise_a = std(Accelerometer);
cov_matrix_acc = cov(Accelerometer);
disp(cov_matrix_acc);
disp(noise_a);

noise_g = std(Gyroscope);
cov_matrix_gyr = cov(Gyroscope);
disp(cov_matrix_gyr);
disp(noise_g);

noise_m = std(Magnetometer);
cov_matrix_mag = cov(Magnetometer);
disp(cov_matrix_mag);
disp(noise_m);

cov_matrix_pos = 3.4;
cov_matrix_vel = 0.01;


%refloc = [12.957036	77.718811 1.5];
refloc = [23.6711180000000 86.9690780000000	11.4831720000000];
fusionfilt = insfilterAsync('ReferenceLocation', refloc);
fusionfilt_marg = insfilterMARG('ReferenceLocation', refloc);

[XYZ, H, D, I, F] = wrldmagm(refloc(3), refloc(1), refloc(2), decyear(2020,7,4),'2020');
mag_field = XYZ/1000;


fusionfilt.State(1:4) = [1 0 0 0];
fusionfilt.State(23:25) = [mag_field(1) mag_field(2) mag_field(3)];
fusionfilt.State(20:22) = deg2rad([3.125 3.125 3.125]); 
fusionfilt.StateCovariance = diag(1e-2 * ones(28,1));


fusionfilt_marg.State(1:4) = [1 0 0 0];
fusionfilt_marg.IMUSampleRate = 100;
fusionfilt_marg.State(17:19) = [mag_field(1) mag_field(2) mag_field(3)] ;
fusionfilt_marg.StateCovariance = diag(1e-5 * ones(22,1));

%--------------------Noise insfilterAsync---------------------------
mn_t = tunernoise('insfilterAsync');
mn_t.AccelerometerNoise = cov_matrix_acc;
mn_t.GyroscopeNoise = cov_matrix_gyr;
mn_t.MagnetometerNoise = cov_matrix_mag;
mn_t.GPSPositionNoise = cov_matrix_pos;
mn_t.GPSVelocityNoise = cov_matrix_vel;

% Process noises
fusionfilt.QuaternionNoise = 1.0e-4; 
fusionfilt.AngularVelocityNoise = 0.05;
fusionfilt.AccelerationNoise = 0.0061;      %0.005;
fusionfilt.MagnetometerBiasNoise = 5.0e-2;
fusionfilt.AccelerometerBiasNoise = 5.6e-5;
fusionfilt.GyroscopeBiasNoise = 1.5e-15;
fusionfilt.VelocityNoise = 1.0e-6;
fusionfilt.PositionNoise = 1.0e-6;
fusionfilt.GeomagneticVectorNoise = 1.0e-6;

filtTuned = copy(fusionfilt);

%--------------------Noise insfilterMarg---------------------------

mn_m = tunernoise('insfilterMARG');

mn_m.MagnetometerNoise = 0.862; 
mn_m.GPSVelocityNoise = 0.051;
mn_m.GPSPositionNoise = 5.169; 

% Process noises
fusionfilt_marg.AccelerometerBiasNoise =  1.0e-8; 
fusionfilt_marg.AccelerometerNoise = 0.0061; 
fusionfilt_marg.GyroscopeBiasNoise = 1.3436e-12; 
fusionfilt_marg.GyroscopeNoise =  3.046e-6; 
fusionfilt_marg.MagnetometerBiasNoise = 2.189e-8;
fusionfilt_marg.GeomagneticVectorNoise = 7.67e-12;

%-----------------------------------AHRS filt------------------------------
fuse = ahrsfilter('SampleRate',100);
Orientation = fuse(Accelerometer,Gyroscope,Magnetometer);
ahrs_eul  = quat2eul(Orientation,'XYZ');
ahrs_eul = rad2deg(ahrs_eul);

Position = GPSPosition;
sensorData = timetable(Accelerometer,Gyroscope,Magnetometer,GPSPosition,GPSVelocity,'SampleRate',100);
groundTruth = timetable(Position,Orientation,'SampleRate',100);

plot_data(sensorData.Time, ahrs_eul,1);

[posTunedEst, orientTunedEst, eul_untune] = fuse_t(fusionfilt,sensorData,mn_t,sample);
%[posTunedEst, orientTunedEst, eul_untune] = fuse_m(fusionfilt_marg,sensorData,mn_m,sample);
eul_untune = rad2deg(eul_untune);

plot_data(sensorData.Time, eul_untune,2);

ld = readmatrix('out_heading.csv');
plot_data(sensorData.Time, ld(:,1:3), 3);

%cfg = tunerconfig(class(filtTuned),'MaxIterations',30,'StepForward',1.1);
%tunedmn = tune(filtTuned,mn_t,sensorData,groundTruth,cfg);
%[posTunedEst, orientTunedEst, eul_tune] = fuse_t(filtTuned,sensorData,mn_t,sample);
%eul_tune = rad2deg(eul_tune);

%plot_data(sensorData.Time, eul_tune,3);

