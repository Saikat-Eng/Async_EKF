function [pos, vel, orient, eul, accl, angR, accl_ino, gyro_ino, mag_ino] = fuse_t(filt, sensorData, measNoise, sample)
%dt = seconds(diff(sensorData.Time));
N = size(sensorData,1);
orient = quaternion.zeros(N,1);
pos = zeros(N,3);
vel = zeros(N,3);
eul = zeros(N,3);
accl = zeros(N,3); 
angR = zeros(N,3); 
accl_ino = zeros(N,3); 
gyro_ino = zeros(N,3); 
mag_ino = zeros(N,3); 

%    Orientation (quaternion parts)             1:4  
%    Angular Velocity (XYZ)            rad/s    5:7  
%    Position (NED)                    m        8:10 
%    Velocity (NED)                    m/s      11:13
%    Acceleration (NED)                m/s^2    14:16
%    Accelerometer Bias (XYZ)          m/s^2    17:19
%    Gyroscope Bias (XYZ)              rad/s    20:22
%    Geomagnetic Field Vector (NED)    uT       23:25
%    Magnetometer Bias (XYZ)           uT       26:28

for ii=1:N

    predict(filt, 0.01);
    
    [pos(ii,:), orient(ii,:)] = pose(filt);
    eul(ii,:) = euler(quaternion(orient(ii,:)),"YXZ","point");
    accl(ii,:) = filt.State(14:16);
    angR(ii,:) = filt.State(5:7);
    vel(ii,:) = filt.State(11:13);

    %if(eul(ii,3)<0)
    %    eul(ii,3) = eul(ii,3)+6.28318530717958647692528676656;
    %end
    
    [accl_ino(ii,:),temp] = fuseaccel(filt, sensorData.Accelerometer(ii,:), measNoise.AccelerometerNoise);
    [gyro_ino(ii,:),temp] = fusegyro(filt, sensorData.Gyroscope(ii,:), measNoise.GyroscopeNoise);

    if(mod(ii,5)==0)
        [mag_ino(ii,:),temp] = fusemag(filt, sensorData.Magnetometer(ii,:), measNoise.MagnetometerNoise);
    end
     
    if sample(ii,:)==1  
        fusegps(filt, sensorData.GPSPosition(ii,:), measNoise.GPSPositionNoise,sensorData.GPSVelocity(ii,:), measNoise.GPSVelocityNoise);
    end
    
    
end

end
