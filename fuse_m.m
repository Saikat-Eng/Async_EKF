function [pos, orient, eul] = fuse_m(filt, sensorData, measNoise, sample)
%dt = seconds(diff(sensorData.Time));
N = size(sensorData,1);
orient = quaternion.zeros(N,1);
pos = zeros(N,3);
eul = zeros(N,3);

for ii=1:N

    if ii ~= 1
        predict(filt, sensorData.Accelerometer(ii,:), sensorData.Gyroscope(ii,:));
    end 

    [pos(ii,:), orient(ii,:)] = pose(filt);
    eul(ii,:) = quat2eul(orient(ii,:),'XYZ');
    
 %   fuseaccel(filt, sensorData.Accelerometer(ii,:), measNoise.AccelerometerNoise);
 %   fusegyro(filt, sensorData.Gyroscope(ii,:), measNoise.GyroscopeNoise);
    
    %fusemag(filt, sensorData.Magnetometer(ii,:), measNoise.MagnetometerNoise);
    fusemag(filt,sensorData.Magnetometer(ii,:), measNoise.MagnetometerNoise);

    if sample(ii,:)==1
       % fusegps(filt, sensorData.GPSPosition(ii,:), measNoise.GPSPositionNoise,sensorData.GPSVelocity(ii,:), measNoise.GPSVelocityNoise);
        fusegps(filt, sensorData.GPSPosition(ii,:), measNoise.GPSPositionNoise,sensorData.GPSVelocity(ii,:), measNoise.GPSVelocityNoise);
    end

end

end