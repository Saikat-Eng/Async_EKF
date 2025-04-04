function [pos, orient, eul] = fuse_m(filt, sensorData, measNoise, sample)

N = size(sensorData,1);
orient = quaternion.zeros(N,1);
pos = zeros(N,3);
eul = zeros(N,3);
filt.IMUSampleRate = 100;

for ii=1:N
        %       State                           Units        Index
        %   Orientation (quaternion parts)                   S(1:4)
        %   Position (NAV)                      m            S(5:7)
        %   Velocity (NAV)                      m/s          S(8:10)
        %   Delta Angle Bias (XYZ)              rad/s        S(11:13)
        %   Delta Velocity Bias (XYZ)           m/s          S(14:16)
        %   Geomagnetic Field Vector (NAV)      uT           S(17:19)
        %   Magnetometer Bias (XYZ)             uT           S(20:22)
        
    
    predict(filt, sensorData.Accelerometer(ii,:), sensorData.Gyroscope(ii,:));

    [pos(ii,:), orient(ii,:)] = pose(filt);
    eul(ii,:) = euler(quaternion(orient(ii,:)),"YXZ","frame");

    if(not(mod(ii,100)==0))
        continue;
    end

    %if sample(ii,:)==1
       % fusegps(filt, sensorData.GPSPosition(ii,:), measNoise.GPSPositionNoise,sensorData.GPSVelocity(ii,:), measNoise.GPSVelocityNoise);
        fusegps(filt, sensorData.GPSPosition(ii,:), measNoise.GPSPositionNoise,sensorData.GPSVelocity(ii,:), measNoise.GPSVelocityNoise);
        fusemag(filt,sensorData.Magnetometer(ii,:), measNoise.MagnetometerNoise);
    %end

    %fusemag(filt, sensorData.Magnetometer(ii,:), measNoise.MagnetometerNoise);


end

end
