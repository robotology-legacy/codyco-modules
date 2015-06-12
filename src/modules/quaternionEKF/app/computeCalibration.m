load('./leg0DegreesData.txt');
load('./leg90DegreesData.txt');

sensor_yPose1 = mean(leg0DegreesData)';
sensor_yPose2 = mean(leg90DegreesData)';
 
sensor_zhat_lsole = -sensor_yPose1 ./ (norm(sensor_yPose1));
sensor_xhat_lsole = -sensor_yPose2 ./ (norm(sensor_yPose2));

sensor_yhat_lsole = cross(sensor_zhat_lsole,sensor_xhat_lsole);

sensor_R_lsole = [sensor_xhat_lsole sensor_yhat_lsole sensor_zhat_lsole ];
lsole_R_sensor = sensor_R_lsole';
%lsole_q_sensor = dcm2q(lsole_R_sensor)

disp('Desired Quaternion:');
lsole_q_sensor = dcm2quat(lsole_R_sensor)


% raw reading in sensor frame

lsole_yPose1 = lsole_R_sensor * sensor_yPose1
lsole_yPose2 = lsole_R_sensor * sensor_yPose2


theta_pose1 = computeAngleBetweenVectors(lsole_yPose1./norm(lsole_yPose1), [0;0;-1]) * (180/pi)
theta_pose2 = computeAngleBetweenVectors(lsole_yPose2./norm(lsole_yPose2), [0;0;-1]) * (180/pi)


lsole_normyPose1 = lsole_yPose1 ./ norm(lsole_yPose1);
lsole_normyPose2 = lsole_yPose2 ./ norm(lsole_yPose2);

v1 = [0;0;-1];
v2_pose1 = [0;lsole_normyPose1(2:3)]; 
v2_pose2 = [0;lsole_normyPose2(2:3)]; 
rollTheta_pose1 = atan2(norm(cross(v1,v2_pose1)),dot(v1,v2_pose1))
rollTheta_pose2 = atan2(norm(cross(v1,v2_pose2)),dot(v1,v2_pose2))*pi/180


disp('desired rotation');
lsole_R_sensor