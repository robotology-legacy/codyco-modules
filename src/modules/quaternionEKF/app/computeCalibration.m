load('./calib0deg.txt');
load('./calib90deg.txt');

sensor_yPose1 = mean(calib0deg)';
sensor_yPose2 = mean(calib90deg)';

%load('./leg0DegreesData.txt');
%load('./leg90DegreesData.txt');

%sensor_yPose1 = mean(leg0DegreesData)';
%sensor_yPose2 = mean(leg90DegreesData)';
 
sensor_zhat_lsole = -sensor_yPose1 ./ (norm(sensor_yPose1));
sensor_xhat_lsole_candidate = -sensor_yPose2 ./ (norm(sensor_yPose2));

sensor_xhat_lsole = sensor_xhat_lsole_candidate - dot(sensor_xhat_lsole_candidate,sensor_zhat_lsole).*sensor_zhat_lsole;
sensor_xhat_lsole = sensor_xhat_lsole./norm(sensor_xhat_lsole);

sensor_yhat_lsole = cross(sensor_zhat_lsole,sensor_xhat_lsole) ;
%sensor_yhat_lsole = sensor_yhat_lsole ./ norm(sensor_yhat_lsole);

sensor_R_lsole = [sensor_xhat_lsole sensor_yhat_lsole sensor_zhat_lsole ];

fprintf('Norm of the obtained rotation : %1.4f\n',norm(sensor_R_lsole));

lsole_R_sensor = sensor_R_lsole';
lsole_q_sensor = dcm2q(lsole_R_sensor);
%lsole_q_sensor = dcm2quat(lsole_R_sensor);

%fprintf('Desired Quaternion (dcm2quat):[%1.4f,%1.4f,%1.4f,%1.4f]\n',lsole_q_sensor(1),lsole_q_sensor(2),lsole_q_sensor(3),lsole_q_sensor(4));
fprintf('Desired Quaternion computed (dcm2q):[%1.4f,%1.4f,%1.4f,%1.4f]\n',lsole_q_sensor(1),lsole_q_sensor(2),lsole_q_sensor(3),lsole_q_sensor(4));

% raw reading in sensor frame

lsole_yPose1 = lsole_R_sensor * sensor_yPose1;
lsole_yPose2 = lsole_R_sensor * sensor_yPose2;

fprintf('Raw reading in sensorFrame (pose1) :[%1.4f,%1.4f,%1.4f]\n',lsole_yPose1(1),lsole_yPose1(2),lsole_yPose1(3));
fprintf('Raw reading in sensorFrame (pose2) :[%1.4f,%1.4f,%1.4f]\n',lsole_yPose2(1),lsole_yPose2(2),lsole_yPose2(3));

theta_pose1 = computeAngleBetweenVectors(lsole_yPose1./norm(lsole_yPose1), [0;0;-1]) * (180/pi);
theta_pose2 = computeAngleBetweenVectors(lsole_yPose2./norm(lsole_yPose2), [0;0;-1]) * (180/pi); 


lsole_normyPose1 = lsole_yPose1 ./ norm(lsole_yPose1);
lsole_normyPose2 = lsole_yPose2 ./ norm(lsole_yPose2);

v1 = [0;0;-1];
v2_pose1 = [0;lsole_normyPose1(2:3)]; 
v2_pose2 = [0;lsole_normyPose2(2:3)]; 
rollTheta_pose1 = atan2(norm(cross(v1,v2_pose1)),dot(v1,v2_pose1));
rollTheta_pose2 = atan2(norm(cross(v1,v2_pose2)),dot(v1,v2_pose2))*pi/180;

fprintf('Roll angle (pose1) from calibration data : %1.4f\n',rollTheta_pose1);
fprintf('Roll angle (pose2) from calibration data : %1.4f\n',rollTheta_pose2);

disp('desired rotation');
lsole_R_sensor