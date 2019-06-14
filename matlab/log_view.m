clear
clc

% log_filename = 'log_14_2019-6-3-16-13-58_';
% log_filename = 'log_15_2019-6-3-16-20-22_';
log_filename = 'log_21_2019-6-3-16-44-20_';

log_filename_visual_odometry = [log_filename,'vehicle_visual_odometry_0.csv'];
log_filename_rates_setpoint =  [log_filename,'vehicle_rates_setpoint_0.csv'];
log_filename_rates = [log_filename,'rate_ctrl_status_0.csv'];
log_filename_pos_setpoint = [log_filename,'vehicle_local_position_setpoint_0.csv'];
log_filename_pos = [log_filename,'vehicle_local_position_0.csv'];
log_filename_att_setpoint = [log_filename,'vehicle_attitude_setpoint_0.csv'];
log_filename_att = [log_filename,'vehicle_attitude_0.csv'];
log_filename_rc = [log_filename,'input_rc_0.csv'];
log_filename_cpu = [log_filename,'cpuload_0.csv'];

T_visual_odometry = readtable(log_filename_visual_odometry,'Delimiter',',','ReadVariableNames',true);
T_rates_setpoint= readtable(log_filename_rates_setpoint,'Delimiter',',','ReadVariableNames',true);
T_rates = readtable(log_filename_rates,'Delimiter',',','ReadVariableNames',true);
T_pos_setpoint = readtable(log_filename_pos_setpoint,'Delimiter',',','ReadVariableNames',true);
T_pos = readtable(log_filename_pos,'Delimiter',',','ReadVariableNames',true);
T_att_setpoint = readtable(log_filename_att_setpoint,'Delimiter',',','ReadVariableNames',true);
T_att = readtable(log_filename_att,'Delimiter',',','ReadVariableNames',true);
T_rc = readtable(log_filename_rc,'Delimiter',',','ReadVariableNames',true);
T_cpu = readtable(log_filename_cpu,'Delimiter',',','ReadVariableNames',true);

quat_setpoint = [T_att_setpoint.q_d_0_ T_att_setpoint.q_d_1_ T_att_setpoint.q_d_2_ T_att_setpoint.q_d_3_];
quat = [T_att.q_0_ T_att.q_1_ T_att.q_2_ T_att.q_3_];
quat_vo = [T_visual_odometry.q_0_ T_visual_odometry.q_1_ T_visual_odometry.q_2_ T_visual_odometry.q_3_];

eul_setpoint = quat2eul1(quat_setpoint);
eul = quat2eul1(quat);
eul_vo = quat2eul1(quat_vo);

T_att_eul_setpoint.timestamp = T_att_setpoint.timestamp;
T_att_eul_setpoint.yaw = eul_setpoint(:,1);
T_att_eul_setpoint.pitch = eul_setpoint(:,2);
T_att_eul_setpoint.roll = eul_setpoint(:,3);

T_att_eul.timestamp = T_att.timestamp;
T_att_eul.yaw = eul(:,1);
T_att_eul.pitch = eul(:,2);
T_att_eul.roll = eul(:,3);

T_att_eul_vo.timestamp = T_visual_odometry.timestamp;
T_att_eul_vo.yaw = eul_vo(:,1);
T_att_eul_vo.pitch = eul_vo(:,2);
T_att_eul_vo.roll = eul_vo(:,3);

%%
figure(1);clf;
subplot(3,1,1)
hold on
plot(T_att_eul_setpoint.timestamp * 1e-6,rad2deg(T_att_eul_setpoint.roll),'color',[0.2 0.8 0.2],'linewidth',2)
plot(T_att_eul.timestamp * 1e-6,rad2deg(T_att_eul.roll),'color',[0 0 0],'linewidth',1)
xlabel('sec')
ylabel('deg')
title('roll')

subplot(3,1,2)
hold on
plot(T_att_eul_setpoint.timestamp * 1e-6,rad2deg(T_att_eul_setpoint.pitch),'color',[0.2 0.8 0.2],'linewidth',2)
plot(T_att_eul.timestamp * 1e-6,rad2deg(T_att_eul.pitch),'color',[0 0 0],'linewidth',1)
xlabel('sec')
ylabel('deg')
title('pitch')

subplot(3,1,3)
hold on
plot(T_att_eul_setpoint.timestamp * 1e-6,rad2deg(T_att_eul_setpoint.yaw),'color',[0.2 0.8 0.2],'linewidth',2)
plot(T_att_eul.timestamp * 1e-6,rad2deg(T_att_eul.yaw),'color',[0 0 0],'linewidth',1)
xlabel('sec')
ylabel('deg')
title('yaw')

%%
figure(2);clf;
subplot(3,1,1)
hold on
plot(T_pos_setpoint.timestamp * 1e-6,T_pos_setpoint.x,'color',[0.2 0.8 0.2],'linewidth',2)
plot(T_pos.timestamp * 1e-6,T_pos.x,'color',[0 0 0],'linewidth',1)
xlabel('sec')
ylabel('m')
title('x')

subplot(3,1,2)
hold on
plot(T_pos_setpoint.timestamp * 1e-6,T_pos_setpoint.y,'color',[0.2 0.8 0.2],'linewidth',2)
plot(T_pos.timestamp * 1e-6,T_pos.y,'color',[0 0 0],'linewidth',1)
xlabel('sec')
ylabel('m')
title('y')

subplot(3,1,3)
hold on
plot(T_pos_setpoint.timestamp * 1e-6,T_pos_setpoint.z,'color',[0.2 0.8 0.2],'linewidth',2)
plot(T_pos.timestamp * 1e-6,T_pos.z,'color',[0 0 0],'linewidth',1)
xlabel('sec')
ylabel('m')
title('z')

%%
figure(3);clf;
subplot(3,1,1)
hold on
plot(T_rates_setpoint.timestamp * 1e-6,rad2deg(T_rates_setpoint.roll),'color',[0.2 0.8 0.2],'linewidth',2)
plot(T_rates.timestamp * 1e-6,rad2deg(T_rates.rollspeed),'color',[0 0 0],'linewidth',1)
xlabel('sec')
ylabel('deg/s')
title('body rate x')

subplot(3,1,2)
hold on
plot(T_rates_setpoint.timestamp * 1e-6,rad2deg(T_rates_setpoint.pitch),'color',[0.2 0.8 0.2],'linewidth',2)
plot(T_rates.timestamp * 1e-6,rad2deg(T_rates.pitchspeed),'color',[0 0 0],'linewidth',1)
xlabel('sec')
ylabel('deg/s')
title('body rate y')

subplot(3,1,3)
hold on
plot(T_rates_setpoint.timestamp * 1e-6,rad2deg(T_rates_setpoint.yaw),'color',[0.2 0.8 0.2],'linewidth',2)
plot(T_rates.timestamp * 1e-6,rad2deg(T_rates.yawspeed),'color',[0 0 0],'linewidth',1)
xlabel('sec')
ylabel('deg/s')
title('body rate z')

%%
figure(4);clf;
subplot(5,1,1);
hold on
plot(T_visual_odometry.timestamp * 1e-6,T_visual_odometry.x,'color',[0.2 0.8 0.2],'linewidth',2);
plot(T_pos.timestamp * 1e-6,T_pos.x,'-.b','linewidth',1);
xlabel('sec')
ylabel('m')
title('x compare vo and ekf2')

subplot(5,1,2);
hold on
plot(T_visual_odometry.timestamp * 1e-6,T_visual_odometry.y,'color',[0.2 0.8 0.2],'linewidth',2);
plot(T_pos.timestamp * 1e-6,T_pos.y,'-.b','linewidth',1);
xlabel('sec')
ylabel('m')
title('y compare vo and ekf2')

subplot(5,1,3);
hold on
plot(T_visual_odometry.timestamp * 1e-6,T_visual_odometry.z,'color',[0.2 0.8 0.2],'linewidth',2);
plot(T_pos.timestamp * 1e-6,T_pos.z,'-.b','linewidth',1);
xlabel('sec')
ylabel('m')
title('z compare vo and ekf2')

subplot(5,1,4);
hold on
plot(T_att_eul_vo.timestamp * 1e-6,rad2deg(T_att_eul_vo.yaw),'color',[0.2 0.8 0.2],'linewidth',2);
plot(T_att_eul.timestamp * 1e-6,rad2deg(T_att_eul.yaw),'-.b','linewidth',1);
xlabel('sec')
ylabel('deg')
title('yaw compare vo and ekf2')

subplot(5,1,5);
hold on
plot(T_visual_odometry.timestamp(1:end-1) * 1e-6,diff(T_visual_odometry.timestamp) * 1e-6,'.','linewidth',2)
plot(T_cpu.timestamp * 1e-6,T_cpu.load)
xlabel('sec')
ylabel('sec')
title('vo input time interval')

%%
figure(10);clf;
subplot(2,1,1)
hold on
plot(T_att_eul_setpoint.timestamp * 1e-6,rad2deg(T_att_eul_setpoint.roll),'color',[0.2 0.8 0.2],'linewidth',2)
plot(T_att_eul.timestamp * 1e-6,rad2deg(T_att_eul.roll),'color',[0 0 0],'linewidth',1)
xlabel('sec')
ylabel('deg')
title('roll')

subplot(2,1,2)
hold on
plot(T_pos_setpoint.timestamp * 1e-6,T_pos_setpoint.y,'color',[0.2 0.8 0.2],'linewidth',2)
plot(T_pos.timestamp * 1e-6,T_pos.y,'color',[0 0 0],'linewidth',1)
xlabel('sec')
ylabel('m')
title('y')

%%
figure(11);clf;
subplot(2,1,1)
plot(T_cpu.timestamp * 1e-6,T_cpu.load)
xlabel('sec')
title('cpu')

subplot(2,1,2)
plot(T_cpu.timestamp * 1e-6,T_cpu.ram_usage)
xlabel('sec')
title('ram')