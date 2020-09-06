% Plot Errors
close all;

updates_ang_t_2_=updates_ang_t_2;
nGgroups=size(updates_ang_t_2);
updates_ang_t_2_(2:nGgroups,1)=updates_ang_t_2_(2:nGgroups,1);
updates_ang_t_2_(2:nGgroups,2)=updates_ang_t_2_(2:nGgroups,2)+0.3;
updates_ang_t_2_(2:nGgroups,3)=updates_ang_t_2_(2:nGgroups,3);
updates_ang_t_2_(2:nGgroups,4)=updates_ang_t_2_(2:nGgroups,4)-5;
updates_ang_t_2_(2:nGgroups,5)=updates_ang_t_2_(2:nGgroups,5)+30;
updates_ang_t_2_(2:nGgroups,6)=updates_ang_t_2_(2:nGgroups,6)-15;

figure,
rotate3d on;
h = [-1000 260 800 500];
set(gcf,'Position',h)
subplot(2,3,1),hold on;title('Pitch-Roll');
plot(updates_ang_t_2_(:,1),updates_ang_t_2_(:,2),'*-');
xlabel('Pitch(бу)'); ylabel('Roll(бу)'); axis equal;
subplot(2,3,2),hold on;title('Roll-Yaw');
plot(updates_ang_t_2_(:,2),updates_ang_t_2_(:,3),'*-');
xlabel('Roll(бу)'); ylabel('Yaw(бу)'); axis equal;
subplot(2,3,3),hold on;title('Pitch-Yaw');
plot(updates_ang_t_2_(:,1),updates_ang_t_2_(:,3),'*-');
xlabel('Pitch(бу)'); ylabel('Yaw(бу)'); axis equal;
subplot(2,3,4),hold on;title('X-Y');
plot(updates_ang_t_2_(:,4),updates_ang_t_2_(:,5),'*-');
xlabel('X(mm)'); ylabel('Y(mm)'); axis equal;
subplot(2,3,5),hold on;title('Y-Z');
plot(updates_ang_t_2_(:,5),updates_ang_t_2_(:,6),'*-');
xlabel('Y(mm)'); ylabel('Z(mm)'); axis equal;
subplot(2,3,6),hold on;title('X-Z');
plot(updates_ang_t_2_(:,4),updates_ang_t_2_(:,6),'*-');
xlabel('X(mm)'); ylabel('Z(mm)'); axis equal;