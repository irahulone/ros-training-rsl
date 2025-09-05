
clear all

folderPath = fullfile(pwd,"rosbag2_2025_04_28-17_42_03");
bagReader = ros2bagreader(folderPath);

%%

topic = select(bagReader,"Topic","/motor_pwr");
motor_pwr_f = readMessages(topic);
%%
topic = select(bagReader,"Topic","/ri/liquid_lvl");
is_full_f = readMessages(topic);
%%

topic = select(bagReader,"Topic","/ri/chrg_align");
is_align_f = readMessages(topic);
%%

topic = select(bagReader,"Topic","/ri/pump_ctrl");
pump_ctrl_f = readMessages(topic);

%%

topic = select(bagReader,"Topic","/r4/cmd_vel");
r4_cmdvel_f = readMessages(topic);
%%

topic = select(bagReader,"Topic","/r2/du2/vel");
du2_vel_f = readMessages(topic);

%%

for i = 1:length(motor_pwr_f)
    motor_pwr(i) = motor_pwr_f{i}.data(1);
end

for i = 1:length(is_full_f)
    is_full(i) = is_full_f{i}.data(1);
end

for i = 1:length(is_align_f)
    is_align(i) = is_align_f{i}.data(1);
end

for i = 1:length(pump_ctrl_f)
    pump_ctrl(i) = pump_ctrl_f{i}.data(1);
end

%%
for i = 1:length(r4_cmdvel_f)
    r4_cmdvel_linx(i) = r4_cmdvel_f{i}.linear.x(1);
end


%%

t = (0:0.05:length(motor_pwr_f))';

%%

%aa = 1;
%bb = 400;

%aa1 = 212;
%bb1 = 1128;

%aa2 = 100;
%bb2 = 682;

%t_cmdvel = (0:0.0638:length(cmdvel_f))';

%t_duvel = (0:0.03:length(cmdvel_f))';

%% gps 1 & 2 plot

figure(1)
hold on
set(gca,'fontsize',20)
%plot(des_lat, des_lon, 'd', LineWidth=2, MarkerSize=16, Color=[0 0 0])
%plot(r1_gps(1,1), r1_gps(1,2), 'x', LineWidth=3, MarkerSize=16, Color=[0 0.4470 0.7410])
plot(gps1(aa:bb,1), gps1(aa:bb,2), LineWidth=3, LineStyle="-", Color=[0.6350 0.0780 0.1840]) 
plot(gps2(aa:bb,1), gps2(aa:bb,2), LineWidth=3, LineStyle="-", Color=[0 0.4470 0.7410]) 

%% cmd_vel plot

figure(2)
hold on
set(gca,'fontsize',20)
%plot(des_lat, des_lon, 'd', LineWidth=2, MarkerSize=16, Color=[0 0 0])
%plot(r1_gps(1,1), r1_gps(1,2), 'x', LineWidth=3, MarkerSize=16, Color=[0 0.4470 0.7410])
plot(t(aa1:bb1), lx(aa1:bb1), LineWidth=3, LineStyle="-", Color=[0.6350 0.0780 0.1840]) 
plot(t(aa1:bb1), az(aa1:bb1), LineWidth=3, LineStyle="-", Color=[0 0.4470 0.7410]) 


%%  ref du 1 & 2 vel plot

t_cmdvel = (0:0.025:length(cmdvel_f))';
t = (0:0.1:ln)';
to = 14;

aa2 = 915; bb2 = 1082;
figure(3)
hold on
set(gca,'fontsize',20)
%plot(des_lat, des_lon, 'd', LineWidth=2, MarkerSize=16, Color=[0 0 0])
%plot(r1_gps(1,1), r1_gps(1,2), 'x', LineWidth=3, MarkerSize=16, Color=[0 0.4470 0.7410])
plot(to+t(aa2:bb2)-t(aa2), du1_vel(aa2:bb2), LineWidth=3, LineStyle="-", Color=[0.6350 0.0780 0.1840]) 
plot(to+t(aa2:bb2)-t(aa2), du2_vel(aa2:bb2), LineWidth=3, LineStyle="--", Color=[0 0.4470 0.7410]) 

aa1 = 1570; bb1 = 2236;
figure(4)
hold on
set(gca,'fontsize',20)
%plot(des_lat, des_lon, 'd', LineWidth=2, MarkerSize=16, Color=[0 0 0])
%plot(r1_gps(1,1), r1_gps(1,2), 'x', LineWidth=3, MarkerSize=16, Color=[0 0.4470 0.7410])
plot(to+t_cmdvel(aa1:bb1)-t_cmdvel(aa1), (5.8/5.9)*lx(aa1:bb1), LineWidth=2, LineStyle="-", Color=[0.4940 0.1840 0.5560]) 
plot(to+t_cmdvel(aa1:bb1)-t_cmdvel(aa1), az(aa1:bb1), LineWidth=3, LineStyle="-", Color=[0.4660 0.6740 0.1880]) 


%% gps compute velocity

gps1_vel = 0;
gps2_vel = 0;
gps_agg_vel = 0;
for i = 1:length(gps1)-1
    k = (gps1(i+1, 1) - gps1(i, 1))^2 + (gps1(i+1, 2) - gps1(i, 2))^2;
    kk = (sqrt(k))*10^5;
    if kk <= 0.5
        gps1_vel(i) = kk;
    end

    l = (gps2(i+1, 1) - gps2(i, 1))^2 + (gps2(i+1, 2) - gps2(i, 2))^2;
    ll = (sqrt(l))*10^5;
    if ll <= 0.5
        gps2_vel(i) = ll;
    end

    m = (gps_agg(i+1, 1) - gps_agg(i, 1))^2 + (gps_agg(i+1, 2) - gps_agg(i, 2))^2;
    mm = (sqrt(l))*10^5;
    if ll <= 0.5
        gps_agg_vel(i) = 2.6*mm;
    end
    
end
gps1_vel = smoothdata(gps1_vel, "movmean", 20);
gps2_vel = smoothdata(gps2_vel, "movmean", 20);
gps_agg_vel = smoothdata(gps_agg_vel, "movmean", 15);
gps_agg_vel(1) = 0; gps_agg_vel(2) = 0; gps_agg_vel(3) = 0;



%%  
aa1 = 50;
bb1 = 944;

aa2 = 120;
bb2 = 255;

aa = 1;
bb = 247;


alt = 50;
des_lat = gps_agg(1,1);
des_lon = gps_agg(1,2);
origin = [des_lat, des_lon, alt];
[xEast, yNorth] = latlon2local(gps_agg(aa:bb,1), gps_agg(aa:bb,2), alt, origin);

figure(6); hold on;
set(gca,'fontsize',20); grid on; 

%plot(0, 0, 'd', LineWidth=2, MarkerSize=16, Color=[0 0 0])
plot(xEast(1), yNorth(1), 'x', LineWidth=3, MarkerSize=16, Color=[0.8500 0.3250 0.0980])
plot(xEast, -yNorth,  LineWidth=3, LineStyle="-.", Color=[0.8500 0.3250 0.0980])
plot(xEast(end), -yNorth(end), 'o', LineWidth=3, MarkerSize=16, Color=[0.8500 0.3250 0.0980])
axis('equal'); % set 1:1 aspect ratio to see real-world shape
box on
ylim([0, 30])
xlim([-10, 20])

legend('run 1: rover start', 'run 1: rover path', 'run 1: rover end', 'run 2: rover start', 'run 2: rover path', 'run 2: rover end','Interpreter','latex')
ylabel('\textbf{x-distance (m)}','Interpreter','latex');
xlabel('\textbf{y-distance (m)}','Interpreter','latex');
set(gca,'FontSize',18)


% obtain angular velocity 
td = 32.22 - 0.162;
radius = sqrt((abs(xEast(1)) - abs(xEast(end)))^2 + (abs(yNorth(1)) - abs(yNorth(end)))^2) ;
dy = abs(yNorth(end)) - abs(xEast(end));
v_act = dy/(td*radius);



t_cmdvel = (0:0.056:length(cmdvel_f))';
t = (0:0.208:ln)';



figure(5)

set(gca,'fontsize',20)
hold on; %title("\textbf{Rover velocity (vehicle-level)}",'Interpreter','latex')
plot(t_cmdvel(aa1:bb1)-t_cmdvel(aa1), lx(aa1:bb1), LineWidth=2, LineStyle="-", Color=[0.8500 0.3250 0.0980]) 
plot(t(aa:bb), 2.0*gps_agg_vel(aa:bb), LineWidth=2, LineStyle="-.", Color=[0.8500 0.3250 0.0980]) 
%yline(v_act, LineWidth=2, LineStyle="--", Color=[0.8500 0.3250 0.0980])
%plot(t_cmdvel(aa1:bb1)-t_cmdvel(aa1), az(aa1:bb1), LineWidth=3, LineStyle="-", Color=[0.4940 0.1840 0.5560]) 
xlabel("\textbf{Time (s)}",'Interpreter','latex')
ylabel("\textbf{Translational velocity (m/s)}",'Interpreter','latex')
%xlim([0, 25])
%ylim([0, 0.35])
%legend("run-1: translational velocity set = 0.40 \ m/s", "run-1: translational velocity actual $\approx$ 0.38 \ m/s", "run-1: rotational velocity interpreted $\approx$ 0.02 \ m/s", "run-2: translational velocity set = 0.60 \ m/s", "run-2: translational velocity actual $\approx$ 0.58 \ m/s", "run-2: rotational velocity interpreted $\approx$ 0.02 \ m/s", "run-1 $\&$ 2: rotational velocity set = 0.00 \ m/s", 'Interpreter','latex')
legend("run-1: translational velocity set = 0.40 \ m/s", "run-1: translational velocity actual $\approx$ 0.38 \ m/s", "run-2: translational velocity set = 0.60 \ m/s", "run-2: translational velocity actual $\approx$ 0.58 \ m/s", 'Interpreter','latex')

grid on
box on


%%

t_old = 0;
t_cur = 0;

for i = 11:10:247
    t_cur = t(i);
    td = t_cur - t_old;
    radius = sqrt((abs(xEast(i-10)) - abs(xEast(i)))^2 + (abs(yNorth(i-10)) - abs(yNorth(i)))^2) ;
    dy(i) = xEast(i-10) - xEast(i);
    
    %v_act(i) = dy/(td*radius);

    t_old = t_cur;
end
dy(1) = 0;

aa = 1; bb = 235;

figure(7)

set(gca,'fontsize',20)
hold on; %title("\textbf{Rover velocity (vehicle-level)}",'Interpreter','latex')
%plot(t_cmdvel(aa1:bb1)-t_cmdvel(aa1), lx(aa1:bb1), LineWidth=2, LineStyle="-", Color=[0 0.4470 0.7410]) 
%plot(t(aa:bb), 3.3*gps_agg_vel(aa:bb), LineWidth=2, LineStyle="-.", Color=[0 0.4470 0.7410]) 

plot(t_cmdvel(aa1:bb1)-t_cmdvel(aa1), az(aa1:bb1), LineWidth=3, LineStyle="-", Color=[0.4940 0.1840 0.5560])

for i = 11:10:247
    t_cur = t(i);
    td = t_cur - t_old;
    radius = sqrt((abs(xEast(i-10)) - abs(xEast(i)))^2 + (abs(yNorth(i-10)) - abs(yNorth(i)))^2) ;
    dy(i) = -(xEast(i-10) - xEast(i));
    plot(t(i), 0.15*dy(i), "-*", LineWidth=2, MarkerSize=8, Color=[0.8500 0.3250 0.0980])
   
end

%plot(t(aa:bb), 1*dy(aa:bb), LineWidth=2, LineStyle="-.", Color=[0 0.4470 0.7410])

%yline(v_act, LineWidth=2, LineStyle=":", Color=[0 0.4470 0.7410])
xlabel("\textbf{Time (s)}",'Interpreter','latex')
ylabel("\textbf{Rotational velocity (rad/s)}",'Interpreter','latex')
%xlim([0, 25])
%ylim([0, 0.35])
%legend("Linear velocity (m/s)", "Angular velocity (rad/s)" ,'Interpreter','latex')
grid on
box on
legend("run-1 $\&$ 2: rotational velocity set", "run-1: rotational velocity actual", "run-2: rotational velocity actual", 'Interpreter','latex')



%%






%%


figure (2)

subplot(2,2,1); hold on
set(gca,'fontsize',20)
x = linspace(-3.8,3.8);
y_cos = cos(x);
plot(x,y_cos);
title('Subplot 1: Cosine')

subplot(2,2,2); hold on
set(gca,'fontsize',20)
y_poly = 1 - x.^2./2 + x.^4./24;
plot(x,y_poly,'g');
title('Subplot 2: Polynomial')

subplot(2,2,[3,4]); hold on
set(gca,'fontsize',20)
plot(x,y_cos,'b',x,y_poly,'g');
title('Subplot 3 and 4: Both')




























%%

% print(gcf,'foo.png','-dpng','-r400');

