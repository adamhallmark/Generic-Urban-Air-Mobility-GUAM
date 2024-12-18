% This script is a toplevel script that executes the users desired example case:

addpath('C:\Users\texba\OneDrive\Documents\AEM 699\sci_tech_2025_paper\matlab\Generic-Urban-Air-Mobility-GUAM\Exec_Scripts\');
u_choice = input(sprintf('Specify the desired example case to run:\n\t(1) Sinusoidal Timeseries\n\t(2) Hover to Transition Timeseries\n\t(3) Cruise Climbing Turn Timeseries\n\t(4) Ramp demo\n\t(5) Piecewise Bezier Trajectory\n\t(6) Step Inputs to u, w Timeseries\nUser Input: '));

switch u_choice
    case 1
        exam_TS_Sinusoidal_traj;
    case 2
        exam_TS_Hover2Cruise_traj
    case 3
        exam_TS_Cruise_Climb_Turn_traj
    case 4
        exam_RAMP
    case 5 
        if ~exist("userStruct",'var')
            addpath('./Bez_Functions/');
        end
        exam_Bezier;
    case 6
        exam_uw_step
    otherwise
        fprintf('User needs to supply selection choice (1-5)\n')
        return
end

load('C:\Users\texba\OneDrive\Documents\AEM 699\sci_tech_2025_paper\matlab\data\hover_gains.mat')
load('C:\Users\texba\OneDrive\Documents\AEM 699\sci_tech_2025_paper\matlab\data\transition_gains.mat')
load('C:\Users\texba\OneDrive\Documents\AEM 699\sci_tech_2025_paper\matlab\data\cruise_gains.mat')
% Execute the model
sim(model);
% Create sample output plots
% simPlots_GUAM;

%% Plot stuff I want

vel = logsout{1}.Values.Vehicle.EOM.InertialData.Vel_bIi;
vel_des = logsout{1}.Values.RefInputs.Vel_bIc_des;

figure()

subplot(3,1,1)
plot(vel.Time, vel.Data(:,1), linewidth=2, DisplayName='u')
hold on
plot(vel_des.Time, vel_des.Data(:,1), '--', linewidth=1, DisplayName='u_{cmd}')
grid on
legend(location='best', FontSize=12)
title('x-Velocity Tracking Performance', FontSize=12)
ylabel('ft/s')
ylim('padded')

subplot(3,1,2)
plot(vel.Time, vel.Data(:,2), linewidth=2, DisplayName='v')
hold on
plot(vel_des.Time, vel_des.Data(:,2), '--', linewidth=1, DisplayName='v_{cmd}')
grid on
legend(location='best', FontSize=12)
title('y-Velocity Regulation Performance', FontSize=12)
ylabel('ft/s')
ylim('padded')

subplot(3,1,3)
plot(vel.Time, vel.Data(:,3), linewidth=2, DisplayName='w')
hold on
plot(vel_des.Time, vel_des.Data(:,3), '--', linewidth=1, DisplayName='w_{cmd}')
grid on
legend(location='best', FontSize=12)
title('z-Velocity Tracking Performance', FontSize=12)
ylabel('ft/s')
ylim('padded')