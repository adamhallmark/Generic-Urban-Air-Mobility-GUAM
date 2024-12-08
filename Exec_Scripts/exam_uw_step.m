%% sim parameters
model = 'GUAM';
% use timeseries input
userStruct.variants.refInputType=3; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Piecewise Bezier, 5=Default(doublets)

%% ADAM EDITS TO SPECIFY DESIRED TRAJECTORY AND SETUP REFERENCE INPUTS

% Prescribe column vector of times when desired inertial position and 
% velocity can be prescribed
dt = 0.001;
time = 0:dt:30;
N_time = length(time);

vel     = zeros(N_time, 3);
vel_i   = zeros(N_time, 3);
pos     = zeros(N_time, 3);
chi     = zeros(N_time, 3);
chid    = zeros(N_time, 3);

step_time = 1;
step_idx = floor(step_time/dt)+1;

vel_i(step_idx:end, :) = [ones(N_time-step_idx+1, 1), ...
    zeros(N_time-step_idx+1, 1), -10*ones(N_time-step_idx+1, 1)];

pos = cumtrapz(dt, vel_i);

%%
%
% Compute heading
chi     = atan2(vel_i(:,2),vel_i(:,1));
chid    = gradient(chi)./gradient(time).';

% add stars library blocks for quaternion functions
addpath(genpath('lib'));

% compute velocity in heading frame
q = QrotZ(chi);
vel = Qtrans(q,vel_i);

% setup trajectory to match bus
RefInput.Vel_bIc_des    = timeseries(vel,time); % Heading frame velocity
RefInput.pos_des        = timeseries(pos,time); % Inertial Position
RefInput.chi_des        = timeseries(chi,time); % Heading Angle
RefInput.chi_dot_des    = timeseries(chid,time); % Heading Angle Rate
RefInput.vel_des        = timeseries(vel_i,time); % Inertial Position

target.RefInput = RefInput;

%% ADAM MODIFICATION: Specify Controller Variant

userStruct.variants.ctrlType=5;

%% Prepare to run simulation
% set initial conditions and add trajectory to SimInput
simSetup;

%% ADAM MODIFICATION: change aeropropulsive model to s-function model

SimIn.fmType = "SFunction";

%%
open(model);