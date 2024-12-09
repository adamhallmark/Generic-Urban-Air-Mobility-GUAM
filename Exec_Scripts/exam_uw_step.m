%% sim parameters
model = 'GUAM';
% use timeseries input
userStruct.variants.refInputType=3; % 1=FOUR_RAMP, 2= ONE_RAMP, 3=Timeseries, 4=Piecewise Bezier, 5=Default(doublets)

%% ADAM EDITS TO SPECIFY DESIRED TRAJECTORY AND SETUP REFERENCE INPUTS

% Load refTraj mat file for the reference trajectory in one of the three
% modes: hover, cruise, or transition

% hover:
% load("C:\Users\texba\OneDrive\Documents\AEM 699\sci_tech_2025_paper\matlab\data\refTraj_hover.mat")

% cruise:
% load("C:\Users\texba\OneDrive\Documents\AEM 699\sci_tech_2025_paper\matlab\data\refTraj_cruise.mat")

% transition:
load("C:\Users\texba\OneDrive\Documents\AEM 699\sci_tech_2025_paper\matlab\data\refTraj_transition.mat")

%%

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
%   5: Hover Baseline
%   6: Cruise Baseline
%   7: Transition Baseline
userStruct.variants.ctrlType=7;

%% Prepare to run simulation
% set initial conditions and add trajectory to SimInput
simSetup;

%% ADAM MODIFICATION: change aeropropulsive model to s-function model

% SimIn.fmType = "SFunction";

%%
open(model);