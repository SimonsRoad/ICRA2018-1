%%% Configuration file
%%% Permits various adjustments to parameters of the SLAM algorithm.
%%% See ekfslam_sim.m for more information

addpath('./utilities');
set(0,'DefaultFigureWindowStyle','normal')
format compact

% Initialise states and other global variables
global XX PX LM POSES  DATA PARAMS SWITCH IA MSMTS LB LM_obstacle

% Load table for the lower bounds of the non-centrality parameter
% load('msmts.mat'); MSMTS= msmts; clear msmts;
% load('poses.mat'); POSES= poses; clear poses;
% load('LM.mat'); LM= LM'; 
% load('LowerBound2dof_1e8_new.mat');
POSES= [0,0,0; 10000,0,0];
% LM= [ [20;-10],[20;10],[60;-9],[60;9],[100;-8],[100;8],[140;-7],[140;7],[180;-6],[180;6],...
%       [220;-5],[220;5],[260;-4],[260;4],[300;-3],[300;3],[340;-2],[340;2],[380;-1],[380;1],...
%       [420;-0.5],[420;0.5]];

% Tunel LMs
posLM(1,:)= linspace(0,3000,102);
negLM(1,:)= linspace(0,3000,102);

% well spaced
posLM(2,1:15)= linspace(-15,-15,15);
negLM(2,1:15)= linspace(15,15,15);

% Bad spaced
posLM(2,16:102)= linspace(-2.5,-2.5,102-15);
negLM(2,16:102)= linspace(2.5,2.5,102-15);
LM= [posLM,negLM];

% Add or remove landmarks
% LM(:,[73,75])= [];
% LM(:,[37,42])= [];
% LM(:,[17])= [];
% LM= [LM, [12;-110], [12;-100], [-15;-105]];
% LM= [LM, [-77;-130]];
% 
% % LM= [LM, [-77.1;-93], [-77.2;-93], [-77.3;-93], [-77.4;-93], [-77.5;-93]];
% % LM= [LM, [-130;-100], [-130;-101], [-130;-102], [-130;-103], [-130;-104]];
% % LM= [LM, [-150;-81.5], [-150;-81], [-150;-80], [-150;-83], [-150;-85]];
% % LM= [LM, [-165;-161], [-164;-160], [-164;-162], [-165.5;-160], [-165;-162]];




% Number of time epochs to run the simulation
PARAMS.numSteps= 6000;
PARAMS.dt= 0.1;

% Initial control inputs
iwp= 1;
G= deg2rad(0);

% control parameters
PARAMS.at_waypoint= 5;
PARAMS.V= 15;
PARAMS.maxRateG= deg2rad(50);
PARAMS.maxG= deg2rad(30);
PARAMS.wheelbase= 1; % metres, vehicle wheel-base
PARAMS.veh= [0 -4    -4   0; 
              0 0.5  -0.5 0]; % vehicle animation

% control noises
PARAMS.sigmaV= 0.3; % m/s ( default= 0.3 )
PARAMS.sigmaG= deg2rad(2); % radians ( default= (3.0*pi/180) )
PARAMS.Q= [PARAMS.sigmaV^2, 0; 0, PARAMS.sigmaG^2];

% observation parameters
PARAMS.maxRange= 25;
PARAMS.m_F= 2; % d.o.f. of one measurement

% observation noises
PARAMS.sigmaR= 0.3; % metres ( default 0.1 )
PARAMS.sigmaB= deg2rad(2); % radians ( default (1.0*pi/180) )
PARAMS.R= [PARAMS.sigmaR^2 0; 0 PARAMS.sigmaB^2];

% Integrity 
PARAMS.I_y= 1e-10;
PARAMS.I_c= 0.01;
PARAMS.I_FOV= 1e-12;
PARAMS.I_REQ= 1e-5;
PARAMS.alert_limit= 1;
PARAMS.gate= chi2inv(1-PARAMS.I_c,PARAMS.m_F); 
% PARAMS.gate= inf;

% Misdetection probability
PARAMS.P_ME= 0; %0.01;

% switches
SWITCH.control_noise= 1; % if 0, velocity and gamma are perfect
SWITCH.sensor_noise= 1; % if 0, measurements are perfect
SWITCH.inflate_noise= 0; % if 1, the estimated Q and R are inflated (ie, add stabilising noise)
SWITCH.heading_known= 0; % if 1, the vehicle heading is observed directly at each iteration
SWITCH.batch_update= 1; % if 1, process scan in batch, if 0, process sequentially
SWITCH.seed_random= 6; % if not 0, seed the randn() with its value at beginning of simulation (for repeatability)
SWITCH.use_IEKF= 0; % if 1, use iterated EKF for updates, if 0, use normal EKF
SWITCH.profile= 0; % if 1, turn on MatLab profiling to measure time consumed by simulator functions
SWITCH.graphics= 1; % if 0, avoids plotting most animation data to maximise simulation speed
SWITCH.update_global= 0; % if 1, This alternative is a "global constraint" model devised by Jose Guivant, and may have better linearisation properties than the conventional range-bearing model.
SWITCH.association= 1; % if 0, associations are given; if 1, they are estimated using the LNN
SWITCH.consistency= 0; % If 1, checks innovation vector consistency.
SWITCH.ME= 0; % If 0, no mis-extractions; if 1, there are mis-extractions.
SWITCH.update_if_necessary= 0; % If 1, only updates when integrity improves.
if SWITCH.seed_random, rand('state',SWITCH.seed_random), randn('state',SWITCH.seed_random), end
if SWITCH.profile, profile on -detail builtin, end

%% INITIALIZATIONS 
 
% True & estimated state
xtrue= POSES(1,:)';
XX= xtrue; 

% initial pose covariance
PX= [eps,   0,   0;
       0, eps,   0;
       0,   0, eps];


PARAMS.ftag= 1:size(LM,2);     % identifier for each landmark
da_table= zeros(1,size(LM,2)); % data association table 

% more initializations
step= 0; IA= 0;
DATA.errorXX= zeros(5000,3);
DATA.stdXX= zeros(5000,3);
DATA.eps= zeros(5000,1);
DATA.stdEps= zeros(5000,1);
DATA.P_HMI= zeros(5000,1);
DATA.P_HMI_CA= zeros(5000,1);
DATA.path= zeros(5000,2);
DATA.PCA_k= ones(5000,1);
DATA.PCA_K= ones(5000,1);
DATA.IA= zeros(5000,1);
DATA.numAssoc= zeros(5000,1);

DATA.gamma= cell(5000,1);
DATA.stngamma= cell(5000,1);
DATA.PCAt= ones(5000,1);
DATA.realPCA= zeros(5000,1);
DATA.calcPCA= zeros(5000,1);

DATA.frames(500)= struct('cdata',[],'colormap',[]);



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% 
% %%% Configuration file
% %%% Permits various adjustments to parameters of the SLAM algorithm.
% %%% See ekfslam_sim.m for more information
% 
% addpath('./utilities');
% set(0,'DefaultFigureWindowStyle','normal')
% format compact
% 
% % Initialise states and other global variables
% global XX PX LM POSES  DATA PARAMS SWITCH IA MSMTS LB LM_obstacle
% 
% % Load table for the lower bounds of the non-centrality parameter
% % load('msmts.mat'); MSMTS= msmts; clear msmts;
% % load('poses.mat'); POSES= poses; clear poses;
% % load('LM.mat'); LM= LM'; 
% % load('LowerBound2dof_1e8_new.mat');
% POSES= [0,0,0; 10000,0,0];
% % LM= [ [20;-10],[20;10],[60;-9],[60;9],[100;-8],[100;8],[140;-7],[140;7],[180;-6],[180;6],...
% %       [220;-5],[220;5],[260;-4],[260;4],[300;-3],[300;3],[340;-2],[340;2],[380;-1],[380;1],...
% %       [420;-0.5],[420;0.5]];
% 
% % Tunel LMs
% posLM(1,:)= linspace(0,2000,68);
% negLM(1,:)= linspace(0,2000,68);
% 
% % well spaced
% posLM(2,1:15)= linspace(-10,-10,15);
% negLM(2,1:15)= linspace(10,10,15);
% 
% % Bad spaced
% posLM(2,16:68)= linspace(-2.5,-2.5,53);
% negLM(2,16:68)= linspace(2.5,2.5,53);
% LM= [posLM,negLM];
% 
% % Add or remove landmarks
% % LM(:,[73,75])= [];
% % LM(:,[37,42])= [];
% % LM(:,[17])= [];
% % LM= [LM, [12;-110], [12;-100], [-15;-105]];
% % LM= [LM, [-77;-130]];
% % 
% % % LM= [LM, [-77.1;-93], [-77.2;-93], [-77.3;-93], [-77.4;-93], [-77.5;-93]];
% % % LM= [LM, [-130;-100], [-130;-101], [-130;-102], [-130;-103], [-130;-104]];
% % % LM= [LM, [-150;-81.5], [-150;-81], [-150;-80], [-150;-83], [-150;-85]];
% % % LM= [LM, [-165;-161], [-164;-160], [-164;-162], [-165.5;-160], [-165;-162]];
% 
% 
% 
% 
% % Number of time epochs to run the simulation
% PARAMS.numSteps= 1000;
% PARAMS.dt= 0.1;
% 
% % Initial control inputs
% iwp= 1;
% G= deg2rad(0);
% 
% % control parameters
% PARAMS.at_waypoint= 5;
% PARAMS.V= 15;
% PARAMS.maxRateG= deg2rad(50);
% PARAMS.maxG= deg2rad(30);
% PARAMS.wheelbase= 1; % metres, vehicle wheel-base
% PARAMS.veh= [0 -4    -4   0; 
%               0 0.5  -0.5 0]; % vehicle animation
% 
% % control noises
% PARAMS.sigmaV= 0.3; % m/s ( default= 0.3 )
% PARAMS.sigmaG= deg2rad(2); % radians ( default= (3.0*pi/180) )
% PARAMS.Q= [PARAMS.sigmaV^2, 0; 0, PARAMS.sigmaG^2];
% 
% % observation parameters
% PARAMS.maxRange= 25;
% PARAMS.m_F= 2; % d.o.f. of one measurement
% 
% % observation noises
% PARAMS.sigmaR= 0.3; % metres ( default 0.1 )
% PARAMS.sigmaB= deg2rad(2); % radians ( default (1.0*pi/180) )
% PARAMS.R= [PARAMS.sigmaR^2 0; 0 PARAMS.sigmaB^2];
% 
% % Integrity 
% PARAMS.I_y= 1e-8;
% PARAMS.I_c= 0.01;
% PARAMS.I_FOV= 1e-10;
% PARAMS.I_REQ= 1e-5;
% PARAMS.alert_limit= 1;
% PARAMS.gate= chi2inv(1-PARAMS.I_c,PARAMS.m_F); 
% % PARAMS.gate= inf;
% 
% % Misdetection probability
% PARAMS.P_ME= 0; %0.01;
% 
% % switches
% SWITCH.control_noise= 1; % if 0, velocity and gamma are perfect
% SWITCH.sensor_noise= 1; % if 0, measurements are perfect
% SWITCH.inflate_noise= 0; % if 1, the estimated Q and R are inflated (ie, add stabilising noise)
% SWITCH.heading_known= 0; % if 1, the vehicle heading is observed directly at each iteration
% SWITCH.batch_update= 1; % if 1, process scan in batch, if 0, process sequentially
% SWITCH.seed_random= 1; % if not 0, seed the randn() with its value at beginning of simulation (for repeatability)
% SWITCH.use_IEKF= 0; % if 1, use iterated EKF for updates, if 0, use normal EKF
% SWITCH.profile= 0; % if 1, turn on MatLab profiling to measure time consumed by simulator functions
% SWITCH.graphics= 1; % if 0, avoids plotting most animation data to maximise simulation speed
% SWITCH.update_global= 0; % if 1, This alternative is a "global constraint" model devised by Jose Guivant, and may have better linearisation properties than the conventional range-bearing model.
% SWITCH.association= 1; % if 0, associations are given; if 1, they are estimated using the LNN
% SWITCH.consistency= 0; % If 1, checks innovation vector consistency.
% SWITCH.ME= 0; % If 0, no mis-extractions; if 1, there are mis-extractions.
% SWITCH.update_if_necessary= 0; % If 1, only updates when integrity improves.
% if SWITCH.seed_random, rand('state',SWITCH.seed_random), randn('state',SWITCH.seed_random), end
% if SWITCH.profile, profile on -detail builtin, end
% 
% %% INITIALIZATIONS 
%  
% % True & estimated state
% xtrue= POSES(1,:)';
% XX= xtrue; 
% 
% % initial pose covariance
% PX= [eps,   0,   0;
%        0, eps,   0;
%        0,   0, eps];
% 
% 
% PARAMS.ftag= 1:size(LM,2);     % identifier for each landmark
% da_table= zeros(1,size(LM,2)); % data association table 
% 
% % more initializations
% step= 0; IA= 0;
% DATA.errorXX= zeros(5000,3);
% DATA.stdXX= zeros(5000,3);
% DATA.eps= zeros(5000,1);
% DATA.stdEps= zeros(5000,1);
% DATA.P_HMI= zeros(5000,1);
% DATA.P_HMI_CA= zeros(5000,1);
% DATA.path= zeros(5000,2);
% DATA.PCA_k= ones(5000,1);
% DATA.PCA_K= ones(5000,1);
% DATA.IA= zeros(5000,1);
% DATA.numAssoc= zeros(5000,1);
% 
% DATA.gamma= cell(5000,1);
% DATA.stngamma= cell(5000,1);
% DATA.PCAt= ones(5000,1);
% DATA.realPCA= zeros(5000,1);
% DATA.calcPCA= zeros(5000,1);
% 



