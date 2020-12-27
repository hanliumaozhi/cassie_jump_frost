%% Main script for Cassie
% song liu

%% Setting up path
clear; close all; clc;
% Restore search path to factory-installed state, Start up file for MATLAB program.
%restoredefaultpath; matlabrc;  

% specify the path to the FROST
% only for my computer you need change it
frost_path  = '../frost-dev';
addpath(frost_path);
frost_addpath; % initialize FROST
cur = utils.get_root_path();
export_path = fullfile(cur, 'gen/opt'); % path to export compiled C++ and MEX files
utils.init_path(export_path);
%% robot model settings
%cur = utils.get_root_path();
%urdf = fullfile(cur,'urdf','casia_nnn.urdf'); % Build full file name from parts 
urdf = fullfile(cur,'urdf','cassie.urdf'); % Build full file name from parts
% some options

% if 'delay_set' is true, the computation of system dynamics (Coriolis
% vector) will be delayed. Delaying this operation will save significant
% loading time.
delay_set = false;
OMIT_CORIOLIS = true;
LOAD = false;
% if 'load_sym' is true, it will load symbolic expressions from previously
% save external files instead of re-compute them. It reduce the loading
% time by 7-10 faster. 
% Set it to false for the first time, and save expressions after loaded the
% model. 
% % % load_sym  = false; % if true, it will load symbolic expression from 
% % % if load_sym    
% % %     load_path = 'gen/sym'; % path to export binary Mathematica symbolic expression (MX) files
% % %     utils.init_path(load_path);
% % % else
% % %     load_path   = []; 
% % % end
LOAD_PATH = fullfile(cur, 'gen\sym');
%% load robot model
% load the robot model
robot = sys.LoadModel(urdf);  % call  loadDynamics()

if LOAD
    loadDynamics(robot, LOAD_PATH, delay_set,{},'OmitCoriolisSet',OMIT_CORIOLIS);
    %robot.loadDynamics(LOAD_PATH, OMIT_CORIOLIS);
    system = sys.LoadSystem(robot, LOAD_PATH);
else
    robot.configureDynamics('DelayCoriolisSet',OMIT_CORIOLIS,'OmitCoriolisSet',OMIT_CORIOLIS);
    %robot.configureDynamics('DelayCoriolisSet',OMIT_CORIOLIS);
    [system, domains, guards] = sys.LoadSystem(robot, '');
end
% load hybrid system

%% Load optimization problem
% get the boundary values, needs to be manually set all boundaries.
%%%%%%%%%%%
bounds = cassie.GetBounds(robot, 0, 0, 0);
%%%%%%%%%%%
num_grid.RightStance = 10;
num_grid.LeftStance = 10;
% load problem
nlp = HybridTrajectoryOptimization('two_step',system, num_grid, [],'EqualityConstraintBoundary',1e-4);
nlp.Phase(1).Plant.UserNlpConstraint = @cassie.callback.left_stance;
nlp.Phase(2).Plant.UserNlpConstraint = @cassie.callback.right_impact;
nlp.Phase(3).Plant.UserNlpConstraint = @cassie.callback.right_stance;
nlp.Phase(4).Plant.UserNlpConstraint = @cassie.callback.left_impact;
nlp.update; 

nlp.configure(bounds);
nlp.update;

%% Compile stuff if needed (only need to run for the first time)
% compileObjective(nlp,[],[],export_path);
% compileConstraint(nlp,[],[],export_path, {'dynamics_equation'});

compileObjective(nlp,[],[],export_path);
%compileConstraint(nlp,[],[],export_path);


%% Save expression (only need to run for the first time)
load_path   = fullfile(cur, 'gen/sym');
utils.init_path(load_path);
system.saveExpression(load_path); % run this after loaded the optimization problem



%% you can update bounds without reloading the problem. It is much much faster!!!
%%%%%%%%%%%%%%%%%%%%%%%%
bounds = cassie.GetBounds(robot, 0, 0, 0);
%%%%%%%%%%%%%%%%%%%%%%%%%
nlp.configure(bounds);
% bounds = opt.GetBounds(robot); % walk in space
nlp.update;

% removeConstraint(nlp.Phase(1),'u_friction_cone_RightSole');
% removeConstraint(nlp.Phase(1),'u_zmp_RightSole');
% removeConstraint(nlp.Phase(3),'u_friction_cone_LeftSole');
% removeConstraint(nlp.Phase(3),'u_zmp_LeftSole');


%% update initial condition if use pre-existing gaits
% param = load('local/tmp_gait.mat');
% cassie.updateInitCondition(nlp,param.gait);



%% solve (two ways to call "solve")
% 1. If use the typical variables values as the initial guess
% 2. If use existing solutoin as the initial guess
%[gait, sol, info] = opt.solve(nlp, sol); % if use previous solution "sol"
% 3. warm start using use existing solutoin as the initial guess
%[gait, sol, info] = opt.solve(nlp, sol, info); % if use previous solution "sol"
[gait, sol, info, total_time] = cassie.solve(nlp);


 
%% save
save('local/tmp_casia_60mm_leg81_peizhong_gait.mat','gait','sol','info','bounds');

%% single cassia gait
index = 1;
cassia_gait.RightStance.HAlpha = zeros(1,10,6);
cassia_gait.LeftStance.HAlpha = zeros(1,10,6);
height = zeros(1);
vx = 0; %modify
cassia_gait.LeftStance.HAlpha(index,:,:) = reshape(gait(1).params.atime,[10,6]);
cassia_gait.RightStance.HAlpha(index,:,:) = reshape(gait(3).params.atime,[10,6]);
cassia_gait.Desired_Velocity(index) = vx;
cassia_gait.ct(index) = 2.5000;
cassia_gait.Velocity(index) = vx;

%% save single
save('local/tmp_cassie_cassia_gait.mat','gait','sol','info','bounds');
%% multi cassia gait 
index = 1;
cassia_gait.RightStance.HAlpha = zeros(1,10,6);
cassia_gait.LeftStance.HAlpha = zeros(1,10,6);
height = zeros(1);
cassie_com_constraints = load('local/cassie_com_constr.mat');
cassie_com_constraints = cassie_com_constraints.cassie_toe_to_com;
for vx = -0.4:0.05:0.8
    bounds = cassie.GetBounds(robot, vx, 0, 0, cassie_com_constraints(index,:));
    nlp.configure(bounds);
    nlp.update;
    [gait, sol, info, total_time] = cassie.solve(nlp);
    
    gait_index = strcat('casia_gait_',num2str(vx),'.mat');
    save_path = fullfile('local/gait_row',gait_index);
    save(save_path,'gait','sol','info','bounds');
    
    cassia_gait.LeftStance.HAlpha(index,:,:) = reshape(gait(1).params.atime,[10,6]);
    cassia_gait.RightStance.HAlpha(index,:,:) = reshape(gait(3).params.atime,[10,6]);
    cassia_gait.Desired_Velocity(index) = vx;
    cassia_gait.ct(index) = 2.5000;
    cassia_gait.Velocity(index) = vx;
    index = index+1;
    height(index) = gait(1).states.x(3,1);
%     gait_index = strcat('gait_',num2str(vx));
%     save_path = fullfile('local',gait_index);
end
%%
save('local/gait_row/casia_nnn_com_constr_gait_output_row.mat','cassia_gait');
%% modify casia_gait 
cassia_gait.LeftStance.HAlpha(:,(1:10),:)=cassia_gait.LeftStance.HAlpha(:,[6 7 8 9 10 1 2 3 4 5],:);
cassia_gait.RightStance.HAlpha(:,(1:10),:)=cassia_gait.RightStance.HAlpha(:,[6 7 8 9 10 1 2 3 4 5],:);
s = 0:0.02:1;
index = 1;

syms a0 a1 a2 a3 a4 a5;
syms t;
out=a0*nchoosek(5,0)*t^0*(1-t)^5+...
    a1*nchoosek(5,1)*t^1*(1-t)^(5-1)+...
    a2*nchoosek(5,2)*t^2*(1-t)^(5-2)+...
    a3*nchoosek(5,3)*t^3*(1-t)^(5-3)+...
    a4*nchoosek(5,4)*t^4*(1-t)^(5-4)+...
    a5*nchoosek(5,5)*t^5*(1-t)^(5-5);
output=collect(out,t);
ABes2Poy=[-1, 5, -10, 10, -5, 1;
    5,-20,30,-20,5,0;
    -10,30,-30,10,0,0;
    10,-20,10,0,0,0;
    -5,5,0,0,0,0;
    1,0,0,0,0,0];
out2=[t^5,t^4,t^3,t^2,t,1]*ABes2Poy*[a0;a1;a2;a3;a4;a5];
output2=collect(out2,t);
ploy2bezier = inv(ABes2Poy);

%for vx = -0.4:0.05:0.8
for vx = 0
    % left Stance
    coeff_q4_right = reshape(cassia_gait.LeftStance.HAlpha(index,4,:),[1,6]);
    coeff_q3_right = reshape(cassia_gait.LeftStance.HAlpha(index,3,:),[1,6]);
    q4_right = bezier(coeff_q4_right, s);
    q3_right = bezier(coeff_q3_right, s);
    [LA_right, LL_right] = Cassia_FK_LEG(q3_right,q4_right);
    p_LL_right = polyfit(s, LL_right, 5);
    p_LA_right = polyfit(s, LA_right, 5);
    coeff_LL_right = (ploy2bezier*p_LL_right')';
    coeff_LA_right = (ploy2bezier*p_LA_right')';
    cassia_gait.LeftStance.HAlpha(index,4,:) = coeff_LL_right;
    cassia_gait.LeftStance.HAlpha(index,3,:) = coeff_LA_right;
    
    coeff_q4_left = reshape(cassia_gait.LeftStance.HAlpha(index,9,:),[1,6]);
    coeff_q3_left = reshape(cassia_gait.LeftStance.HAlpha(index,8,:),[1,6]);
    q4_left = bezier(coeff_q4_left, s);
    q3_left = bezier(coeff_q3_left, s);
    [LA_left, LL_left] = Cassia_FK_LEG(q3_left,q4_left);
    p_LL_left = polyfit(s, LL_left, 5);
    p_LA_left = polyfit(s, LA_left, 5);
    coeff_LL_left = (ploy2bezier*p_LL_left')';
    coeff_LA_left = (ploy2bezier*p_LA_left')';
    cassia_gait.LeftStance.HAlpha(index,9,:) = coeff_LL_left;
    cassia_gait.LeftStance.HAlpha(index,8,:) = coeff_LA_left;
    
    %right Stance
    coeff_q4_right = reshape(cassia_gait.RightStance.HAlpha(index,4,:),[1,6]);
    coeff_q3_right = reshape(cassia_gait.RightStance.HAlpha(index,3,:),[1,6]);
    q4_right = bezier(coeff_q4_right, s);
    q3_right = bezier(coeff_q3_right, s);
    [LA_right, LL_right] = Cassia_FK_LEG(q3_right,q4_right);
    p_LL_right = polyfit(s, LL_right, 5);
    p_LA_right = polyfit(s, LA_right, 5);
    coeff_LL_right = (ploy2bezier*p_LL_right')';
    coeff_LA_right = (ploy2bezier*p_LA_right')';
    cassia_gait.RightStance.HAlpha(index,4,:) = coeff_LL_right;
    cassia_gait.RightStance.HAlpha(index,3,:) = coeff_LA_right;
    
    coeff_q4_left = reshape(cassia_gait.RightStance.HAlpha(index,9,:),[1,6]);
    coeff_q3_left = reshape(cassia_gait.RightStance.HAlpha(index,8,:),[1,6]);
    q4_left = bezier(coeff_q4_left, s);
    q3_left = bezier(coeff_q3_left, s);
    [LA_left, LL_left] = Cassia_FK_LEG(q3_left,q4_left);
    p_LL_left = polyfit(s, LL_left, 5);
    p_LA_left = polyfit(s, LA_left, 5);
    coeff_LL_left = (ploy2bezier*p_LL_left')';
    coeff_LA_left = (ploy2bezier*p_LA_left')';
    cassia_gait.RightStance.HAlpha(index,9,:) = coeff_LL_left;
    cassia_gait.RightStance.HAlpha(index,8,:) = coeff_LA_left;
    
    
    index = index+1;
end
%%
save('local/cassie_gait_library_test1.mat','cassia_gait');
%% test
vx=0;
index = 1;
cassia_gait.RightStance.HAlpha = zeros(1,10,6);
cassia_gait.LeftStance.HAlpha = zeros(1,10,6);
height = zeros(1);
cassia_gait.LeftStance.HAlpha(index,:,:) = reshape(gait(1).params.atime,[10,6]);
cassia_gait.RightStance.HAlpha(index,:,:) = reshape(gait(3).params.atime,[10,6]);
cassia_gait.Desired_Velocity(index) = vx;
cassia_gait.ct(index) = 2.5000;
cassia_gait.Velocity(index) = vx;

%% animation
ANIM_PATH = fullfile(cur,'gen', 'animator');
if ~exist(ANIM_PATH,'dir')
    mkdir(ANIM_PATH);
end
anim = plot.cassie_load_animation(robot, gait, [], 'ExportPath', ANIM_PATH, 'SkipExporting', true); % set 'SkipExporting' = false, only for the first time!



%% you can check the violation of constraints/variables and the value of each cost function by calling the following functions.
tol = 1e-3;
checkConstraints(nlp,sol,tol,'local/constr_check.txt') % 
checkVariables(nlp,sol,tol,'local/var_check.txt') % 

checkCosts(nlp,sol,'local/cost_check.txt') % 



%% you can also plot the states and torques w.r.t upper/lower bounds.
%plot.plotOptStates(robot,nlp,gait);
plot.plotOptTorques(robot,nlp,gait);

PlotTrajectories(gait(1).states.x, gait(1).states.dx, {robot.Joints.Name});


