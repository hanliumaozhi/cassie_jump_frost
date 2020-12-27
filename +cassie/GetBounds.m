function bounds = getBounds(robot, vx, vy, height)

model_bounds = robot.getLimits();
bounds = struct();

model_bounds.states.x.lb(1:3) = [-10,-10,-10];
model_bounds.states.x.ub(1:3) = [10,10,10];

model_bounds.states.x.lb(4:6) = deg2rad(-5);
model_bounds.states.x.ub(4:6) = deg2rad(5);

model_bounds.states.x.lb([7,8,15,16]) = deg2rad(-5);
model_bounds.states.x.ub([7,8,15,16]) = deg2rad(5);

model_bounds.inputs.Control.u.lb([5,10]) = -0.01;
model_bounds.inputs.Control.u.ub([5,10]) = 0.01;

model_bounds.swing_knee_vel.lb = -30; % Effectively -inf
model_bounds.swing_knee_vel.ub = 30; % Effectively +inf

model_bounds.swing_toe_vel_x.lb = -2;%-abs(vx)*0.3;
model_bounds.swing_toe_vel_x.ub = 2;%+abs(vx)*0.3;
model_bounds.swing_toe_vel_y.lb = -2;
model_bounds.swing_toe_vel_y.ub = 2;
model_bounds.swing_toe_vel_z.lb = -2;
model_bounds.swing_toe_vel_z.ub = 2;


% some error 
model_bounds.average_pitch.lb = -deg2rad(0.5);
model_bounds.average_pitch.ub = deg2rad(0.5);

model_bounds.average_yaw.lb = -deg2rad(0.5);
model_bounds.average_yaw.ub = deg2rad(0.5);



model_bounds.average_hip_abduction.lb = deg2rad(-70*abs(vy));
model_bounds.average_hip_abduction.ub = deg2rad(+70*abs(vy));

model_bounds.average_hip_rotation.lb = deg2rad(-70*abs(vy));
model_bounds.average_hip_rotation.ub = deg2rad(+70*abs(vy));

model_bounds.average_velocity.lb = 0.0;
model_bounds.average_velocity.ub = 0.0;

model_bounds.foot_clearance.lb = 0.15;
model_bounds.foot_clearance.ub = 0.2;

model_bounds.distance_pelvis_to_stance_toe.lb = 0.5;
model_bounds.distance_pelvis_to_stance_toe.ub = 1.0;

model_bounds.toe_to_toe_width.lb = -0.40; % 0.27 nominal width
model_bounds.toe_to_toe_width.ub = -0.10;

%% Right Stance
bounds.RightStance = model_bounds;

% bounds.RightStance.states.dx.lb(17) = max(deg2rad(-3), bounds.RightStance.states.dx.lb(17));
% bounds.RightStance.states.dx.ub(17) = min(deg2rad(+3), bounds.RightStance.states.dx.ub(17));

bounds.RightStance.time.t0.lb = 0;
bounds.RightStance.time.t0.ub = 0;
bounds.RightStance.time.t0.x0 = 0;

bounds.RightStance.time.tf.lb = 0.4;
bounds.RightStance.time.tf.ub = 0.4;
bounds.RightStance.time.tf.x0 = 0.4;

bounds.RightStance.time.duration.lb = 0.4;
bounds.RightStance.time.duration.ub = 0.4;
bounds.RightStance.time.duration.x0 = 0.4;

bounds.RightStance.inputs.ConstraintWrench.fRightSole.lb = -10000;
bounds.RightStance.inputs.ConstraintWrench.fRightSole.ub = 10000;
bounds.RightStance.inputs.ConstraintWrench.fRightSole.x0 = 100;

bounds.RightStance.inputs.ConstraintWrench.fSpringTransmissions.lb = -10000;
bounds.RightStance.inputs.ConstraintWrench.fSpringTransmissions.ub = 10000;
bounds.RightStance.inputs.ConstraintWrench.fSpringTransmissions.x0 = 100;

bounds.RightStance.params.pSpringTransmissions.lb = -0*ones(2,1);
bounds.RightStance.params.pSpringTransmissions.ub = 0*ones(2,1);
bounds.RightStance.params.pSpringTransmissions.x0 = zeros(2,1);



% bounds.RightStance.inputs.ConstraintWrench.ffixedKneeSpring.lb = -10000;
% bounds.RightStance.inputs.ConstraintWrench.ffixedKneeSpring.ub = 10000;
% bounds.RightStance.inputs.ConstraintWrench.ffixedKneeSpring.x0 = 100;
% 
% bounds.RightStance.inputs.ConstraintWrench.ffourBar.lb = -10000;
% bounds.RightStance.inputs.ConstraintWrench.ffourBar.ub = 10000;
% bounds.RightStance.inputs.ConstraintWrench.ffourBar.x0 = 100;
% 
% bounds.RightStance.params.pfixedKneeSpring.lb = -0*ones(2,1);
% bounds.RightStance.params.pfixedKneeSpring.ub = 0*ones(2,1);
% bounds.RightStance.params.pfixedKneeSpring.x0 = zeros(2,1);
% 
% bounds.RightStance.params.pfourBar.lb = -0*ones(2,1);
% bounds.RightStance.params.pfourBar.ub = 0*ones(2,1);
% bounds.RightStance.params.pfourBar.x0 = zeros(2,1);

bounds.RightStance.params.pRightToeBottom.lb = [0;0;0;0;-pi/2];
bounds.RightStance.params.pRightToeBottom.ub = [0;0;0;0;-pi/2];
bounds.RightStance.params.pRightToeBottom.x0 = [0;0;0;0;-pi/2];

bounds.RightStance.params.atime.lb = -10*ones(6*10,1);
bounds.RightStance.params.atime.ub = 10*ones(6*10,1);
bounds.RightStance.params.atime.x0 = zeros(6*10,1);

bounds.RightStance.params.ptime.lb = [bounds.RightStance.time.tf.lb, bounds.RightStance.time.t0.lb];
bounds.RightStance.params.ptime.ub = [bounds.RightStance.time.tf.ub, bounds.RightStance.time.t0.ub];
bounds.RightStance.params.ptime.x0 = [bounds.RightStance.time.t0.x0, bounds.RightStance.time.tf.x0];

bounds.RightStance.time.kp = 100;
bounds.RightStance.time.kd = 20;



%% Left Stance
bounds.LeftStance = model_bounds;

% bounds.RightStance.states.dx.lb(10) = max(deg2rad(-3), bounds.RightStance.states.dx.lb(10));
% bounds.RightStance.states.dx.ub(10) = min(deg2rad(+3), bounds.RightStance.states.dx.ub(10));

bounds.LeftStance.time.t0.lb = 0;
bounds.LeftStance.time.t0.ub = 0;
bounds.LeftStance.time.t0.x0 = 0;

bounds.LeftStance.time.tf.lb = 0.4;
bounds.LeftStance.time.tf.ub = 0.4;
bounds.LeftStance.time.tf.x0 = 0.4;

bounds.LeftStance.time.duration.lb = 0.4;
bounds.LeftStance.time.duration.ub = 0.4;
bounds.LeftStance.time.duration.x0 = 0.4;

bounds.LeftStance.inputs.ConstraintWrench.fLeftSole.lb = -10000;
bounds.LeftStance.inputs.ConstraintWrench.fLeftSole.ub = 10000;
bounds.LeftStance.inputs.ConstraintWrench.fLeftSole.x0 = 100;

bounds.LeftStance.inputs.ConstraintWrench.fSpringTransmissions.lb = -10000;
bounds.LeftStance.inputs.ConstraintWrench.fSpringTransmissions.ub = 10000;
bounds.LeftStance.inputs.ConstraintWrench.fSpringTransmissions.x0 = 100;

bounds.LeftStance.params.pSpringTransmissions.lb = -0*ones(2,1);
bounds.LeftStance.params.pSpringTransmissions.ub = 0*ones(2,1);
bounds.LeftStance.params.pSpringTransmissions.x0 = zeros(2,1);

% bounds.LeftStance.inputs.ConstraintWrench.ffixedKneeSpring.lb = -10000;
% bounds.LeftStance.inputs.ConstraintWrench.ffixedKneeSpring.ub = 10000;
% bounds.LeftStance.inputs.ConstraintWrench.ffixedKneeSpring.x0 = 100;
% 
% bounds.LeftStance.inputs.ConstraintWrench.ffourBar.lb = -10000;
% bounds.LeftStance.inputs.ConstraintWrench.ffourBar.ub = 10000;
% bounds.LeftStance.inputs.ConstraintWrench.ffourBar.x0 = 100;
% 
% bounds.LeftStance.params.pfixedKneeSpring.lb = -0*ones(2,1);
% bounds.LeftStance.params.pfixedKneeSpring.ub = 0*ones(2,1);
% bounds.LeftStance.params.pfixedKneeSpring.x0 = zeros(2,1);
% 
% bounds.LeftStance.params.pfourBar.lb = -0*ones(2,1);
% bounds.LeftStance.params.pfourBar.ub = 0*ones(2,1);
% bounds.LeftStance.params.pfourBar.x0 = zeros(2,1);

bounds.LeftStance.params.pLeftToeBottom.lb = [-10;-10;-10;0;-pi/2];
bounds.LeftStance.params.pLeftToeBottom.ub = [10;10;10;0;-pi/2];
bounds.LeftStance.params.pLeftToeBottom.x0 = [0;0;0;0;-pi/2];

bounds.LeftStance.params.atime.lb = -10*ones(6*10,1);
bounds.LeftStance.params.atime.ub = 10*ones(6*10,1);
bounds.LeftStance.params.atime.x0 = zeros(6*10,1);

bounds.LeftStance.params.ptime.lb = [bounds.LeftStance.time.tf.lb, bounds.LeftStance.time.t0.lb];
bounds.LeftStance.params.ptime.ub = [bounds.LeftStance.time.tf.ub, bounds.LeftStance.time.t0.ub];
bounds.LeftStance.params.ptime.x0 = [bounds.LeftStance.time.t0.x0, bounds.LeftStance.time.tf.x0];

bounds.LeftStance.time.kp = 100;
bounds.LeftStance.time.kd = 20;

bounds.LeftStance.toe_to_toe_width.lb = -model_bounds.toe_to_toe_width.ub; 
bounds.LeftStance.toe_to_toe_width.ub = -model_bounds.toe_to_toe_width.lb;

%% Impacts

% Right Impact
bounds.RightImpact = model_bounds;
% Left Impact
bounds.LeftImpact = model_bounds;

end
