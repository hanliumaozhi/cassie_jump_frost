function bounds = GetBound(robot)

model_bounds = robot.getLimits();
bounds = struct();

model_bounds.states.x.lb(1:3) = [-10,-10,-10];
model_bounds.states.x.ub(1:3) = [10,10,10];

model_bounds.states.x.lb(4:6) = deg2rad(-1);
model_bounds.states.x.ub(4:6) = deg2rad(1);

model_bounds.states.x.lb([7,8,14,15]) = deg2rad(-5);
model_bounds.states.x.ub([7,8,14,15]) = deg2rad(5);

%% Jump
bounds.Jump = model_bounds;

bounds.Jump.time.t0.lb = 0;
bounds.Jump.time.t0.ub = 0;
bounds.Jump.time.t0.x0 = 0;

bounds.Jump.time.tf.lb = 0.4;
bounds.Jump.time.tf.ub = 0.4;
bounds.Jump.time.tf.x0 = 0.4;

bounds.Jump.time.duration.lb = 0.4;
bounds.Jump.time.duration.ub = 0.4;
bounds.Jump.time.duration.x0 = 0.4;

bounds.Jump.inputs.ConstraintWrench.fRightSole.lb = -10000;
bounds.Jump.inputs.ConstraintWrench.fRightSole.ub = 10000;
bounds.Jump.inputs.ConstraintWrench.fRightSole.x0 = 100;

bounds.Jump.inputs.ConstraintWrench.fLeftSole.lb = -10000;
bounds.Jump.inputs.ConstraintWrench.fLeftSole.ub = 10000;
bounds.Jump.inputs.ConstraintWrench.fLeftSole.x0 = 100;

bounds.Jump.params.pRightSole.lb = [-1;-1;0;0;-pi/2];
bounds.Jump.params.pRightSole.ub = [1;1;0;0;-pi/2];
bounds.Jump.params.pRightSole.x0 = [0;0;0;0;-pi/2];

bounds.Jump.params.pLeftSole.lb = [-1;-1;0;0;-pi/2];
bounds.Jump.params.pLeftSole.ub = [1;1;0;0;-pi/2];
bounds.Jump.params.pLeftSole.x0 = [0;0;0;0;-pi/2];

bounds.Jump.params.atime.lb = -10*ones(6*10,1);
bounds.Jump.params.atime.ub = 10*ones(6*10,1);
bounds.Jump.params.atime.x0 = zeros(6*10,1);

bounds.Jump.params.ptime.lb = [bounds.Jump.time.tf.lb, bounds.Jump.time.t0.lb];
bounds.Jump.params.ptime.ub = [bounds.Jump.time.tf.ub, bounds.Jump.time.t0.ub];
bounds.Jump.params.ptime.x0 = [bounds.Jump.time.t0.x0, bounds.Jump.time.tf.x0];

bounds.Jump.time.kp = 100;
bounds.Jump.time.kd = 20;



%% Landing
bounds.Landing = model_bounds;

%% Flight
bounds.Flight = model_bounds;

%% impact

% Right Impact
bounds.ImpactDoubleSupport = model_bounds;



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
