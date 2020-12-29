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

end
