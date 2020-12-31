function bounds = GetBound(robot)

model_bounds = robot.getLimits();
bounds = struct();

model_bounds.states.x.lb(1:3) = [-10,-10,-10];
model_bounds.states.x.ub(1:3) = [10,10,10];

model_bounds.states.x.lb(4:6) = deg2rad(-5);
model_bounds.states.x.ub(4:6) = deg2rad(5);

% yaw equal zero
model_bounds.states.x.lb(4) = deg2rad(0);
model_bounds.states.x.ub(4) = deg2rad(0);

model_bounds.states.x.lb(6) = deg2rad(0);
model_bounds.states.x.ub(6) = deg2rad(0);

model_bounds.states.x.lb([7,8,14,15]) = deg2rad(-10);
model_bounds.states.x.ub([7,8,14,15]) = deg2rad(10);

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

bounds.Jump.inputs.ConstraintWrench.ffixedKneeSpring.lb = -10000;
bounds.Jump.inputs.ConstraintWrench.ffixedKneeSpring.ub = 10000;
bounds.Jump.inputs.ConstraintWrench.ffixedKneeSpring.x0 = 100;

bounds.Jump.inputs.ConstraintWrench.ffourBar.lb = -10000;
bounds.Jump.inputs.ConstraintWrench.ffourBar.ub = 10000;
bounds.Jump.inputs.ConstraintWrench.ffourBar.x0 = 100;

bounds.Jump.params.pfixedKneeSpring.lb = -0*ones(2,1);
bounds.Jump.params.pfixedKneeSpring.ub = 0*ones(2,1);
bounds.Jump.params.pfixedKneeSpring.x0 = zeros(2,1);

bounds.Jump.params.pfourBar.lb = -0*ones(2,1);
bounds.Jump.params.pfourBar.ub = 0*ones(2,1);
bounds.Jump.params.pfourBar.x0 = zeros(2,1);

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

bounds.Jump.leg_length = [0.698551; 0.69871; 0.699385; 0.700854; 0.703276; 0.706807; 0.711606; 0.717832; 0.725642; 0.735194; 0.746646; 0.760157; 0.775885; 0.793987; 0.814621; 0.837946; 0.86412; 0.893301; 0.925647; 0.961315; 1];



%% Landing
bounds.Landing = model_bounds;

bounds.Landing.time.t0.lb = 0;
bounds.Landing.time.t0.ub = 0;
bounds.Landing.time.t0.x0 = 0;

bounds.Landing.time.tf.lb = 0.4;
bounds.Landing.time.tf.ub = 0.4;
bounds.Landing.time.tf.x0 = 0.4;

bounds.Landing.time.duration.lb = 0.4;
bounds.Landing.time.duration.ub = 0.4;
bounds.Landing.time.duration.x0 = 0.4;

bounds.Landing.params.atime.lb = -10*ones(6*10,1);
bounds.Landing.params.atime.ub = 10*ones(6*10,1);
bounds.Landing.params.atime.x0 = zeros(6*10,1);

bounds.Landing.params.ptime.lb = [bounds.Landing.time.tf.lb, bounds.Landing.time.t0.lb];
bounds.Landing.params.ptime.ub = [bounds.Landing.time.tf.ub, bounds.Landing.time.t0.ub];
bounds.Landing.params.ptime.x0 = [bounds.Landing.time.t0.x0, bounds.Landing.time.tf.x0];

bounds.Landing.time.kp = 100;
bounds.Landing.time.kd = 20;

%% Flight
bounds.Flight = model_bounds;

bounds.Flight.time.t0.lb = 0;
bounds.Flight.time.t0.ub = 0;
bounds.Flight.time.t0.x0 = 0;

bounds.Flight.time.tf.lb = 0.4;
bounds.Flight.time.tf.ub = 0.4;
bounds.Flight.time.tf.x0 = 0.4;

bounds.Flight.time.duration.lb = 0.4;
bounds.Flight.time.duration.ub = 0.4;
bounds.Flight.time.duration.x0 = 0.4;

bounds.Flight.params.atime.lb = -10*ones(6*10,1);
bounds.Flight.params.atime.ub = 10*ones(6*10,1);
bounds.Flight.params.atime.x0 = zeros(6*10,1);

bounds.Flight.params.ptime.lb = [bounds.Flight.time.tf.lb, bounds.Flight.time.t0.lb];
bounds.Flight.params.ptime.ub = [bounds.Flight.time.tf.ub, bounds.Flight.time.t0.ub];
bounds.Flight.params.ptime.x0 = [bounds.Flight.time.t0.x0, bounds.Flight.time.tf.x0];

bounds.Flight.time.kp = 100;
bounds.Flight.time.kd = 20;

bounds.Flight.inputs.ConstraintWrench.ffixedKneeSpring.lb = -10000;
bounds.Flight.inputs.ConstraintWrench.ffixedKneeSpring.ub = 10000;
bounds.Flight.inputs.ConstraintWrench.ffixedKneeSpring.x0 = 100;

bounds.Flight.inputs.ConstraintWrench.ffourBar.lb = -10000;
bounds.Flight.inputs.ConstraintWrench.ffourBar.ub = 10000;
bounds.Flight.inputs.ConstraintWrench.ffourBar.x0 = 100;

bounds.Flight.params.pfixedKneeSpring.lb = -0*ones(2,1);
bounds.Flight.params.pfixedKneeSpring.ub = 0*ones(2,1);
bounds.Flight.params.pfixedKneeSpring.x0 = zeros(2,1);

bounds.Flight.params.pfourBar.lb = -0*ones(2,1);
bounds.Flight.params.pfourBar.ub = 0*ones(2,1);
bounds.Flight.params.pfourBar.x0 = zeros(2,1);

%% impact

% Right Impact
bounds.ImpactDoubleSupport = model_bounds;

bounds.DoubleLift = model_bounds;

end
