function h_nsf = left_toe_ground_reaction_force(robot)
% left toe ground reaction force
GRF = robot.Inputs.ConstraintWrench.fLeftSole;
h_nsf = UnilateralConstraint(robot, GRF(3), 'leftToeGroundReactionForce', 'fLeftSole');

end

