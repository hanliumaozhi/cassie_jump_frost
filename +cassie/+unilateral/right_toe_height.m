function h_nsf = right_toe_height(robot)
    % right toe height constraint
    p_right_toe = getCartesianPosition(robot, robot.ContactPoints.RightSole);
    h_nsf = UnilateralConstraint(robot, p_right_toe(3), 'rightToeHeight', 'x');
end

 