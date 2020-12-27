function right_stance(nlp, bounds, varargin)
    % Extract Plant
    domain = nlp.Plant;
    
    % Add Virtual Constraints
    domain.VirtualConstraints.time.imposeNLPConstraint(nlp, [bounds.time.kp, bounds.time.kd], [1,1]);
    
    %% Tau Boundary [0,1]
    addNodeConstraint(nlp, cassie.constraints.tau0(nlp), ...
        [{'T'},domain.VirtualConstraints.time.PhaseParamName], 'first', 0, 0, 'Nonlinear');
    addNodeConstraint(nlp, cassie.constraints.tauF(nlp), ...
        [{'T'},domain.VirtualConstraints.time.PhaseParamName], 'last', 0, 0, 'Nonlinear');
    
    %% Step Length
%     addNodeConstraint(nlp, cassie.constraints.step_length(nlp), ...
%        {'x'}, 'last', bounds.step_length.lb, bounds.step_length.ub, 'Nonlinear');

    %% Average Velocity
    average_velocity_cstr = NlpFunction('Name',['average_velocity_', domain.Name],...
        'Dimension',1,...
        'lb', bounds.average_velocity.lb,...
        'ub', bounds.average_velocity.ub ,...
        'Type','Linear',...
        'SymFun',cassie.constraints.average_velocity(nlp),...
        'DepVariables',[nlp.OptVarTable.T(1),nlp.OptVarTable.x(1),nlp.OptVarTable.x(end)]);
    
    addConstraint(nlp, ['average_velocity_', domain.Name], 'last', average_velocity_cstr);
    
    %% Toe-to-Toe Distance  
    addNodeConstraint(nlp, cassie.constraints.step_width(nlp), ...
        {'x'}, 'all', bounds.toe_to_toe_width.lb, bounds.toe_to_toe_width.ub, 'Nonlinear');

    %% Pelvis-to-Toe Distance
    addNodeConstraint(nlp, cassie.constraints.distance_pelvis_to_stance_toe(nlp), ...
        {'x'}, 'all', bounds.distance_pelvis_to_stance_toe.lb, bounds.distance_pelvis_to_stance_toe.ub, 'Nonlinear');
    
    %% Knee-to-Toe Distance
    %addNodeConstraint(nlp, cassie.constraints.distance_leg(nlp), ...
    %    {'x'}, 'all', bounds.distance_leg.lb, bounds.distance_leg.ub, 'Nonlinear');
    
    %% Com Constraints
    %for k =1:21
    %    addNodeConstraint(nlp, cassie.constraints.com_constraints(nlp), ...
    %        {'x'}, k, bounds.com_constraints.lb(k+21), bounds.com_constraints.ub(k+21), 'Nonlinear');
    %end
    
    %% Swing Foot Clearance
    addNodeConstraint(nlp, cassie.constraints.step_height(nlp), ...
        {'x'}, floor(nlp.NumNode/2), bounds.foot_clearance.lb, bounds.foot_clearance.ub, 'Nonlinear');
    
    %% Swing Toe Velocity
    % Throughout all nodes
    addNodeConstraint(nlp, cassie.constraints.swing_toe_angular_velocity_x(nlp), ...
        {'x','dx'}, 'all', -0.5, 0.5, 'Nonlinear');
    addNodeConstraint(nlp, cassie.constraints.swing_toe_angular_velocity_z(nlp), ...
        {'x','dx'}, 'all', -0.1, 0.1, 'Nonlinear');
    
%     addNodeConstraint(nlp, cassie.constraints.swing_toe_angular_velocity_x(nlp), ...
%         {'x','dx'}, 'all', -1, 1, 'Nonlinear');
%     addNodeConstraint(nlp, cassie.constraints.swing_toe_angular_velocity_z(nlp), ...
%         {'x','dx'}, 'all', -1, 1, 'Nonlinear');
    % Final node
    addNodeConstraint(nlp, cassie.constraints.swing_toe_linear_velocity_x(nlp), ...
        {'x','dx'}, 'last', bounds.swing_toe_vel_x.lb, bounds.swing_toe_vel_x.ub, 'Nonlinear');
    addNodeConstraint(nlp, cassie.constraints.swing_toe_linear_velocity_y(nlp), ...
        {'x','dx'}, 'last', bounds.swing_toe_vel_y.lb, bounds.swing_toe_vel_y.ub, 'Nonlinear');
    addNodeConstraint(nlp, cassie.constraints.swing_toe_linear_velocity_z(nlp), ...
        {'x','dx'}, 'last', bounds.swing_toe_vel_z.lb, bounds.swing_toe_vel_z.ub, 'Nonlinear');
    
    %% Swing knee velocity
    %     dx = nlp.Plant.States.dx;
    %     expression = dx(10);
    %     func = SymFunction(['swing_knee_velocity_',nlp.Plant.Name], expression, {nlp.Plant.States.dx});
    %     addNodeConstraint(nlp, func, ...
    %         {'dx'}, 'last', bounds.swing_knee_vel.lb,  bounds.swing_knee_vel.ub, 'Linear');
    
    %% Average pitch
    average_pitch_cstr = NlpFunction('Name',['average_pitch_', domain.Name],...
        'Dimension',1,...
        'lb', bounds.average_pitch.lb,...
        'ub', bounds.average_pitch.ub ,...
        'Type','Linear',...
        'SymFun', cassie.constraints.average_pitch(nlp),...
        'DepVariables',nlp.OptVarTable.x);
    
    addConstraint(nlp, ['average_pitch_', domain.Name], 'last', average_pitch_cstr);
    
    %% Average yaw
    average_yaw_cstr = NlpFunction('Name',['average_yaw_', domain.Name],...
        'Dimension',1,...
        'lb', bounds.average_yaw.lb,...
        'ub', bounds.average_yaw.ub ,...
        'Type','Linear',...
        'SymFun',cassie.constraints.average_yaw(nlp),...
        'DepVariables',nlp.OptVarTable.x);
    
    addConstraint(nlp, ['average_yaw_', domain.Name], 'last', average_yaw_cstr);
    
    %% Average hip abduction
    average_hip_abduction_cstr = NlpFunction('Name',['average_hip_abduction_', domain.Name],...
        'Dimension',2,...
        'lb', bounds.average_hip_abduction.lb,...
        'ub', bounds.average_hip_abduction.ub ,...
        'Type','Linear',...
        'SymFun', cassie.constraints.average_hip_abduction(nlp),...
        'DepVariables',nlp.OptVarTable.x);
    
    addConstraint(nlp, ['average_hip_abduction_', domain.Name], 'last', average_hip_abduction_cstr);
    
    %% Average hip rotation
    average_hip_rotation_cstr = NlpFunction('Name',['average_hip_rotation_', domain.Name],...
        'Dimension',2,...
        'lb', bounds.average_hip_rotation.lb,...
        'ub', bounds.average_hip_rotation.ub ,...
        'Type','Linear',...
        'SymFun', cassie.constraints.average_hip_rotation(nlp),...
        'DepVariables',nlp.OptVarTable.x);
    
    addConstraint(nlp, ['average_hip_rotation_', domain.Name], 'last', average_hip_rotation_cstr);
        
    %% Costs
    
    % Torque Cost
    addRunningCost(nlp, cassie.costs.velocity(nlp), 'dx');
    
end


