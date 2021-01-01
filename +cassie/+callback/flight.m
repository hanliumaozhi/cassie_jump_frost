function flight(nlp, bounds, varargin)
    % Extract Plant
    domain = nlp.Plant;
    
    % Add Virtual Constraints
    domain.VirtualConstraints.time.imposeNLPConstraint(nlp, [bounds.time.kp, bounds.time.kd], [1,1]);
    %% Tau Boundary [0,1]
    addNodeConstraint(nlp, cassie.constraints.tau0(nlp), ...
        [{'T'},domain.VirtualConstraints.time.PhaseParamName], 'first', 0, 0, 'Nonlinear');
    addNodeConstraint(nlp, cassie.constraints.tauF(nlp), ...
        [{'T'},domain.VirtualConstraints.time.PhaseParamName], 'last', 0, 0, 'Nonlinear');
    %% motor velocity
%    for k = 1:21
    %
%         addNodeConstraint(nlp, cassie.constraints.motor_velocity(nlp, 7), ...
%             {'dx'}, k, 0, 0, 'Nonlinear');
%         addNodeConstraint(nlp, cassie.constraints.motor_velocity(nlp, 8), ...
%             {'dx'}, k, 0, 0, 'Nonlinear');
%         addNodeConstraint(nlp, cassie.constraints.motor_velocity(nlp, 9), ...
%             {'dx'}, k, 0, 0, 'Nonlinear');
%         addNodeConstraint(nlp, cassie.constraints.motor_velocity(nlp, 10), ...
%             {'dx'}, k, 0, 0, 'Nonlinear');
%         addNodeConstraint(nlp, cassie.constraints.motor_velocity(nlp, 13), ...
%             {'dx'}, k, 0, 0, 'Nonlinear');
%         addNodeConstraint(nlp, cassie.constraints.motor_velocity(nlp, 14), ...
%             {'dx'}, k, 0, 0, 'Nonlinear');
%         addNodeConstraint(nlp, cassie.constraints.motor_velocity(nlp, 15), ...
%             {'dx'}, k, 0, 0, 'Nonlinear');
%         addNodeConstraint(nlp, cassie.constraints.motor_velocity(nlp, 16), ...
%             {'dx'}, k, 0, 0, 'Nonlinear');
%         addNodeConstraint(nlp, cassie.constraints.motor_velocity(nlp, 17), ...
%             {'dx'}, k, 0, 0, 'Nonlinear');
%         addNodeConstraint(nlp, cassie.constraints.motor_velocity(nlp, 20), ...
%             {'dx'}, k, 0, 0, 'Nonlinear');
%    end
    
    %% Costs
    X0 = SymVariable('xxx',[nlp.Plant.numState,1]);
    
    % Torque Cost
    addRunningCost(nlp, cassie.costs.motor_position(nlp, X0), {'x','xxx'});
end