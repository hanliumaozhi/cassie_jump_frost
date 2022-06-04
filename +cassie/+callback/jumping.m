function jumping(nlp, bounds, varargin)
    % Extract Plant
    domain = nlp.Plant;
    
    % Add Virtual Constraints
    domain.VirtualConstraints.time.imposeNLPConstraint(nlp, [bounds.time.kp, bounds.time.kd], [1,1]);
    %% Tau Boundary [0,1]
    addNodeConstraint(nlp, cassie.constraints.tau0(nlp), ...
        [{'T'},domain.VirtualConstraints.time.PhaseParamName], 'first', 0, 0, 'Nonlinear');
    addNodeConstraint(nlp, cassie.constraints.tauF(nlp), ...
        [{'T'},domain.VirtualConstraints.time.PhaseParamName], 'last', 0, 0, 'Nonlinear');
    %% leg length
%     for k =1:21
%        addNodeConstraint(nlp, cassie.constraints.left_leg_length(nlp), ...
%            {'x'}, k, bounds.leg_length(k)-0.01, bounds.leg_length(k)+0.01, 'Nonlinear');
%     end
%     
%     for k =1:21
%        addNodeConstraint(nlp, cassie.constraints.right_leg_length(nlp), ...
%            {'x'}, k, bounds.leg_length(k), bounds.leg_length(k), 'Nonlinear');
%     end
    for k=1:21
        addNodeConstraint(nlp, cassie.constraints.base_link_z(nlp), ...
            {'x'}, k, bounds.leg_length(k), bounds.leg_length(k), 'Nonlinear');
    end
    
    %% toe x
    for k=1:21
        addNodeConstraint(nlp, cassie.constraints.toe_x_eq(nlp), ...
            {'x'}, k, 0, 0, 'Nonlinear');
    end
    
    %% com x
    for k =1:21
       addNodeConstraint(nlp, cassie.constraints.com_x(nlp), ...
           {'x'}, k, 0, 0, 'Nonlinear');
    end
    
    %% com y
    for k =1:21
       addNodeConstraint(nlp, cassie.constraints.com_y(nlp), ...
           {'x'}, k, 0, 0, 'Nonlinear');
    end
%     com_x_cstr = NlpFunction('Name',['com_x_' domain.Name],...
%         'Dimension',1,...
%         'lb', 0,...
%         'ub', 0,...
%         'Type','Nonlinear',...
%         'SymFun', cassie.constraints.com_x(nlp),...
%         'DepVariables',{nlp.OptVarTable.x});
%     
%     addConstraint(nlp, ['com_x_' domain.Name], 'all', com_x_cstr);
    
    
    %% pitch momentum
%     pitch_momentum_cstr = NlpFunction('Name', 'pitch_momentum_Jump',...
%         'Dimension',1,...
%         'lb', 0,...
%         'ub', 0,...
%         'Type','Nonlinear',...
%         'SymFun', cassie.constraints.pitch_momentum(nlp),...
%         'DepVariables',{nlp.OptVarTable.x, nlp.OptVarTable.dx});
%     
%     addConstraint(nlp, ['pitch_momentum_', domain.Name], 'all', pitch_momentum_cstr);
%     for k =1:21
%        addNodeConstraint(nlp, cassie.constraints.pitch_momentum(nlp), ...
%            {'x', 'dx'}, k, 0, 0, 'Nonlinear');
%     end
    
    %% Costs
    
    % Torque Cost
    addRunningCost(nlp, cassie.costs.pitch_moment(nlp), {'x','dx','ddx'});
end