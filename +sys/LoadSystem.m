function [sys, domains, guards] = LoadSystem(robot, load_path, varargin)
    %% 
    jumping = cassie.domain.double_support(robot, load_path, 'guard', 'double_lift', 'name', 'Jump');
    double_lift = cassie.domain.lift(robot, load_path, 'next_domain', 'flight', 'leg', 'double');
    flight = cassie.domain.flight(robot, load_path, 'guard', 'double_support_impact');
    
    domains = [jumping, flight];
    guards = [double_lift];
    
    sys = HybridSystem('Cassie');
    sys = addVertex(sys, {'Jump', 'Flight'}, 'Domain', {jumping, flight});
    
    sys = addEdge(sys, 'Jump', 'Flight');
    sys = setEdgeProperties(sys, 'Jump', 'Flight', 'Guard', double_lift);
end
