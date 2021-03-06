function [sys, domains, guards] = LoadSystem(robot, load_path, varargin)
    %% 
    jumping = cassie.domain.double_support(robot, load_path, 'guard', 'double_lift', 'name', 'Jump');
    double_lift = cassie.domain.lift(robot, load_path, 'leg', 'flight', 'leg', 'double');
    flight = cassie.domain.flight(robot, load_path, 'guard', 'double_support_impact');
    double_impact = cassie.domain.impact(robot, load_path, 'leg', 'double' , 'next_domain', 'double_support');
    landing = cassie.domain.double_support(robot, load_path, 'guard', 'double_lift', 'name','Landing');
    
    
    domains = [jumping, flight, landing];
    guards = [double_lift, double_impact];
    
    sys = HybridSystem('Cassie');
    sys = addVertex(sys, {'Jump','Flight', 'Landing'}, 'Domain', {jumping, flight, landing});
    
    sys = addEdge(sys, 'Jump', 'Flight');
    sys = setEdgeProperties(sys, 'Jump', 'Flight', 'Guard', double_lift);
    sys = addEdge(sys, 'Flight', 'Landing');
    sys = setEdgeProperties(sys, 'Flight', 'Landing', 'Guard', double_impact);
end
