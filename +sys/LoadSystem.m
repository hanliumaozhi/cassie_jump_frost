function [sys, domains, guards] = LoadSystem(robot, load_path, varargin)
    %% 
    jumping = cassie.domain.double_support(robot, load_path, 'guard', 'double_lift', 'name', 'Jump');
    
    domains = jumping;
    guards = [];
    
    sys = HybridSystem('Cassie');
    sys = addVertex(sys, {'Jump'}, 'Domain', {jumping});
end
