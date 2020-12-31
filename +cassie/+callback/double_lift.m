function double_lift(nlp, src, tar, bounds, varargin)
    % First call the class method
    nlp.Plant.rigidImpactConstraint(nlp, src, tar, bounds, varargin{:});
    
    % Don't need time continuity constraint
    removeConstraint(nlp,'tContDomain');
    
    % Remove default constraint and add back selectable one
    removeConstraint(nlp,['xDiscreteMap' nlp.Plant.Name]);
    % removeConstraint(nlp,['dxDiscreteMap' nlp.Plant.Name]);
    
    selected = SymVariable('s',[nlp.Plant.numState,1]);
    R = nlp.Plant.R;
    x = nlp.Plant.States.x;
    xn = nlp.Plant.States.xn;
    x_diff = selected.*(R*x-xn);
    x_map = SymFunction(['xDiscreteMap' nlp.Plant.Name],x_diff,{x,xn}, selected);
    
    
    selected = ones(20,1);
    addNodeConstraint(nlp, x_map, {'x','xn'}, 'first', 0, 0, 'Linear', selected);
%     
%     selected = SymVariable('ds',[nlp.Plant.numState,1]);
%     dx = nlp.Plant.States.dx;
%     dxn = nlp.Plant.States.dxn;
%     dx_diff = selected.*(R*dx-dxn);
%     dx_map = SymFunction(['dxDiscreteMap' nlp.Plant.Name],dx_diff,{dx,dxn}, selected);
%     
%     selected = zeros(20,1);
%     selected(1) = 1;
%     selected(2) = 1;
%     selected(3) = 1;
%     selected(4) = 1;
%     selected(5) = 1;
%     selected(6) = 1;
%     addNodeConstraint(nlp, dx_map, {'dx','dxn'}, 'first', 0, 0, 'Linear', selected);
end