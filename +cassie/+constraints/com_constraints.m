function [ com_constraints_fun ] = com_constraints( nlp )
%   com_constraints
%   Creates SymFunction for com constraint
%
%   Author: Ross Hartley
%   Date: 2018-03-19

% Choose frames
switch nlp.Plant.Name
    case 'RightStance'
        p_toe = nlp.Plant.ContactPoints.RightToeBottom.computeCartesianPosition;
        
    case 'LeftStance'
        p_toe = nlp.Plant.ContactPoints.LeftToeBottom.computeCartesianPosition;
        
    otherwise
        error('Cannot create com constraint. Unknown domain type')
end

% distance_leg is defined as the Euclidean distance of 
% knee wrt stance toe
% p_pelvis = nlp.Plant.States.x(1:3);
com = getComPosition(nlp.Plant);
expr = p_toe(1) - com(1);
com_constraints_fun = SymFunction(['com_constraints_',nlp.Plant.Name], expr, {nlp.Plant.States.x});
    
end

