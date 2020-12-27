function [ distance_leg_fun ] = distance_leg( nlp )
%distance_pelvis_to_stance_toe constraint for distance_pelvis_to_stance_toe
%   Creates SymFunction for a distance_pelvis_to_stance_toe constraint
%
%   Author: Ross Hartley
%     Date: 2018-03-19

% Choose frames
switch nlp.Plant.Name
    case 'RightStance'
%         p_toe = nlp.Plant.ContactPoints.RightToeBottom.computeCartesianPosition;
        qpitch = nlp.Plant.States.x(9);
        qknee = nlp.Plant.States.x(10);
        
    case 'LeftStance'
%         p_toe = nlp.Plant.ContactPoints.LeftToeBottom.computeCartesianPosition;
        qpitch = nlp.Plant.States.x(16);
        qknee = nlp.Plant.States.x(17);
        
    otherwise
        error('Cannot create distance_leg constraint. Unknown domain type')
end

% distance_leg is defined as the Euclidean distance of 
% knee wrt stance toe
% p_pelvis = nlp.Plant.States.x(1:3);
[~,qLL] = Cassia_FK_LEG(qpitch,qknee);
expr = qLL;
distance_leg_fun = SymFunction(['distance_leg_',nlp.Plant.Name], expr, {nlp.Plant.States.x});
    
end

