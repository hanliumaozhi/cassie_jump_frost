function [ velocity_fun ] = velocity( nlp )
%pitch cost for pitch
%   Creates SymFunction for a pitch cost
%
%   Author: Ross Hartley
%     Date: 2018-03-19

% Compute function for pitch cost
w = 100;
q_velocity = nlp.Plant.States.dx(1);
cost = 0;
for i=1:length(q_velocity)
    cost = cost + w*((q_velocity(i))*(q_velocity(i)));
end
velocity_fun = SymFunction('rv', cost, {nlp.Plant.States.dx});

end