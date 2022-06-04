function [ leg_velo_fun ] = leg_velocity( nlp )
% velocity min
%   Creates SymFunction for a pitch cost
%
%   Author: Ross Hartley
%     Date: 2018-03-19

cost = 0;
q_velocity = nlp.Plant.States.dx(7:end);
for j=1:14
        cost = cost + q_velocity(j)*q_velocity(j);
end
leg_velo_fun = SymFunction('leg_velocity', cost, {nlp.Plant.States.dx});
end