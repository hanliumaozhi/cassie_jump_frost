function [ leg_acc_fun ] = leg_acc( nlp )
%pitch cost for pitch
%   Creates SymFunction for a pitch cost
%
%   Author: Ross Hartley
%     Date: 2018-03-19

% Compute function for pitch cost
cost = 0;
q_acc = nlp.Plant.States.ddx(3);
for i=1:length(q_acc)
    cost = cost + q_acc(i)*q_acc(i);
end
leg_acc_fun = SymFunction('leg_acc', cost, {nlp.Plant.States.ddx});

end