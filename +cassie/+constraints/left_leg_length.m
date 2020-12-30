function [leg_length_fun] = left_leg_length( nlp)
    qpitch = nlp.Plant.States.x(9);
    qknee = nlp.Plant.States.x(10);
    [~,qLL] = cassie.utils.Forward_Kinematics_p(qpitch,qknee);
    expr = qLL;
    leg_length_fun = SymFunction(['left_leg_length_', nlp.Plant.Name], expr, {nlp.Plant.States.x});
end