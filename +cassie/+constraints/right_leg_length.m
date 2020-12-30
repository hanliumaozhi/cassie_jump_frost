function [leg_length_fun] = right_leg_length( nlp)
    qpitch = nlp.Plant.States.x(16);
    qknee = nlp.Plant.States.x(17);
    [~,qLL] = cassie.utils.Forward_Kinematics_p(qpitch,qknee);
    expr = qLL;
    leg_length_fun = SymFunction(['right_leg_length_',nlp.Plant.Name], expr, {nlp.Plant.States.x});
end