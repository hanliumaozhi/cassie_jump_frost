function [toe_x_eq_fun ] = toe_x_eq(nlp)

H_WL = nlp.Plant.ContactPoints.LeftSole.computeForwardKinematics;
p_WL = H_WL(1:3,end);
H_WR = nlp.Plant.ContactPoints.RightSole.computeForwardKinematics;
p_WR = H_WR(1:3,end);
expr = p_WL(1) - p_WR(1);

toe_x_eq_fun = SymFunction(['toe_x_eq_', nlp.Plant.Name], expr, {nlp.Plant.States.x});

end