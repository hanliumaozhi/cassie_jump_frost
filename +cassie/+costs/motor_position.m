function [motor_position_fun] = motor_position(nlp, X0)

cost = 0;
q_current = nlp.Plant.States.x;

cost = cost + (X0(7)-q_current(7))*(X0(7)-q_current(7));
cost = cost + (X0(8)-q_current(8))*(X0(8)-q_current(8));
cost = cost + (X0(9)-q_current(9))*(X0(9)-q_current(9));
cost = cost + (X0(10)-q_current(10))*(X0(10)-q_current(10));
cost = cost + (X0(13)-q_current(13))*(X0(13)-q_current(13));
cost = cost + (X0(14)-q_current(14))*(X0(14)-q_current(14));
cost = cost + (X0(15)-q_current(15))*(X0(15)-q_current(15));
cost = cost + (X0(16)-q_current(16))*(X0(16)-q_current(16));
cost = cost + (X0(17)-q_current(17))*(X0(17)-q_current(17));
cost = cost + (X0(20)-q_current(20))*(X0(20)-q_current(20));
cost = 10000*cost;

motor_position_fun = SymFunction('motor_position', cost, {nlp.Plant.States.x, X0});

end