function [motor_velocity_fun] = motor_velocity(nlp, joint_index)
    expr = nlp.Plant.States.dx(joint_index);
    motor_velocity_fun = SymFunction(['motor_velocity_', int2str(joint_index), nlp.Plant.Name], expr, {nlp.Plant.States.dx});
end