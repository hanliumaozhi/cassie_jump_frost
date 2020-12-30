function [pitch_momentum_fun] = pitch_momentum( nlp)
    
    plant = nlp.Plant;
    D = plant.Mmat;
    q = plant.States.x;
    dq = plant.States.dx;
    h_t = D(5,:)*dq;
    
    pitch_momentum_fun = SymFunction(['momentum_pitch_', plant.Name], h_t, {q,dq});
end