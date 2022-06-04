function [base_link_vz_fun] = base_link_vz(nlp)
    expr = nlp.Plant.States.dx(3);
    
    base_link_vz_fun = SymFunction(['base_link_vz_',nlp.Plant.Name], expr, {nlp.Plant.States.dx});
end