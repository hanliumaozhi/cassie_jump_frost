function [base_link_z_fun] = base_link_z(nlp)
    expr = nlp.Plant.States.x(3);
    
    base_link_z_fun = SymFunction(['base_link_z_',nlp.Plant.Name], expr, {nlp.Plant.States.x});
end