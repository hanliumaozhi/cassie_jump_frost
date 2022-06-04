function [ pitch_moment_fun ] = pitch_moment( nlp )
pcom = nlp.Plant.getComPosition();
Ag = zeros(6,nlp.Plant.numState);
n_link = length(nlp.Plant.Links);
Jb = cell(nlp.Plant.numState,1);
R = cell(nlp.Plant.numState,1);
p = cell(nlp.Plant.numState,1);
I = cell(nlp.Plant.numState,1);
Xg = cell(nlp.Plant.numState,1);
A = cell(nlp.Plant.numState,1);
for i=1:n_link
    frame = nlp.Plant.Links(i);
    Jb{i} = getBodyJacobian(nlp.Plant,frame);
    gst = computeForwardKinematics(frame);
    R{i} = frame.RigidOrientation(gst);
    p{i} = frame.RigidPosition(gst);
    I{i} = blkdiag(frame.Mass*eye(3),frame.Inertia);
    tmp_v = p{i}-pcom';
    tmp_m = [ 0, -tmp_v(3),  tmp_v(2);
                tmp_v(3),     0, -tmp_v(1);
                -tmp_v(2),  tmp_v(1),     0];
    Xg{i} = [ R{i}', -R{i}'*tmp_m; zeros(3), R{i}' ];
    A{i} =  Xg{i}'*I{i}*Jb{i};
    Ag = Ag + A{i};
end
dAg = jacobian(Ag* nlp.Plant.States.dx, nlp.Plant.States.x);
cost_all = dAg*nlp.Plant.States.dx + Ag*nlp.Plant.States.ddx;
cost = 1000*cost_all(5)*cost_all(5);
pitch_moment_fun = SymFunction('pitch_moment', cost, {nlp.Plant.States.x, nlp.Plant.States.dx, nlp.Plant.States.ddx});
end