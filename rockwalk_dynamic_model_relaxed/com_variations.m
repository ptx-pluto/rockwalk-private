clear all;

cone_params = cone_parameters(false);

offsets = -0.6:0.05:0.6;

N = size(offsets,2);

results = [];

parfor i = 1:N
    delta = offsets(i);
    c = cone_move_com_x(cone_params, delta);
    [t,y] = solve_fixed_apex_dynamics_param(c);
    results(i).t = t;
    results(i).y = y;
end

save('results');

plot_full_state_multi(results);

function rst = cone_move_com_x(cone, ratio)

    G_x = cone.radius;
    G_z = 0;
    
    G = [G_x G_z];
    
    com_x = cone.lateral_CM_offset;
    com_z = cone.vertical_CM_offset;

    CoM = [com_x com_z];
    
    dir = G - CoM;
    
    new_CoM = CoM + ratio*dir;
    
    rst = cone;
    rst.lateral_CM_offset = new_CoM(1);
    rst.vertical_CM_offset = new_CoM(2);
    
end

