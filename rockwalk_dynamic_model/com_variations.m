clear all;

cone_params = cone_parameters(false);

offsets = [-0.01 0 0.01];

N = size(offsets,2);

results = [];

parfor i = 1:N
    c = cone_params;
    delta = offsets(i);
    c.vertical_CM_offset = c.vertical_CM_offset + delta;
    [t,y] = solve_fixed_apex_dynamics_param(c);
    results(i).t = t;
    results(i).y = y;
end