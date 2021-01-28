function [X,Y,Z] = cone_wire_frame(state)

    addpath('./helper_functions')


    X = []; Y = []; Z = [];

    %---------------------------------------------------------------------%
    isSymbolic = false; %should be true 
    cone_params = cone_parameters(isSymbolic);
    
    m = double(cone_params.mass);
    g = double(cone_params.gravity);
    R = double(cone_params.radius);
    xCM = double(cone_params.lateral_CM_offset);
    zCM = double(cone_params.vertical_CM_offset);
    xA = double(cone_params.lateral_apex_offset);
    zA = double(cone_params.vertical_apex_offset);
    %---------------------------------------------------------------------%

N = 20;

slice = 2 * pi / N;

for i = 0:N
    rad = i * slice;
    x = R * cos(rad);
    y = R * sin(rad);
    z = 0;
    X = [X x];
    Y = [Y y];
    Z = [Z z];
end

Cx = xA;
Cy = 0;
Cz = zA;

X = [X Cx];
Y = [Y Cy];
Z = [Z Cz];

[rot_sb, rot_sbprime] = coordinate_frames([0 0 0 state(3) state(4) state(5)]);

cords = [X;Y;Z];
rot_cords = rot_sbprime * cords;

X = rot_cords(1,:);
Y = rot_cords(2,:);
Z = rot_cords(3,:);

end