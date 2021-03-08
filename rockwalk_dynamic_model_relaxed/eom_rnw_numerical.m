function dxdt = eom_rnw_numerical(x)

    %load FixedApexFunctions.mat

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

    
    q1 = x(1); 
    q2 = x(2); 
    q4 = x(3);
    q5 = x(4);
    q6 = x(5);

    dqdt1 = x(6);
    dqdt2 = x(7);
    dqdt4 = x(8);
    dqdt5 = x(9);
    dqdt6 = x(10);

    mat_l = left_augmented_matrix(R,m,q4,q5,q6,xA,xCM,zA,zCM);
    mat_r = right_augmented_matrix(R,dqdt4,dqdt5,dqdt6,g,m,q4,q5,q6,xA,xCM,zA,zCM);
    
    sol_vector = inv(mat_l)*mat_r;
    
    ddqdt1 = sol_vector(1);
    ddqdt2 = sol_vector(2);
    ddqdt4 = sol_vector(3);
    ddqdt5 = sol_vector(4);
    ddqdt6 = sol_vector(5);

    dxdt = [dqdt1;dqdt2;dqdt4;dqdt5;dqdt6;...
            ddqdt1;ddqdt2;ddqdt4;ddqdt5;ddqdt6];
end