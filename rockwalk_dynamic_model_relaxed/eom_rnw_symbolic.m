function dxdt = eom_rnw_symbolic(x)

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

    vddqdt1 = ddqdt1(R,dqdt4,dqdt5,dqdt6,g,m,q4,q5,q6,xA,xCM,zA,zCM);
    vddqdt2 = ddqdt2(R,dqdt4,dqdt5,dqdt6,g,m,q4,q5,q6,xA,xCM,zA,zCM);
    vddqdt4 = ddqdt4(R,dqdt4,dqdt5,dqdt6,g,m,q4,q5,q6,xA,xCM,zA,zCM);
    vddqdt5 = ddqdt5(R,dqdt4,dqdt5,dqdt6,g,m,q4,q5,q6,xA,xCM,zA,zCM);
    vddqdt6 = ddqdt6(R,dqdt4,dqdt5,dqdt6,g,m,q4,q5,q6,xA,xCM,zA,zCM);
    

    dxdt = [dqdt1;dqdt2;dqdt4;dqdt5;dqdt6;...
            vddqdt1;vddqdt2;vddqdt4;vddqdt5;vddqdt6];
end