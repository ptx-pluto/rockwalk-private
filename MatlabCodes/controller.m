function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;

[u1,cmd_phi,cmd_theta] = calc_pos_cmd(s,s_des);

s_euler = calc_euler(s);
%des_euler = calc_euler(s_des);
des_euler = calc_cmd_euler(s_des,cmd_phi,cmd_theta);

s_euler_deri = calc_euler_deri(s);
des_euler_deri = calc_euler_deri(s_des);

err_euler = des_euler - s_euler;
err_euler_deri = des_euler_deri - s_euler_deri;

Kp = 200;
Kd = 120;

p_euler = diag([Kp Kp Kp]);
d_euler_deri = diag([Kd Kd Kd]);

v1 = p_euler*err_euler;
v2 = d_euler_deri*err_euler_deri';

pid = v1+v2;

omega = get_omega(s);

u2 = I * pid + cross(omega,I*omega);

%F = 4.0; M = [2, 0, 0]'; % You should calculate the output F and M
F = u1; M = u2; % You should calculate the output F and M

end

function rst = get_omega(s)
    rst = s(11:13);
end

function rst = get_quat(s)
    rst = s(7:10);
end

function rst = get_R(s)
    rst = QuatToRot(get_quat(s));
end

function rst = calc_euler(s)
    [phi,theta,psi] = RotToRPY_ZXY(get_R(s));
    rst = [phi;theta;psi];
end

function rst = calc_cmd_euler(s,cmd_phi,cmd_theta)
    [phi,theta,psi] = RotToRPY_ZXY(get_R(s));
    rst = [cmd_phi;cmd_theta;psi];
end

function yaw = calc_yaw(s)
    [phi,theta,psi] = RotToRPY_ZXY(get_R(s));
    yaw = psi;
end

function rst = calc_ang_mat(phi,theta,psi)
    rst =  [ 
        cos(theta) 0 -cos(phi)*sin(theta);
        0 1 sin(phi);
        sin(theta) 0 cos(phi)*cos(theta)
        ];
end

function rst = calc_euler_deri(s)
    omega = s(11:13);
    [phi,theta,psi] = RotToRPY_ZXY(get_R(s));
    ang_mat = calc_ang_mat(phi,theta,psi);
    rst = omega';
end

function [u1,cmd_psi,cmd_theta] = calc_pos_cmd(s,s_des)

    global params

    m = params.mass;
    g = params.grav;
    I = params.I;

    s_p = s(1:3);
    s_v = s(4:6);
    
    des_p = s_des(1:3);
    des_v = s_des(4:6);

    Kp = [5 4 8];
    Kd = [10 15 15];
    
    part1 = diag(Kp)*(des_p-s_p);
    part2 = diag(Kd)*(des_v-s_v);
    
    pid_pos = part1+part2;
    
    yaw = calc_yaw(s);
     
    u1 = m*(g+pid_pos(3));
    cmd_psi = (pid_pos(1)*sin(yaw)-pid_pos(2)*cos(yaw))/g;
    cmd_theta = (pid_pos(1)*cos(yaw)+pid_pos(2)*sin(yaw))/g;
    
end