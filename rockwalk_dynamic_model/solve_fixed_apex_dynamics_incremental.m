clear all
close all

FPS = 30;
Duration = 5;
N = round(Duration*FPS);

init_psi = 0;
init_theta = deg2rad(10);
init_phi = deg2rad(40);
init_phi_dot = 0;

xinit = init_state(init_psi, init_theta, init_phi, init_phi_dot);

ctime = 0.02;
T = 0;

all_t = [];
all_y = [];

state = xinit;

while T < 5
   
    [vt,vy] = ode45(@(t,x)eom_rnw_symbolic(x), [T T+ctime], state);

    all_t = [all_t;vt];
    all_y = [all_y;vy];
    
    T = T+ctime;
    state = vy(end,:)';
    
end

plots(all_t,all_y);


function plots(t,y)


    subplot(3,2,1)
    plot(t,y(:,1))
    % hold on
    % plot(t, y(:,6))
    title('x')

    subplot(3,2,2)
    plot(t,y(:,2))
    % hold on
    % plot(t, y(:,7))
    title('y')


    subplot(3,2,3)
    plot(t,y(:,3))
    % hold on
    % plot(t, y(:,8))
    title('\psi')

    subplot(3,2,4)
    plot(t,y(:,4))
    % hold on
    % plot(t, y(:,9))
    title('\theta')

    subplot(3,2,5)
    plot(t,y(:,5))
    % hold on
    % plot(t, y(:,10))
    title('\phi')

    [ts,ps] = find_peaks(t,y);
    
    subplot(3,2,6)
    plot(t,y(:,5))
    hold on
    plot(ts,ps,'o');
    % plot(t, y(:,10))
    title('\phi')

    
end


