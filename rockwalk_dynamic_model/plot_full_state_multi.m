function plot_full_state(states)

    for s = states
        t = s.t;
        y = s.y;
        
        
        subplot(3,2,1)
        hold on;
        plot(t,y(:,1))
        % hold on
        % plot(t, y(:,6))
        title('x')

        subplot(3,2,2)
        hold on;
        plot(t,y(:,2))
        % hold on
        % plot(t, y(:,7))
        title('y')


        subplot(3,2,3)
        hold on;
        plot(t,y(:,3))
        % hold on
        % plot(t, y(:,8))
        title('\psi')

        subplot(3,2,4)
        hold on;
        plot(t,y(:,4))
        % hold on
        % plot(t, y(:,9))
        title('\theta')

        subplot(3,2,5)
        hold on;
        plot(t,y(:,5))
        % hold on
        % plot(t, y(:,10))
        title('\phi')

        [ts,ps] = find_peaks(t,y);

        subplot(3,2,6)
        hold on;
        plot(t,y(:,5))
        plot(ts,ps,'o');
        % plot(t, y(:,10))
        title('\phi')

        disp('period:');
        disp(ts(3));
        
    end

   
    
end
