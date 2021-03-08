function plot_full_state(states)

    N = size(states,2);

    for i = 1:N
        
        s = states(i);
        
        ratio = i/N;
        colr = [1 1 1] - ratio * [0 1 1];
        %colr = [i/N 1 1];
        
        t = s.t;
        y = s.y;
        
        
        subplot(3,2,1)
        hold on;
        plot(t,y(:,1),'Color',colr)
        title('x')

        subplot(3,2,2)
        hold on;
        plot(t,y(:,2),'Color',colr)
        title('y')


        subplot(3,2,3)
        hold on;
        plot(t,y(:,3),'Color',colr)
        title('\psi')

        subplot(3,2,4)
        hold on;
        plot(t,y(:,4),'Color',colr)
        title('\theta')

        subplot(3,2,5)
        hold on;
        plot(t,y(:,5),'Color',colr)
        title('\phi')

        [ts,ps] = find_peaks(t,y);

        subplot(3,2,6)
        hold on;
        plot(t,y(:,5))
        plot(ts,ps,'o');
        title('\phi')

        disp('period:');
        disp(ts(3));
        
    end

   
    
end
