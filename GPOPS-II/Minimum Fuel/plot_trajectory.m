% plot_trajectory.m

function plot_trajectory(solution,params)

    % Unpack solution
    t = solution.phase.time;
    
    x = solution.phase.state(:,1);
    y = solution.phase.state(:,2);
    h = solution.phase.state(:,5);
    
    f1 = 10*solution.phase.control(:,1);
    f2 = 10*solution.phase.control(:,2);
    f3 = 10*solution.phase.control(:,3);
    f4 = 10*solution.phase.control(:,4);
    
    % Unpack parameters
    l = 4*params.l;
    
    % Define trajectory plot intensity
    int = 0.6;
    
    % Initialize figure
    figure
    hold on
    ax = gca;
    ax.FontSize = 20;
    ax.LineWidth = 1.5;
    xlabel('X Position','FontSize',24)
    ylabel('Y Position','FontSize',24)
    plot(x,y,'Color',[int int int],'LineWidth',2)
    grid on
    axis equal
    
    waitforbuttonpress
    
    % Animate spacecraft
    for n = 1:length(x)

        % Define spacecraft corners in spacecraft heading frame
        corners = [[l/2;l/2];...
                   [-l/2;l/2];...
                   [-l/2;-l/2];...
                   [l/2;-l/2]];

        % Define thrusters in spacecraft heading frame
        thrusters = [[l/2;l/2+f1(n)];...
                     [-l/2;l/2+f2(n)];...
                     [-l/2;-l/2-f3(n)];...
                     [l/2;-l/2-f4(n)]];

        % Rotate corners by the given heading
        R = [cos(h(n)) -sin(h(n));...
             sin(h(n))  cos(h(n))];

        for i = 1:4
            corners(2*(i-1)+1:2*i) = R*corners(2*(i-1)+1:2*i);
        end

        % Rotate thrusters by given heading
        for i = 1:4
            thrusters(2*(i-1)+1:2*i) = R*thrusters(2*(i-1)+1:2*i);
        end

        % Plot thrust
        t1 = plot(x(n)+[corners(1) thrusters(1)],y(n)+[corners(2) thrusters(2)],'r-','LineWidth',4);
        t2 = plot(x(n)+[corners(3) thrusters(3)],y(n)+[corners(4) thrusters(4)],'r-','LineWidth',4);
        t3 = plot(x(n)+[corners(5) thrusters(5)],y(n)+[corners(6) thrusters(6)],'r-','LineWidth',4);
        t4 = plot(x(n)+[corners(7) thrusters(7)],y(n)+[corners(8) thrusters(8)],'r-','LineWidth',4);

        % Plot spacecraft
        s1 = plot(x(n)+[corners(1) corners(3)],y(n)+[corners(2) corners(4)],'k-','LineWidth',5);
        s2 = plot(x(n)+[corners(3) corners(5)],y(n)+[corners(4) corners(6)],'k-','LineWidth',2);
        s3 = plot(x(n)+[corners(5) corners(7)],y(n)+[corners(6) corners(8)],'k-','LineWidth',5);
        s4 = plot(x(n)+[corners(7) corners(1)],y(n)+[corners(8) corners(2)],'k-','LineWidth',2);
        
        % Delay and delete plots
        if n < length(x)
            
            pause((t(n+1)-t(n)))
            
            delete(t1)
            delete(t2)
            delete(t3)
            delete(t4)
            delete(s1)
            delete(s2)
            delete(s3)
            delete(s4)
        end
        
    end

end