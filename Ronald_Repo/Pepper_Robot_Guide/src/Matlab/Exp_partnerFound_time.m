%% Plot Time Lapse 
    figure();
    t = [21.15 48.82 41.4 31.18 11.95 20.91 36.15 22.4 13.5 18.3 39.27 18.33 9.75];
    x = 1:1:size(t,2);
    m = mean(t)*ones(1,size(t,2))
    bar(t); hold on;
    plot(x,m,'r')
    xlabel('Participant');
    ylabel('Time (s)');
    title('Partner Found Event')
    x0=600; y0=100; width=500; height=150;
    set(gcf,'units','points','position',[x0,y0,width,height])