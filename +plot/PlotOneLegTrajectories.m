function PlotOneLegTrajectories(gait, indices)
    qAll = [gait(1).states.x, gait(3).states.x(:,2:end),];
    for i = 1:length(indices)
        if i > length(indices)-8
            qi = reshape(qAll(i, :), 1, size(qAll, 2));
            t = linspace(0, 0.8, size(qAll, 2));
            plot(t, qi);
            hold on;
        end        
        title(sprintf('Joint Trajectories'));
        xlabel('t');     
        ylabel('q');
    end
end
