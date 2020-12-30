function PlotOneLegTrajectoriesPhase(gait, indices)
    qAll = [gait(1).states.x, gait(3).states.x(:,2:end)];
    vAll = [gait(1).states.dx, gait(3).states.dx(:,2:end)];
    for i = 1:length(indices)
        if i > length(indices)-8
            qi = reshape(qAll(i, :), 1, size(qAll, 2));
            vi = reshape(vAll(i, :), 1, size(qAll, 2));
            plot(qi, vi);
            hold on;
        end        
        title(sprintf('Joint Trajectories'));
        xlabel('q');     
        ylabel('v');
    end
end
