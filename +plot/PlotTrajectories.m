function PlotTrajectories(qAll, vAll, indices)
    for i = 1:length(indices)
        f = figure;
        ax = axes(f); %#ok<LAXES>
        hold on;
        
        qi = reshape(qAll(i, :), 1, size(qAll, 2));
        t = linspace(0, 1, size(qAll, 2));
        v = vAll(i).*ones(1, size(qAll, 2));
        %plot3(ax, t, v, qi);
        plot(ax, t, qi);
        title(sprintf('Joint %s', cell2mat(indices(1,i))));
        xlabel('t');
        ylabel('q');
%         view(ax, 3);
%         
%         title(sprintf('Joint %s', cell2mat(indices(1,i))));
%         xlabel('t');
%         ylabel('v');
%         zlabel('q');
%         f.Name = ax.Title.String;
    end
end
