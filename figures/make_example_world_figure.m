load('example_world_figure_data.mat')

%%
fh = figure(1) ; clf ; hold on ; axis equal ;
A.plot_view_style = 'none' ;

plot(W)
plot(A)
S.plot_at_time(15)

A.plot_data.trajectory.LineWidth = 3 ;
view(3)
% campos([-41.1739 -448.1342  236.0044])
% campos([40 -313 280])
campos([-17 -303 284])
set(gca,'FontSize',18)
set(fh,'Units','Inches')
pos = get(fh,'Position');
set(fh,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])