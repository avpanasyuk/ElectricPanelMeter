function plot_data(price, hour, Watts, conf)
  plot(hour/24,Watts)
  datetick('x','dd-HH:MM')
  % set(gca,'XTickMode','auto')
  xlabel('Day-Hour')
  ylabel('Watts')
  % AVP.PLOT.legend(cellstr([num2str([1:14;price].')]))  
  [ax,objs,ploth,texth] = AVP.PLOT.legend(strcat(cellstr(num2str([1:numel(price);price].','%2i %3.0f <')), {conf.port(:).name}.'),'Location','Best');
  [objs(1:numel(objs)/3).FontName] = deal('Monospaced');
end