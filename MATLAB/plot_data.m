function plot_data(price, hour, Watts, conf)
  t = datetime(hour/24,'ConvertFrom','datenum');
  plot(t,Watts)
  xlabel('Day-Hour')
  ylabel('Watts')
  % AVP.PLOT.legend(cellstr([num2str([1:14;price].')]))  
  [ax,objs,ploth,texth] = AVP.PLOT.legend(strcat(cellstr(num2str([1:numel(price);price].','%2i %3.0f <')), {conf.port(:).name}.'),'Location','Best');
  [objs(1:numel(objs)/3).FontName] = deal('Monospaced');
end