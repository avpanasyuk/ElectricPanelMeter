function month_by_month(PanelID)
  % plot all data month by month
  global PROJECT_DIR
  
  cd('R:\ARCHIVE\POWER');
  FileNameTemp = ['PowerMonitor.v*.*.*.' PanelID '.csv'];
  f = dir(FileNameTemp);
  for fI = 1:numel(f)
    fn = f(fI).name
    Parsed = sscanf(fn,'PowerMonitor.v%1c.%02d.%02d.%[submain]');
    if numel(Parsed) ~= 6 && numel(Parsed) ~= 7
      error('File name seems to be in the wrong format, please check!')
    end
    run([PROJECT_DIR '\conf_', char(Parsed(4:end).'),'_v', Parsed(1), '.m'])
    
    price(:,fI) = read_file(fn, conf);
    month(fI) = Parsed(2) + 12*Parsed(3);
  end
  [month_sorted, sI] = sort(month);
  plot(month_sorted,price(:,sI),'+-');
  AVP.PLOT.legend({conf.port(:).name}.','Location','Best');
  ax = gca;
  m_str = strsplit(num2str(mod(ax.XTick,12)));
  y_str = strsplit(num2str(fix(ax.XTick/12)));
  ax.XTickLabel = strcat(m_str','\',y_str');
  ylabel('Dolalrs per month')
  xlabel('Date MM\YR')
end
