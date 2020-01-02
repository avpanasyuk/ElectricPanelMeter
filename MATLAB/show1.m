function show1(IsSub,month,year,indexes)
  %> @param indexes - indexes of circuits to show
  global PROJECT_DIR
  cd('R:\ARCHIVE\POWER');
  if IsSub, PanelID = 'sub'; else PanelID = 'main'; end
  FileNameTemp = sprintf(['PowerMonitor.v*.%02d.%02d.' PanelID '.csv'],month,year);
  f = dir(FileNameTemp);
  if isempty(f), error('Can not find the record!'); end
  Ver = sscanf(f.name,'PowerMonitor.v%1c.');
  
  run([PROJECT_DIR '\conf_', PanelID ,'_v', Ver, '.m'])
  
  [price, hour, Watts] = read_file(f.name, conf);
  plot_data(price, hour, Watts, conf);
end