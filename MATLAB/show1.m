function show1(IsSub,month,year,indexes)
  %> @param indexes - indexes of circuits to show
  global PROJECT_DIR
  % bsd's http_server writes every ESP log CSV straight into this share, so
  % the in-progress current month is readable here alongside the finished
  % ones. There is no second location and no monthly move.
  cd('//bsd/ARCHIVE/ESP_LOGS/')
  if IsSub, PanelID = 'sub'; else PanelID = 'main'; end
  FileNameTemp = sprintf(['PowerMonitor.v*.%02d.%02d.' PanelID '.csv'],month,year);
  f = dir(FileNameTemp);
  if isempty(f), error('Can not find the record!'); end
  Ver = sscanf(f.name,'PowerMonitor.v%1c.');
  
  run([PROJECT_DIR '\conf_', PanelID ,'_v', Ver, '.m'])
  
  [price, hour, Watts] = read_file(f.name, conf);
  
  if exist('indexes','var') && numel(indexes) > 0
    Watts = Watts(:,indexes);
    conf.port = conf.port(indexes);
    price = price(indexes);
  end

  plot_data(price, hour, Watts, conf);
end