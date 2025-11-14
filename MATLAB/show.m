function show(filename,indexes)
  %> @param indexes - indexes of circuits to show
  global PROJECT_DIR
  
  [~, name] = fileparts(filename);
  Parsed = sscanf(name,'PowerMonitor.v%1c.%02d.%02d.%[submain]');
  
  if numel(Parsed) ~= 6 && numel(Parsed) ~= 7
    error('File name seems to be in the wrong format, please check!')
  end
  
  run([PROJECT_DIR '\conf_', char(Parsed(4:end).'),'_v', Parsed(1), '.m'])
    
  [price, hour, Watts] = read_file(filename, conf);

  if exist('indexes','var') && numel(indexes) > 0
    Watts = Watts(:,indexes);
    conf.port = conf.port(indexes);
  end

  plot_data(price, hour, Watts, conf);
end