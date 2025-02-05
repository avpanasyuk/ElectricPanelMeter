function [price, hour, Watts] = read_file(filename, conf)
  %m = abs(csvread(filename,2)); % first string may be corrupted
  % !!!!!!SOMETIMES THERE IS A SPARE CR WHICH BREAKS csvread

  fid = fopen(filename);
  x = sscanf(fgets(fid),'%g%c',[2 Inf]); % reading the first line which may be corrupted, counting entries
  NumVars = size(x,2);
  A = [];
  while ~feof(fid)
    A = [A, fscanf(fid,'%g%c',[2 Inf])];
    fgets(fid);
  end
  Nlines = fix(size(A,2)/NumVars);

  m = abs(reshape(A(1,1:Nlines*NumVars),NumVars,[]).');
  fclose(fid);

  % csv file structure: first column is epoch 1970 second, second is volt
  % channel, last one is ground
  hour = datenum(1970,1,1)*24 + m(:,1)/60/60 - 5; % last term is timezone
  % let's mark breaks > 10 min
  breaks = unique([1;find(AVP.diff(hour) > 1/6)+1;numel(hour)+1]);

  Watts = max(m(:,3:end-1) - repmat(m(:,end),1,size(m,2)-3),0); % ADC times ADC data, ground subtracted
  Watts = Watts/conf.coeff; % ADC times ADC to "rms volt * rms volt" conversion factor
  Watts = Watts./repmat([conf.port(:).coeff],size(m,1),1)*1000; % take into account transformers sensitivity
  % and convert "rms volt * rms volt" to "rms volt * rms amp = Watt"

  if exist('indexes','var') && numel(indexes) > 0
    Watts = Watts(:,indexes);
    conf.port = conf.port(indexes);
  end

  kWh = 0; Hrs = 0;
  for brI=1:numel(breaks)-1
    brInds = [breaks(brI):breaks(brI+1)-1];
    if numel(brInds) > 1 % trapz does not work otherwise
      kWh = kWh + trapz(hour(brInds),Watts(brInds,:))/1000;
      Hrs = Hrs + (hour(brInds(end),1) - hour(brInds(1),1));
    end
  end

  price = kWh/1536*335/Hrs*30*24; % at current prices for one month

  hour(breaks(2:end-1)) = NaN;
end