function show1(filename,indexes)
  %> SHOW1 differs from SHOW in that it works with the firmware in the
  %> sender which sends complex wattages, so instead of one value per circuit
  %> real/image pair comes
  %> @param indexes - indexes of circuits to show
  if strfind(filename,'sub.'), run conf_sub.m; else
    if strfind(filename,'main.'), run conf_main.m; else
      error('Filename should contain either "sub." or "main." substring');
    end
  end
  
  m = abs(csvread(filename,2)); % first string may be corrupted
  % csv file structure: first column is epoch 1970 second, following are real/image pairs
  % first pair is the volt channel, last one is ground
  hour = datenum(1970,1,1)*24 + m(:,1)/60/60 - 5; % last term is timezone
  % let's mark breaks > 10 min
  breaks = unique([1;find(AVP.diff(hour) > 1/6)+1;numel(hour)+1]);
  
  Watts = reshape(m(:,4:end),size(m,1),2,[]); % remove time mark and voltage channel
  Watts = squeeze(complex(Watts(:,1,:),Watts(:,2,:))); % and convert to complex
  Watts = Watts(:,1:end-1) - repmat(Watts(:,end),1,size(Watts,2)-1)*0; % ADC times ADC data, ground subtracted
  Watts = Watts/conf.coeff; % ADC times ADC to "rms volt * rms volt" conversion factor
  Watts = Watts./repmat([conf.port(:).coeff],size(Watts,1),1)*1000; % take into account transformers sensitivity 
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
  
  price = abs(kWh)/1536*335/Hrs*30*24; % at current prices for one month
  
  hour(breaks(2:end-1)) = NaN;
  
  subplot(2,1,1)
  plot(hour/24,real(Watts))
  datetick('x','dd-HH:MM')
  % set(gca,'XTickMode','auto')
  xlabel('Day-Hour')
  ylabel('angle(Watts)')
  % AVP.PLOT.legend(cellstr([num2str([1:14;price].')]))  
  [ax,objs,ploth,texth] = AVP.PLOT.legend(strcat(cellstr(num2str([1:numel(price);price].',...
    '%2i %3.0f <')), {conf.port(:).name}.'),'Location','Best');
  [objs(1:numel(objs)/3).FontName] = deal('Monospaced');
  subplot(2,1,2)
  plot(hour/24,angle(Watts))
  datetick('x','dd-HH:MM')
  % set(gca,'XTickMode','auto')
  xlabel('Day-Hour')
  ylabel('phase(Watts)')  
end