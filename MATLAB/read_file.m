function [price, hour, Watts] = read_file(filename, conf)
  % CSV layout: column 1 is a timestamp, the rest are ADC values (col 2 = the
  % shared voltage channel, last column = ground reference). The timestamp
  % column appears in two formats, which may be mixed within one file:
  %   legacy : Unix epoch seconds as a float             e.g. 1518000000.00
  %   current: local wall-clock 'yyyy-MM-dd HH:mm:ss.SS' (bsd http_server.py)
  % Both are normalized to epoch seconds here, then to local hours below.

  lines = strip(readlines(filename));
  lines = lines(strlength(lines) > 0); % drop blank/trailing lines

  % Keep only well-formed rows (the dominant column count); guards against a
  % corrupted first line or a stray CR that splits a row.
  ncommas = count(lines, ',');
  NumVars = mode(ncommas) + 1;
  lines = lines(ncommas == NumVars - 1);
  parts = split(lines, ','); % N x NumVars string array

  % Column 1 -> epoch seconds. The human-readable format contains '-' (date
  % separators); legacy epoch floats don't, so split on that and parse each
  % group its own way. (datetime() errors, rather than returning NaT, when no
  % element matches its InputFormat, so it can't be applied to the epoch rows.)
  % The local zone is bsd's (America/New_York), so posixtime() round-trips the
  % human-readable rows to the same epoch the conversion below maps back from.
  ts = parts(:,1);
  human = contains(ts, '-');
  epoch = nan(numel(ts), 1);
  epoch(~human) = str2double(ts(~human));
  if any(human)
    t = datetime(ts(human), 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SS', ...
                 'TimeZone', 'America/New_York');
    epoch(human) = posixtime(t);
  end

  m = [epoch, abs(double(parts(:, 2:end)))]; % ADC cols: abs; unparseable -> NaN
  m = m(~any(isnan(m), 2), :); % drop rows with any bad field

  % Convert epoch seconds (col 1) to local wall-clock time, DST-aware: the
  % America/New_York zone applies the EST(-5)/EDT(-4) switch automatically.
  % Drop the zone to a naive datenum*24 so the downstream diff/trapz/plot all
  % see correct local hours (a fixed -5 was an hour off for the EDT half-year).
  tLocal = datetime(m(:,1), 'ConvertFrom', 'posixtime', 'TimeZone', 'America/New_York');
  tLocal.TimeZone = '';
  hour = datenum(tLocal) * 24;
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