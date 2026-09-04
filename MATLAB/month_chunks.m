function files = month_chunks(pattern)
  %> All log files holding one month's data, as full paths, oldest first.
  %> @param pattern - a filename or glob for the bare .csv, e.g.
  %>        'PowerMonitor.v*.08.26.main.csv'
  %
  % A power log is one file per month: bsd's http_server exempts a filename that
  % carries the month from size rotation, precisely so a month is never split.
  % This still collects siblings, because a log that is NOT exempt is rotated to
  % <name>.<YYYYmmdd-HHMMSS>, and because a month split by the old rotation would
  % otherwise be read as just its tail with nothing to say so. Ordering here is
  % approximate (bare .csv last); read_file sorts rows by timestamp.

  [pdir, pname, pext] = fileparts(char(pattern));
  d = dir(fullfile(pdir, [pname pext '*']));
  d = d(~[d.isdir]);
  if isempty(d), files = strings(0,1); return; end

  paths = fullfile(string({d.folder}.'), string({d.name}.'));
  isBare = endsWith(paths, '.csv');
  files = [sort(paths(~isBare)); paths(isBare)];
end
