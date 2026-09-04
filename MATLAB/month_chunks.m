function files = month_chunks(pattern)
  %> All log files holding one month's data, as full paths.
  %> @param pattern - a filename or glob for the bare .csv, e.g.
  %>        'PowerMonitor.v*.08.26.main.csv'
  %
  % bsd's http_server rotates a log to <name>.csv.<N> once it passes its size
  % cap, so one month is spread over several files: the numbered chunks hold the
  % EARLIER data and the bare .csv holds the most recent. Globbing only the bare
  % .csv yields the tail of the month with nothing to indicate the rest exists.
  % Order here is approximate (bare .csv last); read_file sorts rows by timestamp,
  % so a suffix scheme this does not understand still comes out chronological.

  d = dir([char(pattern) '*']);
  d = d(~[d.isdir]);
  if isempty(d), files = strings(0,1); return; end

  names = string({d.name}.');
  isBare = endsWith(names, '.csv');
  names = [sort(names(~isBare)); names(isBare)];
  files = fullfile(string(d(1).folder), names);
end
