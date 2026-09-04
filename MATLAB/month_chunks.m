function files = month_chunks(pattern)
  %> All log files holding one month's data, as full paths, oldest first.
  %> @param pattern - a filename or glob for the bare .csv, e.g.
  %>        'PowerMonitor.v*.08.26.main.csv'
  %
  % A month is spread over several files:
  %  - RECOVERED/<name>   rows restored from ZFS snapshots, absent from the sink
  %  - <name>.<suffix>    chunks the sink rotated aside, earliest first
  %  - <name>             the live file, holding the most recent rows
  % Globbing only the bare .csv yields the tail of the month with nothing to
  % indicate the rest exists. Ordering here is approximate; read_file sorts rows
  % by timestamp, so a suffix scheme this does not understand still comes out
  % chronological.

  [pdir, pname, pext] = fileparts(char(pattern));
  glob = [pname pext '*'];

  d = dir(fullfile(pdir, glob));
  r = dir(fullfile(pdir, 'RECOVERED', glob));
  d = [r(:); d(:)];
  d = d(~[d.isdir]);
  if isempty(d), files = strings(0,1); return; end

  paths = fullfile(string({d.folder}.'), string({d.name}.'));
  isBare = endsWith(paths, '.csv') & ~contains(paths, [filesep 'RECOVERED' filesep]);
  files = [paths(~isBare); paths(isBare)];
end
