%% display results
m = csvread('q:\test.csv',2);
serial_date = datenum(1970,1,1) + m(:,1)/60/60/24 - 5/24; % last term is timezone

% let's mark breaks > 10 min
breaks = unique([1;find(AVP.diff(m(:,1)) > 600)+1;size(m,1)+1]);

kWh = 0; Hrs = 0;
for brI=1:numel(breaks)-1
  brInds = [breaks(brI):breaks(brI+1)-1];
  kWh = kWh + trapz(m(brInds,1),abs(m(brInds,2:end)))/60/60/1000;
  Hrs = Hrs + (m(brInds(end),1) - m(brInds(1),1))/60/60;
end

price = kWh/2129*443/Hrs*30*24; % at current prices

serial_date(breaks(2:end-1)) = NaN;

plot(serial_date,abs(m(:,2:end)))
datetick('x','dd HH')
xlabel('Day Hour')
AVP.PLOT.legend(cellstr([num2str([1:12;price].')]))


%% plot(m(:,1)-m(1,1),abs(m(:,2:end)))

