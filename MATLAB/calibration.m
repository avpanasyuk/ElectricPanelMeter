% in new calibration we do not apply calibration in firmware, we store it
% in calibration file and apply it in MATLAB/show code
% Now we can calibrate by the port 0 data which is Voltage*Voltage.
% So, we have following voltage dividers:
% switch to ADC - Rsw
% outlet to switch - Rout
% we can not get both from one measurement. We can get (Rsw*Rout)^2
% What we need is Rvv = Rsw^2*Rout. So we need another measurement, say at 
% U2-3. 
%%  Measuring the board V2
% Ok, we see outlet voltage=122.6 and U2-3 voltage=1.922. So, 
Rout = 122.6/1.922
% We have Power at port 0 = 91400 while outlet voltage=124. It means
% (Rsw*Rout)^2 = 124^2/91400, so
Rsw = 124/sqrt(91400)/Rout % = 0.00643
% we should note that it is quite a bit off LTspice
% value of 0.147/0.0242/1024 = 0.005932, which would produce even bigger
% error

% The coefficient we need
Rvv = Rsw^2*Rout % 0.0026373

% we have to multiply it by sensor ratio

14000*Rvv*20 = 740. % Should be 780. hmm?



%% OLD CALIBRATION
c = [[-432 0 -444 0 -421 0  6451 313  6467 312  6440 310 22160 1010 22060 1013 22060 1011 15880 731 15828 732 15827 732];...
[16048 732 16108 731 16111 730 22450 1016 22424 1022 22390 1014 6727 311 6701 311 6705 310 -143 0 -172 0 -191 0];...
[127 0 130 0 85 0 7213 310 7006 308 6979 307 22535 1000 22336 1010 22387 1001 16059 721 16210 723 16153 722];...
[16344 725 16693 724 16614 723 23021 1000 22931 999 22872 999 7345 308 7290 307 7320 306 370 0 435 0 431 0];...
[650 0 738 0 700 0 -6225 307 -6128 305 -6164 305 -21527 999 -21599 1000 -21632 1009 -15881 721 -15454 722 -15436 721];...
[17056 721 17129 723 17194 724 23374 998 23480 1000 23505 1000 7827 306 7847 306 7789 306 1291 0 984 0 990 0];...
[1409 0 1169 0 1205 0 8305 309 8116 307 8167 306 23533 997 23734 1008 23709 996 17815 720 17469 718 17487 722];...
[17793 720 17596 720 17616 721 24120 1003 23940 1001 23910 1001 8206 307 8248 306 8265 307 1267 0 1293 0 1333 0];...
[1426 0 1331 0 1329 0 8370 307 8247 307 8317 307 23936 999 23897 998 23923 999 17439 719 17609 721 17535 719];...
[-15180 718 -15266 719 -15236 719 -21330 997 -21598 996 -21622 994 -5591 307 -5765 306 -5795 308 1207 0 1220 0 1193 0];...
[1028 0 992 0 1034 0 -5955 307 -5949 306 -5968 306 -21911 999 -21724 996 -21725 998 -15592 731 -15637 729 -15652 728];...
[-15740 729 -15767 729 -15817 723 -22108 1004 -21943 1004 -21983 998 -6113 308 -6208 307 -6207 307 1164 0 706 0 783 0]]

c1 = reshape(c,[],2,12)
a = [];
b = [];
for portI=1:size(c,1)
  c2(portI,:) = AVP.linear(c1(portI,2,:),c1(portI,1,:));
end


x = [310 6387 311 6424 312 6430 311 6348 1020 21926 1016 22000 1015 22100 735 15823 735 15812 732 15887]
x1 = reshape(x,2,[]).'
[a,b] = AVP.linear_fit(x1(:,2),x1(:,1))