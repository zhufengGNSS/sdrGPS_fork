function [] = disp_tracking(filename);
close all;

% load tracking data
sv_tracking_data = load(filename);

[cr, cn ] = size(sv_tracking_data);
time_idx  = [1:cr];
%file format:
% colomn 1 -----column 11
% column 1: ms_count
% column 2: I_arm output
% column 3: Q_arm_output
% column 4: instantaneous_theta
% column 5: instantaneous_freq
% column 6: doppler_freq, carrier
% column 7: dcarr
% column 8: code NCO freq
% column 9: magnitude of early corr
% column 10: magnitude of prompt corr
% column 11: magnitude of late corr

figure(1);
 subplot(211),plot(time_idx, sv_tracking_data(:,2));
 ylabel('I_arm output');
 xlabel('time in MS');
 axis tight
 subplot(212),plot(time_idx, sv_tracking_data(:,3));
 ylabel('Q_arm output');
 xlabel('time in MS');
 axis tight
 
 figure(2);
  subplot(211),plot(time_idx, sv_tracking_data(:,4));
 ylabel('\Delta phase in Costas loop');
 xlabel('time in MS');
 axis tight
 subplot(212),plot(time_idx, sv_tracking_data(:,5));
 ylabel('\Delta \tau in DLL loop');
 xlabel('time in MS');
 axis tight
 
 figure(3);
 subplot(211),plot(time_idx, sv_tracking_data(:,6) - 1.4053968E6 );
 ylabel('Carrier Doppler Freq');
 xlabel('time in MS');
 axis tight
 subplot(212),plot(time_idx, sv_tracking_data(:,8)/2 - 1.023e6 );
 ylabel('Code Doppler Freq');
 xlabel('time in MS');
 axis tight
 
 figure(4);
 plot(time_idx, sv_tracking_data(:,9), '.b-', time_idx, sv_tracking_data(:,10),'+k-', time_idx,sv_tracking_data(:,11),'*r-');
 legend('Early','Prompt','Late');
 ylabel('Correlator output');
 xlabel('time in MS');
 axis tight
 
 figure(5);
 plot(sv_tracking_data(2000:cr,2), sv_tracking_data(2000:cr,3),'*','MarkerSize',2);
 axis([-2500,2500,-2500,2500]);
title('I VS Q');

figure(6);
plot( sv_tracking_data(:,10));
