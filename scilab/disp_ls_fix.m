function [] = disp_ls_fix(filename);
close all;

% load fix data
ls_fix_res = load(filename);
%%data format
% column1:   Pos_x  ,   in meter
% column2:   pos_y  ,   in meter
% column3:   pos_z  ,   in meter
% column4:   lat    ,   in radient
% column5:   long   ,   in radient
% column6:   alt    ,   in meter
% column7:   sv_num ,   
% column8:   vx     ,   in m/s
% column9:   vy     ,   in m/s
% column10:  vz     ,   in m/s
% column11:  drft   ,   
% column12:  bias   ,   
% column13:  hdop   ,   
% column14:  vdop   ,   
% column15:  pdop   ,   
% column16:  gdop   ,   
% column17:  localt ,   

[nc,nr] = size(ls_fix_res);
t_idx   = [1:nc];
usr_pos = [-2430670.125632789 -4704149.122364625 3544315.9942142];
st_pos_x = ls_fix_res(:,1)-usr_pos(1);
st_pos_y = ls_fix_res(:,2)-usr_pos(2);
st_pos_z = ls_fix_res(:,3)-usr_pos(3);
lati     = ls_fix_res(:,4);
longi    = ls_fix_res(:,5);
alt      = ls_fix_res(:,6);
sv_num   = ls_fix_res(:,7);
st_vel_x = ls_fix_res(:,8);
st_vel_y = ls_fix_res(:,9);
st_vel_z = ls_fix_res(:,10);
st_bias  = ls_fix_res(:,12);
st_drft  = ls_fix_res(:,11);
hdop     = ls_fix_res(:,13);
vdop     = ls_fix_res(:,14);
pdop     = ls_fix_res(:,15);
gdop     = ls_fix_res(:,16);

figure(1);
subplot(321), plot(t_idx, st_pos_x, '.k-','MarkerSize',5);
title('LS pos ECEF x');xlabel('time is 0.1s');
subplot(323), plot(t_idx, st_pos_y, '.k-','MarkerSize',5);
title('LS pos ECEF y');xlabel('time is 0.1s');
subplot(325), plot(t_idx, st_pos_z, '.k-','MarkerSize',5);
title('LS pos ECEF z');xlabel('time is 0.1s');
subplot(322), plot(t_idx, st_vel_x, '.k-');
title('LS velocity ECEF x');xlabel('time is 0.1s');
subplot(324), plot(t_idx, st_vel_y, '.k-');
title('LS velocity ECEF y');xlabel('time is 0.1s');
subplot(326), plot(t_idx, st_vel_z, '.k-');
title('LS velocity ECEF z');xlabel('time is 0.1s');

figure(2);
subplot(321), plot(t_idx, hdop, '.k-');
title('LS HDOP');xlabel('time is 0.1s');
subplot(323), plot(t_idx, vdop, '.k-');
title('LS VDOP');xlabel('time is 0.1s');
subplot(325), plot(t_idx, pdop, '.k-');
title('LS PDOP');xlabel('time is 0.1s');
subplot(322), plot(t_idx, gdop, '.k-');
title('LS GDOP');xlabel('time is 0.1s');
subplot(324), plot(t_idx, st_bias, '.k-');
title('Clock bias');xlabel('time is 0.1s');
subplot(326), plot(t_idx, st_drft, '.k-');
title('Clock drift');xlabel('time is 0.1s');



% figure(1);
% subplot(311), plot(t_idx, st_pos_x, '.k-','MarkerSize',5);
% title('LS pos ECEF x');xlabel('time is MS');
% subplot(312), plot(t_idx, st_pos_y, '.k-','MarkerSize',5);
% title('LS pos ECEF y');xlabel('time is MS');
% subplot(313), plot(t_idx, st_pos_z, '.k-','MarkerSize',5);
% title('LS pos ECEF z');xlabel('time is MS');
% 
% figure(2);
% subplot(311), plot(t_idx, st_vel_x, '.k-');
% title('LS velocity ECEF x');xlabel('time is MS');
% subplot(312), plot(t_idx, st_vel_y, '.k-');
% title('LS velocity ECEF y');xlabel('time is MS');
% subplot(313), plot(t_idx, st_vel_z, '.k-');
% title('LS velocity ECEF z');xlabel('time is MS');
% 
% figure(3);
% subplot(411), plot(t_idx, hdop, '.k-');
% title('LS HDOP');xlabel('time is MS');
% subplot(412), plot(t_idx, vdop, '.k-');
% title('LS VDOP');xlabel('time is MS');
% subplot(413), plot(t_idx, pdop, '.k-');
% title('LS PDOP');xlabel('time is MS');
% subplot(414), plot(t_idx, gdop, '.k-');
% title('LS GDOP');xlabel('time is MS');
% 
% figure(4);
% subplot(211), plot(t_idx, st_bias, '.k-');
% title('Clock bias');xlabel('time is MS');
% subplot(212), plot(t_idx, st_drft, '.k-');
% title('Clock drift');xlabel('time is MS');
