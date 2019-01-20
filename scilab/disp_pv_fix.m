function [] = disp_pv_fix(filename);
close all;

% load tracking data
pv_fix_res = load(filename);
%%data format
% column1:   Pos_x  ,   p_state[0]
% column2:   pos_y  ,   p_state[1]
% column3:   pos_z  ,   p_state[2]
% column4:   vel_x  ,   p_state[3]
% column5:   vel_y  ,   p_state[4]
% column6:   vel_z  ,   p_state[5]
% column7:   bias   ,   p_state[6]
% column8:   drft   ,   p_state[7]
% column9:   dx     ,   correction of p_state[0]
% column10:  dy     ,   correction of p_state[1]
% column11:  dz     ,   correction of p_state[2]
% column12:  dvx    ,   correction of p_state[3]
% column13:  dvy    ,   correction of p_state[4]
% column14:  dvz    ,   correction of p_state[5]
% column15:  dbias  ,   correction of p_state[6]
% column16:  ddrft  ,   correction of p_state[7]
% column17:  P_x    ,   cov() of P_state[0]    
% column18:  P_y    ,   cov() of P_state[1]
% column19:  P_z    ,   cov() of P_state[2]
% column20:  P_vx   ,   cov() of P_state[3]    
% column21:  P_vy   ,   cov() of P_state[4]
% column22:  P_vz   ,   cov() of P_state[5]
% column23:  P_b    ,   cov() of P_state[6]
% column24:  P_d    ,   cov() of P_state[7]

[nc,nr] = size(pv_fix_res);
t_idx   = [1:nc];
usr_pos = [-2430670.125632789 -4704149.122364625 3544315.9942142];
st_pos_x = pv_fix_res(:,1)-usr_pos(1);
st_pos_y = pv_fix_res(:,2)-usr_pos(2);
st_pos_z = pv_fix_res(:,3)-usr_pos(3);
st_vel_x = pv_fix_res(:,4);
st_vel_y = pv_fix_res(:,5);
st_vel_z = pv_fix_res(:,6);
st_bias  = pv_fix_res(:,7);
st_drft  = pv_fix_res(:,8);
d_pos_x = pv_fix_res(:,9);
d_pos_y = pv_fix_res(:,10);
d_pos_z = pv_fix_res(:,11);
d_vel_x = pv_fix_res(:,12);
d_vel_y = pv_fix_res(:,13);
d_vel_z = pv_fix_res(:,14);
d_bias  = pv_fix_res(:,15);
d_drft  = pv_fix_res(:,16);
p_pos_x = pv_fix_res(:,17);
p_pos_y = pv_fix_res(:,18);
p_pos_z = pv_fix_res(:,19);
p_vel_x = pv_fix_res(:,20);
p_vel_y = pv_fix_res(:,21);
p_vel_z = pv_fix_res(:,22);
p_bias  = pv_fix_res(:,23);
p_drft  = pv_fix_res(:,24);

figure(1);
subplot(421), plot(t_idx, st_pos_x, '.k-','MarkerSize',5);
title('pos ECEF x');xlabel('time is 0.1s');
subplot(422), plot(t_idx, st_pos_y, '.k-','MarkerSize',5);
title('pos ECEF y');xlabel('time is 0.1s');
subplot(423), plot(t_idx, st_pos_z, '.k-','MarkerSize',5);
title('pos ECEF z');xlabel('time is 0.1s');
subplot(424), plot(t_idx, st_vel_x, '.k-');
title('velocity ECEF x');xlabel('time is 0.1s');
subplot(425), plot(t_idx, st_vel_y, '.k-');
title('velocity ECEF y');xlabel('time is 0.1s');
subplot(426), plot(t_idx, st_vel_z, '.k-');
title('velcity ECEF z');xlabel('time is 0.1s');
subplot(427), plot(t_idx, st_bias, '.k-');
title('Clock bias');xlabel('time is 0.1s');
subplot(428), plot(t_idx, st_drft, '.k-');
title('Clock drift');xlabel('time is 0.1s');

figure(2);
subplot(421), plot(t_idx, d_pos_x, '.k-');
title('KF correction ECEF \delta x');xlabel('time is 0.1s');
subplot(422), plot(t_idx, d_pos_y, '.k-');
title('KF correction ECEF \delta y');xlabel('time is 0.1s');
subplot(423), plot(t_idx, d_pos_z, '.k-');
title('KF correction ECEF \delta z');xlabel('time is 0.1s');
subplot(424), plot(t_idx, d_vel_x, '.k-');
title('KF correction ECEF \delta v_x');xlabel('time is 0.1s');
subplot(425), plot(t_idx, d_vel_y, '.k-');
title('KF correction ECEF \delta v_y');xlabel('time is 0.1s');
subplot(426), plot(t_idx, d_vel_z, '.k-');
title('KF correction ECEF \delta v_z');xlabel('time is 0.1s');
subplot(427), plot(t_idx, d_bias, '.k-');
title('KF correction of \delta bias');xlabel('time is 0.1s');
subplot(428), plot(t_idx, d_drft, '.k-');
title('KF correction of \delta drift');xlabel('time is 0.1s');

figure(3);
subplot(421), plot(t_idx, p_pos_x, '.k-');
title('KF covariance ECEF P_x');xlabel('time is 0.1s');
subplot(422), plot(t_idx, p_pos_y, '.k-');
title('KF covariance ECEF P_y');xlabel('time is0.1s');
subplot(423), plot(t_idx, p_pos_z, '.k-');
title('KF covariance ECEF P_z');xlabel('time is 0.1s');
subplot(424), plot(t_idx, p_vel_x, '.k-');
title('KF covariance ECEF P_{vx}');xlabel('time is 0.1s');
subplot(425), plot(t_idx, p_vel_y, '.k-');
title('KF covariance ECEF P_{vy}');xlabel('time is 0.1s');
subplot(426), plot(t_idx, p_vel_z, '.k');
title('KF covariance ECEF P_{vz}');xlabel('time is 0.1s');
subplot(427), plot(t_idx, p_bias, '.k-');
title('KF covariance of Clock bias');xlabel('time is 0.1s');
subplot(428), plot(t_idx, p_drft, '.k-');
title('KF covariance of clock drift');xlabel('time is 0.1s');

% figure(1);
% subplot(311), plot(t_idx, st_pos_x, '.k-','MarkerSize',5);
% title('pos ECEF x');xlabel('time is MS');
% subplot(312), plot(t_idx, st_pos_y, '.k-','MarkerSize',5);
% title('pos ECEF y');xlabel('time is MS');
% subplot(313), plot(t_idx, st_pos_z, '.k-','MarkerSize',5);
% title('pos ECEF z');xlabel('time is MS');
% 
% figure(2);
% subplot(311), plot(t_idx, st_vel_x, '.k-');
% title('velocity ECEF x');xlabel('time is MS');
% subplot(312), plot(t_idx, st_vel_y, '.k-');
% title('velocity ECEF y');xlabel('time is MS');
% subplot(313), plot(t_idx, st_vel_z, '.k-');
% title('velcity ECEF z');xlabel('time is MS');
% 
% figure(3);
% subplot(311), plot(t_idx, d_pos_x, '.k-');
% title('KF correction ECEF delta_x');xlabel('time is MS');
% subplot(312), plot(t_idx, d_pos_y, '.k-');
% title('KF correction ECEF delta_y');xlabel('time is MS');
% subplot(313), plot(t_idx, d_pos_z, '.k-');
% title('KF correction ECEF delta_z');xlabel('time is MS');
% 
% figure(4);
% subplot(311), plot(t_idx, d_vel_x, '.k-');
% title('KF correction ECEF delta_vx');xlabel('time is MS');
% subplot(312), plot(t_idx, d_vel_y, '.k-');
% title('KF correction ECEF delta_vy');xlabel('time is MS');
% subplot(313), plot(t_idx, d_vel_z, '.k');
% title('KF correction ECEF delta_vz');xlabel('time is MS');
% 
% figure(5);
% subplot(311), plot(t_idx, p_pos_x, '.k-');
% title('KF covariance ECEF P_x');xlabel('time is MS');
% subplot(312), plot(t_idx, p_pos_y, '.k-');
% title('KF covariance ECEF P_y');xlabel('time is MS');
% subplot(313), plot(t_idx, p_pos_z, '.k-');
% title('KF covariance ECEF P_z');xlabel('time is MS');
% 
% figure(6);
% subplot(311), plot(t_idx, p_vel_x, '.k-');
% title('KF covariance ECEF P_vx');xlabel('time is MS');
% subplot(312), plot(t_idx, p_vel_y, '.k-');
% title('KF covariance ECEF P_vy');xlabel('time is MS');
% subplot(313), plot(t_idx, p_vel_z, '.k');
% title('KF covariance ECEF P_vz');xlabel('time is MS');
% 
% figure(7);
% subplot(211), plot(t_idx, st_bias, '.k-');
% title('Clock bias');xlabel('time is MS');
% subplot(212), plot(t_idx, st_drft, '.k-');
% title('Clock drift');xlabel('time is MS');
% 
% figure(8);
% subplot(211), plot(t_idx, d_bias, '.k-');
% title('KF correction of Clock bias');xlabel('time is MS');
% subplot(212), plot(t_idx, d_drft, '.k-');
% title('KF correction of clock drift');xlabel('time is MS');
% 
% figure(9);
% subplot(211), plot(t_idx, p_bias, '.k-');
% title('KF covariance of Clock bias');xlabel('time is MS');
% subplot(212), plot(t_idx, p_drft, '.k-');
% title('KF covariance of clock drift');xlabel('time is MS');