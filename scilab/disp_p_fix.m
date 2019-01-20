function [] = disp_p_fix(filename);
close all;

% load tracking data
p_fix_res = load(filename);
%%data format
% column1:   Pos_x  ,   p_state[0]
% column2:   pos_y  ,   p_state[1]
% column3:   pos_z  ,   p_state[2]
% column4:   bias   ,   p_state[3]
% column5:   drft   ,   p_state[4]
% column6:   dx     ,   correction of p_state[0]
% column7:   dy     ,   correction of p_state[1]
% column8:   dz     ,   correction of p_state[2]
% column9:   dbias  ,   correction of p_state[3]
% column10:  ddrft  ,   correction of p_state[4]
% column11:  P_x    ,   cov() of P_state[0]    
% column12:  P_y    ,   cov() of P_state[1]
% column13:  P_z    ,   cov() of P_state[2]
% column14:  P_b    ,   cov() of P_state[3]
% column15:  P_d    ,   cov() of P_state[4]

[nc,nr] = size(p_fix_res);
t_idx   = [1:nc];
usr_pos = [-2430670.125632789 -4704149.122364625 3544315.9942142];
st_pos_x = p_fix_res(:,1)-usr_pos(1);
st_pos_y = p_fix_res(:,2)-usr_pos(2);
st_pos_z = p_fix_res(:,3)-usr_pos(3);
st_bias  = p_fix_res(:,4);
st_drft  = p_fix_res(:,5);
d_pos_x = p_fix_res(:,6);
d_pos_y = p_fix_res(:,7);
d_pos_z = p_fix_res(:,8);
d_bias  = p_fix_res(:,9);
d_drft  = p_fix_res(:,10);
p_pos_x = p_fix_res(:,11);
p_pos_y = p_fix_res(:,12);
p_pos_z = p_fix_res(:,13);
p_bias  = p_fix_res(:,14);
p_drft  = p_fix_res(:,15);

figure(1);
subplot(511), plot(t_idx, st_pos_x, '.k-');
title('pos ECEF x');xlabel('time is 0.1s');
subplot(512), plot(t_idx, st_pos_y, '.k-');
title('pos ECEF y');xlabel('time is 0.1s');
subplot(513), plot(t_idx, st_pos_z, '.k-');
title('pos ECEF z');xlabel('time is 0.1s');
subplot(514), plot(t_idx, st_bias, '.k-');
title('Clock bias');xlabel('time is 0.1s');
subplot(515), plot(t_idx, st_drft, '.k-');
title('Clock drift');xlabel('time is 0.1s');

figure(2);
subplot(511), plot(t_idx, d_pos_x, '.k-');
title('KF correction ECEF delta_x');xlabel('time is 0.1s');
subplot(512), plot(t_idx, d_pos_y, '.k-');
title('KF correction ECEF delta_y');xlabel('time is 0.1s');
subplot(513), plot(t_idx, d_pos_z, '.k-');
title('KF correction ECEF delta_z');xlabel('time is 0.1s');
subplot(514), plot(t_idx, d_bias, '.k-');
title('KF correction of Clock bias');xlabel('time is 0.1s');
subplot(515), plot(t_idx, d_drft, '.k-');
title('KF correction of clock drift');xlabel('time is 0.1s');

figure(3);
subplot(511), plot(t_idx, p_pos_x, '.k-');
title('KF covariance ECEF P_x');xlabel('time is 0.1s');
subplot(512), plot(t_idx, p_pos_y, '.k-');
title('KF covariance ECEF P_y');xlabel('time is 0.1s');
subplot(513), plot(t_idx, p_pos_z, '.k-');
title('KF covariance ECEF P_z');xlabel('time is 0.1s');
subplot(514), plot(t_idx, p_bias, '.k-');
title('KF covariance of Clock bias');xlabel('time is 0.1s');
subplot(515), plot(t_idx, p_drft, '.k-');
title('KF covariance of clock drift');xlabel('time is 0.1s');

% figure(4);
% subplot(211), plot(t_idx, st_bias, '.k-');
% title('Clock bias');xlabel('time is MS');
% subplot(212), plot(t_idx, st_drft, '.k-');
% title('Clock drift');xlabel('time is MS');
% 
% figure(5);
% 
% 
% figure(6);

