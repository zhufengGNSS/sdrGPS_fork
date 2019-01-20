res_log=read('../data/gps_kf_pv_log.m',-1,24);
[r_n,c_n]=size(res_log);
idx=[1:r_n];

// First plot KF state
//xset('color',3);
//xset('background', 1);
xbasc();

xset('window',0);
// plot pos_x result
subplot(811);
plot2d(res_log(:,1));
xtitle("ECEF_x",'100ms','m');
xset('background', 4);

// plot pos_y result
subplot(812);
plot2d(res_log(:,2));
xtitle("ECEF_y",'100ms','m');
xset('background', 4);

// plot pos_z result
subplot(813);
plot2d(res_log(:,3));
xtitle("ECEF_z", '100ms','m');
xset('background', 4);

// plot VEL_x result
subplot(814);
plot2d(res_log(:,4));
xtitle("VEL_x", '100ms','m/s');
xset('background', 4);

// plot VEL_y result
subplot(815);
plot2d(res_log(:,5));
xtitle("VEL_y", '100ms','m/s');
xset('background', 4);

// plot vel_z result
subplot(816);
plot2d(res_log(:,6));
xtitle("VEL_z", '100ms','m/s');
xset('background', 4);

// plot clk_bias result
subplot(817);
plot2d(res_log(:,7));
xtitle("clk_bias", '100ms','m');
xset('background', 4);

// plot clk_drift result
subplot(818);
plot2d(res_log(:,8));
xtitle("clk_drift", '100ms','m/s');
xset('background', 4);

// Then plot corrections

xset('window',1);
// plot corr_pos_x result
subplot(811);
plot2d(res_log(:,9));
xtitle("ECEF_x correction",'100ms','m');
xset('background', 4);

// plot corr_pos_y result
subplot(812);
plot2d(res_log(:,10));
xtitle("ECEF_y_correction",'100ms','m');
xset('background', 4);

// plot corr_pos_z result
subplot(813);
plot2d(res_log(:,11));
xtitle("ECEF_z_correction", '100ms','m');
xset('background', 4);

// plot corr_vel_x result
subplot(814);
plot2d(res_log(:,12));
xtitle("VEL_x correction",'100ms','m/s');
xset('background', 4);

// plot corr_vel_y result
subplot(815);
plot2d(res_log(:,13));
xtitle("VEL_y_correction",'100ms','m/s');
xset('background', 4);

// plot corr_vel_z result
subplot(816);
plot2d(res_log(:,14));
xtitle("VEL_z_correction", '100ms','m/s');
xset('background', 4);

// plot corr_bias result
subplot(817);
plot2d(res_log(:,15));
xtitle("clk_bias_correction", '100ms','m');
xset('background', 4);

// plot corr_drift result
subplot(818);
plot2d(res_log(:,16));
xtitle("clk_drift_correction", '100ms','m/s');
xset('background', 4);

// Then plot diag of P_matrix

xset('window',2);
// plot p_pos_x result
subplot(811);
plot2d(res_log(:,17));
xtitle("ECEF_x_cov",'100ms','m^2');
xset('background', 4);

// plot p_pos_y result
subplot(812);
plot2d(res_log(:,18));
xtitle("ECEF_y_cov",'100ms','m^2');
xset('background', 4);

// plot p_pos_z result
subplot(813);
plot2d(res_log(:,19));
xtitle("ECEF_z_cov", '100ms','m^2');
xset('background', 4);

// plot p_vel_x result
subplot(814);
plot2d(res_log(:,20));
xtitle("VEL_x_cov",'100ms','m^2');
xset('background', 4);

// plot p_vel_y result
subplot(815);
plot2d(res_log(:,21));
xtitle("VEL_y_cov",'100ms','m^2');
xset('background', 4);

// plot p_vel_z result
subplot(816);
plot2d(res_log(:,22));
xtitle("VEL_z_cov", '100ms','m^2');
xset('background', 4);

// plot p_bias result
subplot(817);
plot2d(res_log(:,23));
xtitle("clk_bias_cov", '100ms','m^2');
xset('background', 4);

// plot p_drift result
subplot(818);
plot2d(res_log(:,24));
xtitle("clk_drift_cov", '100ms','(m/s)^2');
xset('background', 4);

