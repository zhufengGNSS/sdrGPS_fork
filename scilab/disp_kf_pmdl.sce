res_log=read('../data/gps_kf_p_log.m',-1,16);
[r_n,c_n]=size(res_log);
idx=[1:r_n];

// First plot KF state
//xset('color',3);
//xset('background', 1);
xbasc();

xset('window',0);
// plot pos_x result
subplot(511);
plot2d(res_log(:,1));
xtitle("ECEF_x",'100ms','m');
xset('background', 4);

// plot pos_y result
subplot(512);
plot2d(res_log(:,2));
xtitle("ECEF_y",'100ms','m');
xset('background', 4);

// plot pos_z result
subplot(513);
plot2d(res_log(:,3));
xtitle("ECEF_z", '100ms','m');
xset('background', 4);

// plot clk_bias result
subplot(514);
plot2d(res_log(:,4));
xtitle("clk_bias", '100ms','m');
xset('background', 4);

// plot clk_drift result
subplot(515);
plot2d(res_log(:,5));
xtitle("clk_drift", '100ms','m/s');
xset('background', 4);

// Then plot corrections

xset('window',1);
// plot corr_pos_x result
subplot(511);
plot2d(res_log(:,6));
xtitle("ECEF_x correction",'100ms','m');
xset('background', 4);

// plot corr_pos_y result
subplot(512);
plot2d(res_log(:,7));
xtitle("ECEF_y_correction",'100ms','m');
xset('background', 4);

// plot corr_pos_z result
subplot(513);
plot2d(res_log(:,8));
xtitle("ECEF_z_correction", '100ms','m');
xset('background', 4);

// plot corr_bias result
subplot(514);
plot2d(res_log(:,9));
xtitle("clk_bias_correction", '100ms','m');
xset('background', 4);

// plot corr_drift result
subplot(515);
plot2d(res_log(:,10));
xtitle("clk_drift_correction", '100ms','m/s');
xset('background', 4);

// Then plot diag of P_matrix

xset('window',2);
// plot p_pos_x result
subplot(511);
plot2d(res_log(:,11));
xtitle("ECEF_x_cov",'100ms','m^2');
xset('background', 4);

// plot p_pos_y result
subplot(512);
plot2d(res_log(:,12));
xtitle("ECEF_y_cov",'100ms','m^2');
xset('background', 4);

// plot p_pos_z result
subplot(513);
plot2d(res_log(:,13));
xtitle("ECEF_z_cov", '100ms','m^2');
xset('background', 4);

// plot p_bias result
subplot(514);
plot2d(res_log(:,14));
xtitle("clk_bias_cov", '100ms','m^2');
xset('background', 4);

// plot p_drift result
subplot(515);
plot2d(res_log(:,15));
xtitle("clk_drift_cov", '100ms','(m/s)^2');
xset('background', 4);

