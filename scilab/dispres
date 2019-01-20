res_log=read('../data/gpscntl_log_0.m',-1,11);
pvt_log=read('../data/gps_navfix_log.m',-1, 16);
[r_n,c_n]=size(res_log);
idx=[1:r_n];



//xset('color',3);
//xset('background', 1);
xbasc();
// plot I result
xset('window',0);
subplot(221);
plot2d(res_log(:,2));
xtitle("I Output",'ms','Mag');
xset('background', 4);

// plot Q result
subplot(223);
plot2d(res_log(:,3));
xtitle("Q Output",'ms','Mag');
xset('background', 4);

subplot(222);
plot2d(res_log(:,6)-(4.1304e6));
xtitle("Err of carrier freq", 'ms','Hz');
xset('background', 4);

subplot(224);
plot2d(res_log(:,8)/2-1.023e6);
xtitle("Err of code freq", 'ms', 'Hz');

xset('background', 4);

xset('window',1);
plot2d(idx, [res_log(:,9), res_log(:,10), res_log(:,11)], leg="Early@Prompt@Late");
xtitle("Correlation result for Early, Prompt, Late phase");


xset('window',2);
subplot(311);
plot2d(pvt_log(:,1));
xtitle("ECEF position x",'TIC','m');
xset('background',4);

subplot(312);
plot2d(pvt_log(:,2));
xtitle("ECEF position y",'TIC','m');
xset('background',4);

subplot(313);
plot2d(pvt_log(:,3));
xtitle("ECEF position z",'TIC','m');
xset('background',4);

xset('window',3);
subplot(311);
plot2d(pvt_log(:,8));
xtitle("ECEF velocity x",'TIC','m/s');
xset('background',4);

subplot(312);
plot2d(pvt_log(:,8));
xtitle("ECEF velocity y",'TIC','m/s');
xset('background',4);

subplot(313);
plot2d(pvt_log(:,10));
xtitle("ECEF velocity z",'TIC','m/s');
xset('background',4);

xset('window',4);
subplot(211);
plot2d(pvt_log(:,11));
xtitle("Clock drift rate", 'TIC', '1/s');
xset('background',4);

subplot(212);
plot2d(pvt_log(:,12));
xtitle("Clock bias", 'TIC', 's');
xset('background',4);

xset('window',5);
subplot(411);
plot2d(pvt_log(:,13));
xtitle("hdop", 'TIC', '1/s');
xset('background',4);

subplot(412);
plot2d(pvt_log(:,14));
xtitle("vdop", 'TIC', 's');
xset('background',4);
subplot(413);
plot2d(pvt_log(:,15));
xtitle("pdop", 'TIC', '1/s');
xset('background',4);

subplot(414);
plot2d(pvt_log(:,16));
xtitle("gdop", 'TIC', 's');
xset('background',4);
