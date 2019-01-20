    res_log=read('../data/gpscntl_log_3.m',-1,11);
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

