acq_res=read('../data/acq_2.m',-1,40);
[r_n,c_n]=size(acq_res)
idx=[1:r_n];
idy=[1:c_n];
set("figure_style","new");
a=get("current_axes");
a.cube_scaling="on";

plot3d(idx,idy,acq_res,35,45,'CA phase@Freq bin@Magnitude', [2 1 4], [0 2046 0 40 0 2000]);


