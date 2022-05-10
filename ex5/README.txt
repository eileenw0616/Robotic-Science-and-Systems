OPT:
opt.m --code for (a) and (b), run this .m file. To use different cost function please modify line 7.
x = fmincon(@cost, x_init, [],[],[],[],[],[],@nonlcon);
Please change @cost to @cost1 / @cost2 / @cost3
Figures of trajectory can be found in the folder(opt_a.fig, opt_b1.fig, opt_b2.fig, opt_b3.fig).
Output and answers are in opt_a&b.txt(the sequence of points)


opt_c.m --code for (c),(d) and (e), run this .m file. To try different T please modify gloal T.
Figures of trajectory can be found in the folder(opt_d.fig, opt_e_40.fig, opt_e_60.fig, opt_e_80.fig, opt_e_100.fig).
Output and answers are in opt_d&e.txt(the sequence of points)

V1

v1.m Please run this file.
Examples of outputs can be found in V1output.txt
2 png file for homograghy named v1b_cover_to_im1 and v1b_im1_to_im2
images used in V1 can be found in folder imgs.  

V2
(b) v2.m Please run this file. Output is in V2_b.txt file
(c) v2_c.m Please run this file. Output is in V2_c.txt file

V3
(a) v3_a.m Please run this file. Sample(random point) output is in v3_a.txt
(b) v3_b.m Run this file. It may take a while. Output(normal and center of 3 planes) is in v3b_output.txt
(c) v3_b.m Run this file. About 5 min.Output is in v3c_output.txt
(d) unfinished