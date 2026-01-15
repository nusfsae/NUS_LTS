tirParams = mfeval.readTIR("D:\Tire Data and Model\TTC Round 6\43105_18x7.5_10_R25B_7.tir");
% [Fx, Fy] = mfeval.MFrelationOld(tirParams, 3000, 1, 12, 0, 10, 9);
[Fx, Fy] = MF61(3000, 1, 12, 0, 10, 9, tirParams);
disp([Fx,Fy])
[FXo, FYo] = tires(tirParams, 3000, 1, 12, 0, 10,9);
disp([FYo, FXo]);