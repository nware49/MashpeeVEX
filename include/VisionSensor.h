/*vex-vision-config:begin*/

vex::vision::signature SIG_1 = vex::vision::signature (1, -4633, -3737, -4185, -6117, -5161, -5639, 3, 0);
vex::vision::signature SIG_2 = vex::vision::signature (2,  7129, 9057, 8093, -639, -211, -425, 3, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, -3449, -2073, -2761, 8987, 13635, 11311, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);

vex::vision Vision1 = vex::vision (vex::PORT8, 50, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/