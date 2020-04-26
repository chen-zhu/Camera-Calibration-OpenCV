To run the program, type the following into your terminal:
cmake . && make && ./opengl_cv_skeleton


<SAMPLE POTOT errors>:
Height x width: 480 * 640
RMS error: 1.24762
avg reprojection error: 1.2476175
H-fov: 63.18
V-fov: 50.01
cx: 4.0878827641099446e+02
cy: 2.3161888140981759e+02
fx: 5.2039099291642742e+02
fy: 5.1457672296261217e+02
k1: 3.2831473731976549e-02
k2: -1.3888348850463150e-02
p1: -1.4222056885107616e-03
p2: 6.7542112942790165e-03
k3: 0


<MY CAMERA'S errors>:
Height x width: 720 * 1280
RMS error: 0.20382
avg reprojection error: 0.2038160
H-fov: 68.6898
V-fov: 42.0195
cx: 6.3937849242430661e+02
cy: 3.5571784517823068e+02
fx: 9.3662793596091171e+02
fy: 9.3735420508958236e+02
k1: 6.9191186809877112e-02
k2: -9.9163998698758346e-02
p1: -1.2398694086340389e-03
p2: 3.2412933543665302e-03
k3: 0



My camera has smaller reprojection error than the sample photos, which could be that my camera is newer & well-produced/well-calibrated and it has way higher resolutions (720*1280) than the sample photos (480*640). (I actually ran the calibration program a couple times. I did notice that the program returned different avg reproduction errors with different camera intrinsic values). In addition, the vertical FOV of my camera is smaller than the sample pictures, while the horizontal FOV is larger than the sample pictures. 


