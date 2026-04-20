; Circle + half-circle — exercises G2/G3 arc interpolation.
G21 G90
M17
G92 X0 Y0
M5
G0 X20 Y0
M3 S1700
G2 X20 Y0 I-20 J0 F600  ; full circle CW (end == start, I/J to center)
M5
G0 X40 Y0
M3 S1700
G3 X60 Y0 I10 J0 F600   ; half-circle CCW radius 10
M5
G0 X0 Y0
M18
