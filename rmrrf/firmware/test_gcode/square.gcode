; 20mm square at origin, pen down/up, returns home.
; Use a scale-down (e.g. $100 temporarily set to 40) for bench validation
; without paper. Assumes carriage was manually positioned before start.
G21 G90
M17           ; enable steppers
G92 X0 Y0     ; call this spot origin
M5            ; pen up
G0 X0 Y0
M3 S1700      ; pen down
G1 X20 Y0 F600
G1 X20 Y20
G1 X0 Y20
G1 X0 Y0
M5            ; pen up
G0 X0 Y0
M18           ; release steppers
