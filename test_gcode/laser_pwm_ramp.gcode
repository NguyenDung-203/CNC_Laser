; ==================================================
; Laser PWM Ramp Test
; Test dam/nhat bang S0..S1000
; Vung test nam gan giua ban, trong hanh trinh X=328mm Y=284mm
; ==================================================
G21
G90
G17
G0 Z5.0

; Di toi vung test, laser chua bat
G0 X82.000 Y100.000

; Arm laser, ha Z xuong job-mid, duty = 0
M3 S0

; Moi vach dai 80mm, cong suat tang dan
G0 X82.000 Y105.000
G1 X82.000 Y185.000 F600 S100

G0 X98.000 Y105.000
G1 X98.000 Y185.000 F600 S200

G0 X114.000 Y105.000
G1 X114.000 Y185.000 F600 S350

G0 X130.000 Y105.000
G1 X130.000 Y185.000 F600 S500

G0 X146.000 Y105.000
G1 X146.000 Y185.000 F600 S650

G0 X162.000 Y105.000
G1 X162.000 Y185.000 F600 S800

G0 X178.000 Y105.000
G1 X178.000 Y185.000 F600 S1000

M5
M2
