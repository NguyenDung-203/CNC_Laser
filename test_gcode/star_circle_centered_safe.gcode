; ==================================================
; Safe centered G-code test - star + circles
; Source design offset by X+114, Y+92 so center (50,50) -> machine center (164,142)
; STM32 debug firmware ignores Z and forces laser OFF.
; Final move returns to center instead of machine home.
; ==================================================
G21
G90
G17
G0 Z5.0

; Outer circle, radius 40 mm, centered at X164 Y142
G0 X204.000 Y142.000
M3 S1200
G1 Z-1.0 F300
G2 X124.000 Y142.000 I-40.000 J0 F800
G2 X204.000 Y142.000 I40.000 J0
G0 Z5.0

; Five-point star
G0 X164.000 Y112.000
G1 Z-1.0 F300
G1 X171.053 Y132.292 F800
G1 X192.532 Y132.729 F800
G1 X175.413 Y145.708 F800
G1 X181.634 Y166.271 F800
G1 X164.000 Y154.000 F800
G1 X146.366 Y166.271 F800
G1 X152.587 Y145.708 F800
G1 X135.468 Y132.729 F800
G1 X156.947 Y132.292 F800
G1 X164.000 Y112.000 F800
G0 Z5.0

; Small center circle, radius 5 mm
G0 X169.000 Y142.000
G1 Z-1.0 F300
G3 X159.000 Y142.000 I-5.000 J0 F800
G3 X169.000 Y142.000 I5.000 J0
G0 Z5.0

M5
G0 X164.000 Y142.000
M2
