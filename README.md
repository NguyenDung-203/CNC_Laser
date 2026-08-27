# CNC Laser Notes

## Mapping truc thuc te

Ket qua test auto scan sau reset:

```text
Thu tu firmware scan cu: CH1 -> CH2 -> CH3 -> CH4
Thu tu truc thay chay:    Y   -> X   -> Z
```

Suy ra mapping thuc te hien tai:

```text
Y = CH1 = ENA1 / STEP1 / DIR1
X = CH2 = ENA2 / STEP2 / DIR2
Z = CH3 = ENA3 / STEP3 / DIR3
CH4 = chua thay co motor / chua dung
```

## Chieu quay / DIR polarity

Ket qua quan sat khi firmware auto test chay moi truc theo thu tu `DIR=0` roi `DIR=1`:

```text
X: DIR=0 -> X+, DIR=1 -> X-
Y: DIR=0 -> Y-, DIR=1 -> Y+
Z: DIR=0 -> Z- / di xuong, DIR=1 -> Z+ / di len
```

Suy ra khi code lenh di chuyen theo chieu duong:

```text
X+ dung DIR=0
Y+ dung DIR=1
Z+ dung DIR=1
```

Pin STM32 tu `Core/Inc/main.h`:

```text
Y/CH1: ENA1 = PE15, STEP1 = PE14, DIR1 = PB1
X/CH2: ENA2 = PB0,  STEP2 = PE13, DIR2 = PC5
Z/CH3: ENA3 = PE10, STEP3 = PE9,  DIR3 = PE8
CH4:   ENA4 = PE12, STEP4 = PE11, DIR4 = PC4
```

## Firmware test hien tai

Firmware hien tai doc W25Q128FVSIG truoc khi quyet dinh boot flow:

```text
Khi W25Q co record hop le: restore X/Y/Z last-commanded position, dung im, khong auto home
Khi W25Q chua co record/CRC sai: home Z/X/Y dong thoi, sau do dua X/Y ra chinh giua dong thoi
Center tu home min:
  X center = 26369 pulse theo X+ / DIR=0
  Y center = 22720 pulse theo Y+ / DIR=1
  Z giu o home tren cung, Zdown = 0.000 mm
Home hien tai chay dong thoi X/Y/Z: truc nao cham limit truoc thi dung rieng truc do
Center hien tai chay dong thoi X/Y bang TIM5 DDA, Z giu nguyen
Lenh h: chi home Z, Z+ / di len / DIR=1
Lenh H: full home Z/X/Y
  Home Z/X/Y cung luc
  Z: Z+ / di len / DIR=1
  X: X- / DIR=1
  Y: Y- / DIR=0
Lenh s: doc trang thai LIMIT
Lenh ! hoac d: dung khan va tat driver khi dang chay
Lenh R: reset software STM32 qua UART, khong can bam nut reset
Khi LIMIT nao doi trang thai thi dung truc do va tat tat ca driver
Neu truc dang nam tren home limit san thi firmware se skip truc do, khong keo tiep vao cong tac
```

Luu vi tri bang W25Q SPI flash:

```text
Chip user nhan dien: W25Q128FVSIG tren board STM32, SPI1 PB3/PB4/PB5, CS PD7
JEDEC doc thuc te sau khi nap: EF 40 17
Firmware tinh dung luong theo JEDEC capacity code:
  EF 40 17 -> 8 MiB, sector cuoi 0x007FF000
  EF 40 18 -> 16 MiB/W25Q128, sector cuoi 0x00FFF000
Ban ghi: magic/version/sequence/Xsteps/Ysteps/Zsteps/Xum/Yum/Zum/CRC32
CS flash idle = HIGH
Sau khi motion DONE: danh dau dirty, doi may dung yen ~1s moi ghi flash
Sau khi home thanh cong: set X/Y/Z ve 0 va ghi flash ngay
Lenh R/B: flush dirty truoc khi reset
Lenh s: in them W25Q=off/empty/dirty/saved va seq
Ghi chu quan trong: day la last commanded position cua stepper open-loop; neu tat may roi day truc bang tay thi flash khong biet vi tri that, can gui H de home lai truoc khi center/chay job.
```

Khoang cach dung de tinh lai sau homing full hanh trinh:

```text
X travel = 328.000 mm
Y travel = 284.000 mm
Z travel =   5.400 mm
```

Ket qua homing da xac nhan qua ESP32 UART:

```text
Z/CH3 home/up  -> LIMIT3, 11657 pulse
X/CH2 home/min -> LIMIT2, 21374 pulse
Y/CH1 home/min -> LIMIT1, 22145 pulse
Limit dang active-low: binh thuong = 1, cham cong tac = 0
```

Ket qua homing full hanh trinh voi X=328 mm, Y=284 mm, Z=5.4 mm:

```text
Z/CH3 home/up  -> NO HIT, dung o max 25000 pulse
X/CH2 home/min -> LIMIT2, 52738 pulse
Y/CH1 home/min -> LIMIT1, 45440 pulse

Tinh ra voi driver 1/16 microstep:
X = 19.902 mm/vong motor
Y = 20.000 mm/vong motor
Z = chua tinh duoc vi chua hit LIMIT3 lan nay
```

Ket qua retest lenh `h` cho rieng Z:

```text
Z/CH3 home/up -> NO HIT, dung o max 25000 pulse
Baseline va ket thuc deu L3=1, tuc LIMIT3 khong doi trang thai
```

Ket qua Z retest sau khi tang max len 50000 pulse:

```text
Z/CH3 home/up -> LIMIT3, 40377 pulse
Tinh voi Z travel = 5.400 mm:
Z = 0.427 mm/vong motor neu driver 1/16 microstep
Z = 7477 pulse/mm neu driver 1/16 microstep
Max Z homing da chinh lai = 45000 pulse
```

Ket qua startup auto-home + center da kiem tra:

```text
Z/CH3 dang o LIMIT3 nen skip home Z, sau do giu Z o home/top
X/CH2 home -> LIMIT2, sau do center X+ 26369 pulse
Y/CH1 home -> LIMIT1, sau do center Y+ 22720 pulse
Ket thuc: CENTER DONE, drivers disabled, POS X=164.000mm Y=142.000mm Zdown=0.000mm
```

Startup center hien tai:

```text
Sau khi home Z/X/Y dong thoi va set vi tri ve X0 Y0 Zdown0, firmware goi motion engine TIM5:
G0 tu X0 Y0 -> X164 Y142, Z giu o Zdown0
X va Y cung enable va chay dong thoi ve center bang DDA/Bresenham
Ket thuc thi tat driver va cap nhat POS X=164.000mm Y=142.000mm Zdown=0.000mm
```

Ket qua startup sau khi tang X/Y len 4 kHz:

```text
STEP frequency: X/Y 4 kHz, Z 2 kHz
Z/CH3 home -> LIMIT3, 24717 pulse
X/CH2 home -> LIMIT2, 26247 pulse
Y/CH1 home -> LIMIT1, 22859 pulse
Sau do center X+ 26369 pulse, center Y+ 22720 pulse
Ket thuc: CENTER DONE, drivers disabled
```

Ket qua startup sau khi tang Z len 4 kHz va sua LED 10ms/1s:

```text
STEP frequency: X/Y/Z 4 kHz
Z/CH3 dang o LIMIT3 nen skip, giu Z tren cung
X/CH2 home -> LIMIT2, 26368 pulse, sau do center X+ 26369 pulse
Y/CH1 home -> LIMIT1, 22853 pulse, sau do center Y+ 22720 pulse
Ket thuc: CENTER DONE, drivers disabled
```

Thong so STEP hien tai:

```text
STEP frequency X/Y = 8 kHz
STEP frequency Z   = 8 kHz
STEP high      = 10 us
STEP low X/Y   = 115 us
STEP low Z     = 115 us
Max Z           = 45000 pulse
Max X/Y         = 60000 pulse
LED status      = flash 1 Hz, sang 2 ms moi chu ky 1000 ms
LED timing      = TIM14 1 ms interrupt, khong phu thuoc motor/UART idle
UART idle poll  = non-blocking
```

Noi suy XYZ hien tai:

```text
Da ho tro UART line command:
- G21: mm mode
- G90: toa do tuyet doi
- G91: toa do tuong doi
- G17: chon mat phang XY, firmware parse/accept de tuong thich file CAM
- G0 X.. Y..: noi suy XY, rapid feed mac dinh 3000 mm/min, laser off khi travel
- G1 X.. Y.. F.. S..: noi suy XY theo feed, laser ON trong luc chay neu da M3/M4 va S>0
- G2/G3 X.. Y.. I.. J.. F..: cung tron XY, STM32 chia thanh cac doan G1 nho bang TIM5
- Z word / G0 Z.. / G1 Z..: G0 Z truoc M3 duoc accept nhung khong ha Z; khi dang khac thi dam bao Z o mid
- M3/M4 S..: arm laser, ha Z xuong giua hanh trinh; firmware dang binary mode, S0 tat va S>0 bat full
- M3/M4 S0: arm laser va ha Z xuong giua hanh trinh nhung output laser = off, de file co the bat bang G1 S...
- M5: laser off
- M2/M30: ket thuc chuong trinh, laser off, home Z/X/Y ve cong tac home

Vi du test nho tu vi tri center:
G91
G1 X10 Y10 F600
G1 X-10 Y-10 F600
G90

Ket qua test thuc te voi 5 mm:
G91 -> ok
G1 X5 Y5 F600 -> Xsteps=804, Ysteps=800, period_us=879, ok
G1 X-5 Y-5 F600 -> Xsteps=804, Ysteps=800, period_us=879, ok
G90 + s -> POS X=164.000mm Y=142.000mm Zdown=0.000mm, ve lai center

Ghi chu:
- Noi suy STEP hien chay bang TIM5 interrupt 1 MHz, DDA/Bresenham cho X/Y/Z.
- Startup/Home/M2 homing chay X/Y/Z dong thoi; startup center chi chay X/Y, Z giu home/top.
- Main loop khong con busy-wait trong ca hanh trinh G0/G1; ok chi tra sau khi timer motion DONE.
- Cung tron G2/G3 duoc chia segment khoang 2 mm, toi thieu 12 segment va toi da 240 segment.
- Voi G2/G3, STM32 chi tra ok sau khi chay het toan bo cung, khong tra ok sau tung segment noi bo.
- Z job-mid = 20189 pulse theo Z- / DIR=0, tu home tren di xuong khoang 2.700 mm.
- Sau home/center, Z van o home/top; M3/M4 se ha Z ve mid truoc khi tra ok.
- Firmware chi ha Z job-mid khi gap M3/M4 hoac khi dang khac ma file co dong Z-only; G0 Z truoc M3 duoc accept nhung khong ha Z.
- G0 luon tat laser trong luc travel; G1/G2/G3 bat laser trong luc motion neu M3/M4 dang active.
- Sau moi line/segment motion, firmware tat output laser de tranh chay diem khi doi dong tiep theo.
- Laser phan cung hien tai chi on/off; firmware dat LASER_BINARY_ONLY=1.
- PD14/TIM4_CH3 van duoc init, nhung S>0 bi ep thanh full ON, S0 la OFF.
- Lenh s in them laser=armed/off, S=<power>/1000 va mode=binary.
- M5 chi tat trang thai khac, khong home; M2/M30 moi home Z/X/Y de file co rapid cuoi khong lam Z ha lai.
- Chua co planner queue va acceleration.
- Host nen gui tung dong va doi firmware tra ok roi moi gui dong tiep theo.
- Neu byte den trong luc motion, firmware chi buffer duoc mot dong phu; ban chuan van can UART interrupt/ring buffer.
- G0/G1 dang dung soft limit X=0..328 mm, Y=0..284 mm, Zdown=0..5.4 mm.
```

File test ngoi sao + duong tron vua them:

```text
test_gcode/star_circle_centered_safe.gcode
test_gcode/complex_mandala_z_mid.gcode
test_gcode/complex_laser_job_home_test.gcode
test_gcode/laser_pwm_ramp.gcode
test_gcode/dragon_180mm_lineart_s650.gcode
test_gcode/ultra_complex_cyber_mandala_15mm_onoff.gcode
test_gcode/viet_nam_flag_150x60_onoff.gcode
test_gcode/hoang_dau_huyen_times_thin_80x60_onoff.gcode
test_gcode/vietnam_map_sticker_80x60_onoff.gcode
test_gcode/son_o_horn_times_thin_80x60_onoff.gcode
```

File rong `dragon_180mm_lineart_s650.gcode`:

```text
Nguon: /home/dungnd/Downloads/Pngtree vector of a detailed black dragon PNG
Kieu xuat: raster line-art, bo nen alpha, threshold=80
Kich thuoc khac: 180.0 x 179.4 mm, can giua ban
Vung toa do: X74.0..253.7, Y52.3..231.4
Scan step: 0.30 mm
Laser: M3 S0 de ha Z/arm, moi net khac G1 S650 F900; binary mode coi S650 la full ON
Preview mask: test_gcode/dragon_180mm_lineart_preview.png
```

File `ultra_complex_cyber_mandala_15mm_onoff.gcode`:

```text
Kieu xuat: hinh procedural cuc phuc tap, mandala/co khi/xoan/lace/hatch
Kich thuoc khac: 15 x 15 mm, can giua ban
Vung toa do: X156.532..171.468, Y134.532..149.468
Laser: M3 S0 de ha Z/arm, cac net G1 dung S1000 F900, chi on/off
Preview: test_gcode/ultra_complex_cyber_mandala_15mm_preview.png
```

File `viet_nam_flag_150x60_onoff.gcode`:

```text
Kieu xuat: chu VIET NAM bang net don + co Viet Nam o ben phai
Kich thuoc khac: khoang 144 x 53 mm, nam trong X90..234 Y110..163
Laser: M3 S0 de ha Z/arm, cac net G1 dung S1000 F900, chi on/off
Preview: test_gcode/viet_nam_flag_150x60_preview.png
```

File `hoang_dau_huyen_times_thin_80x60_onoff.gcode`:

```text
Kieu xuat: chu HOANG font Times New Roman-compatible, net outline mong, co dau huyen tren chu A
Khung lam viec: 80 x 60 mm, can giua tai X164 Y142
Vung khac thuc te: X128..200 Y133.229..150.771, khong phong to vua khit khung
Laser: M3 S0 de ha Z/arm, cac net G1 dung S1000 F900, chi on/off
Preview: test_gcode/hoang_dau_huyen_times_thin_80x60_preview.png
```

File `vietnam_map_sticker_80x60_onoff.gcode`:

```text
Kieu xuat: sticker ban do Viet Nam, sao 5 canh, hatch than ban do, chu VIETNAM nghieng phia duoi
Nguon outline: Natural Earth 10m admin-0 countries, Vietnam
Khung lam viec: 80 x 60 mm, can giua tai X164 Y142
Vung khac thuc te: X133.401..186.866 Y113.756..170.958
Laser: M3 S0 de ha Z/arm, cac net G1 dung S1000 F900, chi on/off
Preview: test_gcode/vietnam_map_sticker_80x60_preview.png
```

File `son_o_horn_times_thin_80x60_onoff.gcode`:

```text
Kieu xuat: chu SON font Times New Roman-compatible, net outline mong, co dau moc tren chu O
Khung lam viec: 80 x 60 mm, can giua tai X164 Y142
Vung khac thuc te: X134..194 Y130.629..153.371
Laser: M3 S0 de ha Z/arm, cac net G1 dung S1000 F900, chi on/off
Preview: test_gcode/son_o_horn_times_thin_80x60_preview.png
```

Ghi chu file test nay:

```text
- File goc user dua co tam thiet ke tai X50 Y50, vung ve X10..90 va Y20..74.
- De ve o giua ban, da cong offset X+114 va Y+92 de tam ve nam tai X164 Y142.
- G0 X0 Y0 o cuoi file goc da doi thanh G0 X164 Y142 de khong lao ve sat home sau khi test.
- M3/M5 hien dieu khien laser that tren PD14; can che/bao ve mat khi chay file test.
- File complex_mandala_z_mid.gcode co dong G0 Z-2.700 dau job; voi firmware moi dong nay duoc accept nhung Z chi ha khi gap M3/M4.
```

## Web upload G-code qua ESP32

ESP32 hien co web upload:

```text
URL: http://192.168.1.140/
Status plain text: http://192.168.1.140/status
Stop job: nut Stop job tren web se gui ! xuong STM32
Job %: hien tren web va LCD, tinh theo byte ESP32 da doc/gui trong file G-code hien tai
```

UI hien tai:

```text
- Khong con meta refresh 3 s, tranh mat file dang chon khi user thao tac cham.
- Upload bang JavaScript/XHR, co progress bar va thong bao loi.
- Co o keo-tha/chon file, card Sent/OK/Skip/Job %, status poll moi 1 s.
- Smoke test upload file chi co G90: state=done, sent=1, ok=1.
```

Quy trinh:

```text
1. Mo web ESP32.
2. Upload file .gcode/.nc/.gc.
3. ESP32 luu file vao SPIFFS tai /job.gcode.
4. Sau khi upload xong, ESP32 tu dong stream tung dong sang STM32 UART2.
5. ESP32 doi STM32 tra ok moi gui dong tiep theo.
6. Neu STM32 tra error hoac qua timeout 120 s khong co ok, ESP32 dung job va gui !.
```

Ket qua test web upload:

```text
File test:
G91
G1 X2 Y0 F600
G1 X-2 Y0 F600
G90

ESP32 status sau test:
state=done
message=job complete
sent=4
ok=4

STM32 log:
G1 line Xsteps=322 Ysteps=0 period_us=621
line done, drivers disabled
ok
G1 line Xsteps=322 Ysteps=0 period_us=621
line done, drivers disabled
ok
POS X=164.000mm Y=142.000mm, ve lai center
```

Ghi chu:

```text
- File upload nen la G-code chuan G21/G90/G91/G17/G0/G1/G2/G3/M3/M4/M5/M2.
- Khong dua lenh debug rieng nhu s/x/y/z vao file, vi do khong phai G-code job.
- Moi dong hien gio toi da 95 ky tu do buffer STM32/ESP32 dang de nho cho an toan.
- Laser khong con forced OFF: M3/M4 arm laser, G1/G2/G3 bat laser neu S>0, G0/M5/!/d/reset tat output laser.
```

Neu driver dang full-step:

```text
200 pulse = 1 vong motor
400 pulse = 2 vong motor
```

Neu driver dang 1/16 microstep:

```text
3200 pulse = 1 vong motor
400 pulse  = 1/8 vong motor
```

Khi homing, firmware in ra UART dang:

```text
STOP X/CH2 pulses=<so_pulse> limits L1=... L2=... L3=... L4=...
HIT LIMITn -> X/CH2
```

Dung so pulse tu vi tri ban dau den home de tinh khoang cach moi vong:

```text
mm_per_motor_rev = travel_mm * pulses_per_motor_rev / pulses_to_home
```

Vi du:

```text
full-step: pulses_per_motor_rev = 200
1/16 microstep: pulses_per_motor_rev = 3200
1/32 microstep: pulses_per_motor_rev = 6400
```

## UART debug qua ESP32

Ket noi ESP32 UART2 voi STM32 USART1:

```text
ESP32-C6 GPIO16 TX -> STM32 RXD / PB7
ESP32-C6 GPIO17 RX -> STM32 TXD / PB6
ESP32 GND        -> STM32 GND
Baud             = 115200 8N1
```

Sketch bridge ESP32 nam tai:

```text
tools/esp32_uart_bridge
```

Board ESP32 thuc te dang dung:

```text
Da chuyen sang ESP32-C6 Super Mini, target gan voi ESP32-C6-DevKitM-1.
Board C6 co LED RGB dia chi tren GPIO8.
Dang dung UART ESP32:
  GPIO16 TX -> STM32 PB7 RXD
  GPIO17 RX -> STM32 PB6 TXD
```

Ghi chu voi ESP32-C6 Super Mini:

```text
- PlatformIO USB: pio run -e esp32c6_supermini_usb -t upload
- PlatformIO OTA: pio run -e esp32c6_supermini_ota -t upload
- RGB status LED GPIO8:
  Green        = WiFi chinh connected, idle
  Purple blink = AP cau hinh dang bat, chua connect WiFi chinh
  Blue         = job dang chay
  Orange blink = dang update firmware STM32 / bootloader
  Red blink    = job/firmware error
  Yellow       = job stopped
- Target C6 khong dung LCD/touch. Endpoint /lcd-test se test RGB GPIO8.
- GPIO8 la strapping pin cua ESP32-C6, chi dung LED onboard, khong keo ngoai lung tung khi reset.
- Tren ESP32-C6, Arduino Serial2 la LP-UART va doi GPIO4/5. Firmware dung HardwareSerial(1) de giu UART STM32 tren GPIO16/17.
- Nut BOOT tren C6 la GPIO9. Giu BOOT 3 giay khi firmware dang chay -> xoa WiFi config trong NVS, restart vao portal CNC-Laser-Setup.
```

Ghi chu voi board man hinh ESP32 cu:

```text
- Diymore ESP32 WiFi + Bluetooth 2.8 inch LCD TFT 240x320 touch, ESP32-WROOM.
- Loai nay gan voi dong ESP32-2432S028R / "Cheap Yellow Display" nhung co nhieu bien the pinout.
- Board co the dung LCD SPI 240x320 va touch dien tro, thuong la ILI9341 + XPT2046 nhung can xac nhan bang pinout/anh mat sau board.
- Pinout tham khao CYD pho bien: TFT MISO=12, MOSI=13, SCLK=14, CS=15, DC=2, BL=21; touch IRQ=36, MOSI=32, MISO=39, CLK=25, CS=33.
- RGB LED tren CYD pho bien dung GPIO4/16/17 active-low; vi GPIO16/17 dang lam UART STM32 nen khong dung RGB LED xanh/lam neu chua doi day UART.
- Neu sau nay lam UI local tren LCD, giu UART 16/17 nhu hien tai va chi them TFT/touch theo pinout da xac nhan.
```

ESP32 da co OTA qua WiFi:

```text
IP tinh       = 192.168.1.140
Telnet debug  = nc 192.168.1.140 23
ESP32 OTA cu  = pio run -e esp32dev_ota -t upload
ESP32-C6 USB  = pio run -e esp32c6_supermini_usb -t upload
ESP32-C6 OTA  = pio run -e esp32c6_supermini_ota -t upload
ESP32-C6 HTTP OTA page = http://192.168.1.140/esp32-ota
ESP32-C6 HTTP OTA curl = curl -F 'file=@tools/esp32_uart_bridge/.pio/build/esp32c6_supermini_ota/firmware.bin;filename=firmware.bin' http://192.168.1.140/esp32-ota
```

WiFi config/fallback AP:

```text
Khi ESP32 connect duoc WiFi chinh:
- Web console: http://192.168.1.140/
- WiFi setup:  http://192.168.1.140/wifi

Khi ESP32 khong connect duoc WiFi chinh:
- ESP32 tu phat AP: CNC-Laser-Setup
- Mat khau AP: cnclaser
- Vao trang cau hinh: http://192.168.4.1/wifi
- Captive portal se day cac URL la ve /wifi khi AP dang bat.
- AP fallback chi de cau hinh WiFi/IP, khong dung de upload G-code, OTA, telnet debug, jog, home, center hay chay job.

Trang /wifi:
- Quet va chon SSID WiFi ngoai.
- Nhap mat khau.
- Chon DHCP hoac static IP.
- Luu vao NVS Preferences namespace "wifi", sau do ESP32 restart.
- Nut Clear Saved WiFi xoa cau hinh da luu va restart vao setup portal.
- Giu BOOT 3 giay tren ESP32-C6 cung xoa cau hinh da luu va restart vao setup portal.

/status co them:
- wifi_mode=sta / sta_ap / ap_config / offline
- wifi_ssid=<ssid dang dung>
- sta_ip=<ip khi da connect WiFi ngoai>
- ap_ip=<ip AP fallback khi dang phat AP>
- wifi_reset_button_gpio=9 tren ESP32-C6
- wifi_force_setup=1 khi dang bi ep vao setup portal sau khi clear config
- ota_active=1 khi ESP32 dang tu update firmware
- esp32_ota=<trang thai OTA ESP32 gan nhat>

Ghi chu:
- Khi dang o AP fallback, ESP32 khong retry STA nen AP cau hinh on dinh hon.
- Sau khi Save & Restart, ESP32 reboot va thu connect WiFi chinh.
- OTA/telnet/web console chi dung khi ESP32 da connect vao WiFi chinh; neu mat WiFi chinh, vao AP fallback de sua cau hinh truoc.
- Neu `pio ... esp32c6_supermini_ota -t upload` bi timeout giua chung, dung HTTP OTA `/esp32-ota` hoac curl nhu tren.
```

## STM32 bootloader / update qua ESP32

Da them bootloader rieng cho STM32:

```text
Bootloader project = CNC_Laser_Bootloader
Bootloader flash   = 0x08000000..0x0800FFFF, 64 KiB dau flash
App CNC flash      = 0x08010000..0x0807FFFF, 448 KiB con lai
App vector offset  = 0x00010000
Boot wait window   = 5 s moi lan reset, sau do tu nhay vao app neu app hop le
```

Build:

```text
cd CNC_Laser_Bootloader
cmake --preset Debug
cmake --build --preset Debug

cd ../CNC_Laser_Fw
cmake --build --preset Debug
```

Nap lan dau bang ST-LINK:

```text
st-flash write CNC_Laser_Bootloader/build/Debug/CNC_Laser_Bootloader.bin 0x8000000
st-flash write CNC_Laser_Fw/build/Debug/CNC_Laser_Fw.bin 0x8010000
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init; reset run; shutdown"
```

Sau khi da co bootloader, update app STM32 qua ESP32:

```text
Web:  http://192.168.1.140/firmware
Curl: curl -F 'file=@CNC_Laser_Fw/build/Debug/CNC_Laser_Fw.bin;filename=CNC_Laser_Fw.bin' http://192.168.1.140/fw-upload
```

Can nho:

```text
- File upload qua /firmware phai la app bin da link tai 0x08010000.
- Khong upload CNC_Laser_Bootloader.bin qua /firmware.
- Neu muon sua chinh bootloader, van can ST-LINK de nap bootloader vao 0x08000000.
- Neu app STM32 bi treo khong nhan lenh B, co the power-cycle STM32 trong luc ESP32 dang retry bootloader; neu van khong bat duoc boot window thi dung ST-LINK de cuu.
- ST-LINK tren may hien khong noi NRST, nen st-flash reset khong keo reset duoc; dung OpenOCD reset qua SWD nhu lenh tren.
```

Giao thuc UART ESP32 <-> bootloader STM32:

```text
Bootloader in: BL READY APP=0x08010000 MAX=458752
PING           -> PONG
BOOT           -> BOOT APP neu app hop le
FWUP size crc  -> ERASE, READY DATA, sau do yeu cau tung block:
NEXT offset len
ESP32 gui len byte raw
Lap lai den het file
OK FW neu CRC32 dung va vector app hop le
```

Ket qua test thuc te:

```text
ESP32 upload app qua /fw-upload:
firmware update done, bytes=56772
HTTP 200

STM32 bootloader log:
BL READY APP=0x08010000 MAX=458752
ERASE
READY DATA
NEXT 0 256
...
NEXT 56576 196
OK FW
BOOT APP

Sau do app boot lai, home Z/X/Y, va center X/Y dong thoi thanh cong.
```

## Motor va driver

Motor dang dung:

```text
Model: 17PM-K049EP10CN
Gan dung ho NMB 17PM-K049
Step angle: 1.8 deg
Full step: 200 step/vong
Dong tham khao: khoang 1.0 A/phase
```

Driver:

```text
DRV8825 carrier
VMOT can cap nguon motor rieng, toi thieu khoang 8.2 V
Logic 3.3 V tu STM32 dieu khien duoc STEP/DIR/ENA
ENA active-low: keo LOW de enable, keo HIGH de disable
```

Chinh dong neu Rsense = 0.1 ohm:

```text
I_limit = Vref * 2
Vref 0.35 V -> I_limit 0.7 A
Vref 0.50 V -> I_limit 1.0 A
```

## Checklist neu motor chi nhich hoac khong chay

```text
1. Kiem tra VMOT +12 V vao tung DRV8825.
2. Kiem tra GND motor power va GND MCU chung nhau.
3. Kiem tra Vref cua tung DRV8825, bat dau 0.35-0.50 V.
4. Gui enable/hoac trong auto test, so truc motor co cung lai khong.
5. Do STEP tai chan driver co xung 8 kHz voi X/Y/Z khong.
6. Kiem tra day motor dung cap coil A/B, khong cam nham center tap neu motor 6 day.
7. Neu dang microstep 1/16, 400 pulse chi la 1/8 vong.
```

## Van de da gap va cach xu ly

### Motor khong chay / chi nhich

```text
Hien tuong:
- Ban dau motor gan nhu khong thay di chuyen.
- Khi giam toc do xuong 1 kHz thi thay motor chi nhich mot chut.

Nguyen nhan / bai hoc:
- Chua chac mapping truc va driver.
- 400 pulse khong phai luc nao cung la 1 vong tai truc neu driver dang microstep.
- Voi 1/16 microstep, 400 pulse chi bang 1/8 vong motor.
- Can kiem tra VMOT, GND chung, Vref driver, cap coil A/B.

Cach xu ly:
- Viet firmware test tung channel, ban dau chay cham 1 kHz de quan sat.
- Xac dinh duoc CH1=Y, CH2=X, CH3=Z.
- Tang toc do dan len 2 kHz, 4 kHz, hien tai 8 kHz khi da biet he co chay.
```

### Mapping truc bi lech so voi ten channel

```text
Hien tuong:
- Khi reset firmware scan lan luot cac channel, truc thuc te chay theo thu tu Y, X, Z.

Ket luan:
- Y = CH1 = ENA1 / STEP1 / DIR1
- X = CH2 = ENA2 / STEP2 / DIR2
- Z = CH3 = ENA3 / STEP3 / DIR3
- CH4 chua dung
```

### Chieu quay / DIR polarity chua ro

```text
Hien tuong:
- Can biet DIR=0 hay DIR=1 ung voi chieu duong cua tung truc.

Ket luan da do:
- X: DIR=0 -> X+, DIR=1 -> X-
- Y: DIR=0 -> Y-, DIR=1 -> Y+
- Z: DIR=0 -> Z- / di xuong, DIR=1 -> Z+ / di len

Bai hoc:
- Khong doan theo ten pin tren board, phai test thuc te tung truc.
```

### Driver enable active-low

```text
Hien tuong:
- Can dam bao khi boot/reset driver khong bi enable ngoai y muon.

Ket luan:
- ENA cua DRV8825 active-low: LOW = enable, HIGH = disable.

Cach xu ly:
- GPIO init dat ENA1/ENA2/ENA3/ENA4 = HIGH de disable mac dinh.
- Khi di chuyen moi keo ENA cua truc do xuong LOW.
- Khi homing xong, jog xong, center xong thi tat driver.
```

### Limit switch active-low va mapping home

```text
Hien tuong:
- Khi binh thuong LIMIT doc la 1, khi cham cong tac doc la 0.
- Can tim cong tac nao la cua truc nao.

Ket luan:
- LIMIT1 = Y home/min
- LIMIT2 = X home/min
- LIMIT3 = Z home/top
- LIMIT4 chua dung

Cach xu ly:
- Homing dung khi limit doi trang thai so voi baseline.
- Neu truc da nam tren home limit tu dau thi skip truc do, khong day tiep vao cong tac.
```

### Z homing bi thieu hanh trinh

```text
Hien tuong:
- Z chi chay duoc mot nua, lenh home Z dung o max pulse nhung chua cham LIMIT3.

Nguyen nhan:
- Gioi han pulse cho Z ban dau qua thap.

Cach xu ly:
- Tang max Z len de test full hanh trinh.
- Do duoc Z/CH3 home/up -> LIMIT3 tai 40377 pulse voi travel Z = 5.4 mm.
- Dat max Z homing hien tai = 45000 pulse.
```

### LED nhap nhay sai khi motor dung im

```text
Hien tuong:
- Khi motor chay, LED nhap nhay dung gan 1 Hz.
- Khi dung im, LED nhap nhay voi tan so khac hoac kho nhin thay.

Nguyen nhan:
- LED truoc do duoc update trong main loop.
- UART receive tung co timeout/blocking, lam main loop khong update deu.
- Pulse LED ngan 10 ms / 3 ms / 2 ms de bi bo lo neu task khong chay dung nhip.

Cach xu ly:
- Doi UART poll sang non-blocking.
- Dua LED timing vao callback TIM14 1 ms.
- Hien tai LED sang 2 ms moi chu ky 1000 ms, doc lap motor/UART idle.
```

### STEP timer / TIM1 chua phai motion engine that

```text
Hien tuong:
- CubeMX co cau hinh TIM1 PWM tren cac chan STEP PE9/PE11/PE13/PE14.
- Firmware debug hien tai van pulse STEP bang GPIO + DWT delay.
- Da them noi suy XYZ bang TIM5 interrupt, DDA/Bresenham cho X/Y/Z.

Bai hoc:
- TIM1 hien chua duoc dung lam motion planner.
- STEP job G0/G1 khong con busy-wait ca hanh trinh trong main loop.
- Startup/home calibration da doi sang home X/Y/Z dong thoi; startup center chi X/Y.
- Van can planner queue, acceleration, UART ring buffer de thanh firmware khac laser that.
```

### Toc do STEP da tang nhieu lan

```text
Qua trinh:
- Giam xuong 1 kHz de thay motor nhich va xac nhan driver.
- Tang X/Y len 4 kHz.
- Tang Z len 4 kHz.
- Hien tai tang ca X/Y/Z len 8 kHz.

Thong so hien tai:
- STEP high = 10 us
- STEP period = 125 us tai 8 kHz
- STEP low = 115 us
```

### ESP32 WiFi sai mode ban dau

```text
Hien tuong:
- Ban dau ESP32 phat WiFi/AP, nhung yeu cau thuc te la bat WiFi ngoai.
- Sau do can them che do cau hinh WiFi de doi SSID/mat khau khong can sua code.

Cach xu ly:
- Mac dinh ESP32 connect WiFi ngoai SSID ChatGPT voi IP tinh 192.168.1.140.
- Neu connect fail thi bat fallback AP CNC-Laser-Setup / cnclaser.
- Vao http://192.168.4.1/wifi de chon WiFi, mat khau, DHCP/static IP.
- Cau hinh luu vao NVS, restart xong ESP32 dung WiFi moi.
- AP fallback chi dung lam config portal, khong chay web console/upload G-code/jog/OTA/telnet qua AP vi ket noi kem va de timeout.
- Khi o AP config-only, cac endpoint may chay that se tra 503.
- Giu BOOT tren ESP32-C6 trong 3 giay de xoa config WiFi hien tai va ep may vao setup portal.
- Telnet debug khi o WiFi chinh: nc 192.168.1.140 23.
- OTA target khi o WiFi chinh:
  - ESP32 cu: pio run -e esp32dev_ota -t upload
  - ESP32-C6 Super Mini: pio run -e esp32c6_supermini_ota -t upload
  - ESP32-C6 fallback HTTP: http://192.168.1.140/esp32-ota
```

### ESP32-C6 PlatformIO espota bi timeout giua chung

```text
Hien tuong:
- Ping toi 192.168.1.140 on dinh, 0% packet loss.
- pio run -e esp32c6_supermini_ota -t upload authenticate OK voi password OTA.
- Upload qua espota bi dut giua chung: co lan Broken pipe/Receive Failed, co lan timed out sau vai KB.

Cach xu ly:
- Van giu ArduinoOTA de sau nay thu lai khi mang/driver on hon.
- Da them HTTP OTA qua WiFi chinh: http://192.168.1.140/esp32-ota.
- Test thuc te HTTP OTA da upload du 1243344 byte firmware.bin, ESP32 restart va len lai IP 192.168.1.140.
- Lenh test HTTP OTA:
  curl -H 'Expect:' -F 'file=@tools/esp32_uart_bridge/.pio/build/esp32c6_supermini_ota/firmware.bin;filename=firmware.bin' http://192.168.1.140/esp32-ota
```

### ESP32-C6 UART voi STM32 bi nguoc TX/RX

```text
Hien tuong:
- ESP32 web/telnet len binh thuong qua 192.168.1.140.
- Gui `s` qua telnet bridge khong thay STM32 tra STATUS.
- May tinh khong cam ESP/STM qua USB, chi test qua WiFi cua ESP32 va nguon chung.

Cach xu ly:
- Da dao mapping UART trong env ESP32-C6:
  ESP32-C6 GPIO16 TX -> STM32 RXD / PB7
  ESP32-C6 GPIO17 RX -> STM32 TXD / PB6
- Sau khi nap HTTP OTA, /status bao:
  stm32_uart_tx_gpio=16
  stm32_uart_rx_gpio=17
- Test qua telnet `s` da nhan:
  STATUS limits L1=1 L2=1 L3=0 L4=1
  POS X=0.000mm Y=0.000mm Zdown=0.000mm ...
```

### Telnet ESP32 bao Bridge busy

```text
Hien tuong:
- Telnet moi vao ESP32 bi bao: Bridge busy: one Telnet client is already connected.
- User khong mo log nao.

Nguyen nhan thuc te lan nay:
- Tren may local con process nc cu giu ket noi:
  192.168.1.14:<port> -> 192.168.1.140:23

Cach xu ly tam thoi:
- Dung ss -tnp de tim process nc.
- Kill process nc cu.
- Mo lai nc 192.168.1.140 23 thi vao duoc.

Cach xu ly trong code ESP32:
- Da sua bridge de telnet client moi se dong client cu va chiem quyen log.
- Nhu vay lan sau neu socket bi ket thi chi can connect lai, khong bi Bridge busy nua.
```

### Telnet bang nc trong PTY can Enter de gui lenh

```text
Hien tuong:
- Gui ky tu s rieng le trong PTY chi thay echo, STM32 chua tra STATUS.

Nguyen nhan:
- nc chay trong terminal/PTY co the dang theo line discipline, ky tu chua flush den khi Enter.

Cach xu ly:
- Gui s roi Enter, firmware bo qua \r/\n va xu ly s binh thuong.
- Sau Enter nhan duoc:
  STATUS limits L1=1 L2=1 L3=0 L4=1
```

### PlatformIO co 2 core, lenh pio mac dinh bi cu

```text
Hien tuong:
- Chay pio run -e esp32dev_ota -t upload bi loi:
  Obsolete PIO Core v4.3.4 is used
  Error: Unknown development platform 'espressif32'

Nguyen nhan:
- /usr/bin/pio dang la PlatformIO core cu.
- Ban dung nam o:
  /home/dungnd/.platformio/penv/bin/pio

Cach xu ly:
- Dung lenh:
  /home/dungnd/.platformio/penv/bin/pio run -e esp32dev_ota -t upload
```

### STM32 reset se tu dong chay may

```text
Hien tuong:
- Sau khi nap/reset STM32, firmware auto-home Z/X/Y roi dua X/Y ra giua.

Bai hoc an toan:
- Truoc khi reset/nap phai dam bao may khong vuong hanh trinh.
- Khi can dung khan trong motion, gui ! hoac d qua UART.
- Firmware luon tat LASER va MOTOR775 khi boot.
```

### Firmware hien tai chua phai firmware khac laser 2D

```text
Hien trang:
- Firmware hien tai la ban debug/homing/jog.
- Da co G-code parser toi thieu cho G21/G90/G91/G0/G1/M3/M4/M5.
- Da co noi suy dong bo X/Y bang TIM5 interrupt + DDA/Bresenham.
- Chua co motion planner queue.
- Chua co acceleration/deceleration.
- Chua co laser PWM theo S value.
- LASER hien la GPIO PD14 on/off, mac dinh OFF.
- M3/M4 S>0 bat laser digital, M5/S0/!/d/reset/home tat laser.
- G0 laser off, G1/G2/G3 laser on trong luc motion neu da M3/M4.

Ket luan:
- Muon khac 2D that can lam them lop GRBL-like.
```

## Quy trinh khac laser 2D can huong toi

### Workflow tu phan mem ben ngoai

```text
1. Ve/import file tren PC:
   - SVG/DXF cho vector.
   - PNG/JPG/BMP cho anh raster.

2. CAM/GUI tao G-code:
   - LightBurn, LaserGRBL, LaserWeb, hoac tool rieng.
   - Chon mode: Line, Fill, Image.
   - Chon speed, power, line interval, so pass.

3. Gui G-code den controller:
   - Qua USB/UART/WiFi bridge.
   - Controller doc tung dong, parse modal state, dua len motion queue.

4. Controller chay job:
   - Home truoc de biet machine coordinate.
   - Di G0 toi diem bat dau voi laser off.
   - Chay G1/G2/G3 voi feed F va laser power S.
   - Tat laser khi travel, pause, alarm, limit, reset.
```

### Cac mode khac 2D

```text
Line / vector:
- Laser trace theo duong vector.
- Dung cho cat, vien, outline.
- G-code chu yeu la G0 travel, G1 cut/mark, M3/M4/M5 hoac inline S.

Fill:
- Dung cho vung vector dong kin.
- Dau laser scan qua lai tung dong nhu may in.
- Line interval quyet dinh mat do scan.
- Can overscan hai dau de giu toc do deu, tranh chay dam o mep.

Image / raster:
- Anh duoc doi thanh cac dong scan.
- Do sang pixel doi thanh power S hoac dithering 1-bit.
- Can bidirectional compensation neu khac ca hai chieu.
```

### G-code subset nen ho tro truoc

```text
Lenh can co toi thieu:
- G21: mm mode
- G90/G91: absolute/relative
- G0 X Y: travel, laser off
- G1 X Y F S: move co feed, laser power theo S
- M3 S: laser constant power mode
- M4 S: laser dynamic power mode, ve sau moi can
- M5 hoac S0: laser off
- F: feed rate mm/min
- S: laser power 0..Smax
- ?: status
- !: feed hold / emergency stop
- ~: resume, ve sau
- $H: homing, ve sau neu muon giong GRBL

Ban dau co the bo qua:
- G2/G3 arc, co the de CAM convert arc thanh line segment.
- EEPROM settings $100..$132, co the hard-code truoc.
```

### Motion engine can viet lai

```text
Can thay code delay blocking bang:
- Position hien tai theo step va mm.
- Steps/mm:
  X = 52738 / 328 = 160.79 step/mm
  Y = 45440 / 284 = 160.00 step/mm
  Z = 40377 / 5.4 = 7477.22 step/mm
- Queue block motion.
- DDA/Bresenham de dong bo X/Y tren cung mot line.
- Timer interrupt phat STEP deu.
- Acceleration/deceleration de khong mat buoc.
- Limit hard stop.
- Planner biet G0 laser off va G1 laser co the on.
```

### Laser PWM can lam

```text
Can co:
- Laser mac dinh OFF khi boot/reset/error.
- PWM hardware rieng cho LASER, khong dung blocking GPIO.
- Map S value sang duty cycle.
- Smax nen chon 1000 de hop voi GRBL/LightBurn pho bien.
- Khi G0, pause, alarm, limit: duty = 0 ngay lap tuc.

Goi y phan cung:
- LASER hien tai la PD14.
- Tren STM32F407, PD14 thuong co the dung TIM4_CH3 AF2, phu hop lam PWM laser.
- Khong nen dung TIM1 cho laser vi TIM1 dang lien quan cac chan STEP PE9/PE11/PE13/PE14.
```

### An toan va workflow van hanh

```text
Trinh tu an toan nen co:
1. Boot: laser off, motor spindle off, driver disabled.
2. Home Z/X/Y.
3. Move ve origin/center hoac cho user set work origin.
4. Chi allow laser on khi da unlock/homed.
5. G0 luon tat laser.
6. Limit hit -> tat laser, tat driver, vao ALARM.
7. Watchdog/timeout UART job -> tat laser.
8. Nut emergency stop hoac lenh ! -> tat laser ngay.

Can lam material test:
- Bang grid speed/power.
- Tim power toi thieu de danh dau vat lieu.
- Tim speed/power cho cat neu dung cat.
- Khac raster can test line interval va overscan.
```

### Huong lam firmware tiep theo

```text
Buoc 1:
- Giu debug commands hien tai.
- Them PWM laser test lenh m/M:
  m Sxxx -> set laser duty cuc thap de test
  M -> laser off

Buoc 2:
- Them move mm:
  G0/G1 don gian chi X/Y, absolute G90, mm G21.
  Chua can acceleration, chay cham de test.
  Trang thai: da lam.

Buoc 3:
- Viet DDA timer interrupt cho XY line.
- Them feed rate F va steps/mm chuan.
  Trang thai: da lam ban dau bang TIM5, chua co acceleration/queue.

Buoc 4:
- Them laser inline S cho G1.
- Dam bao G0/S0/M5 tat laser khong pause gay vet chay.

Buoc 5:
- Them acceleration, planner queue, soft limit 328x284, alarm state.

Buoc 6:
- Cho LightBurn/LaserGRBL gui G-code qua ESP32 UART bridge.
```

## Nguon tham khao laser 2D

```text
GRBL laser mode:
https://github.com/gnea/grbl/wiki/Grbl-v1.1-Laser-Mode

LightBurn GRBL setup:
https://docs.lightburnsoftware.com/legacy/CommonGrblSetups

LightBurn Fill mode:
https://docs.lightburnsoftware.com/2.1/Reference/CutSettingsEditor/FillMode/

LightBurn Image mode:
https://docs.lightburnsoftware.com/legacy/UI/CutSettings/CutSettings-Image

LaserGRBL project:
https://github.com/arkypita/LaserGRBL

Marlin M3 laser/spindle command:
https://marlinfw.org/docs/gcode/M003.html
```

## ESP32 web/LCD manual UI

```text
ESP32 board hien tai: ESP32-C6 Super Mini, khong co LCD, co RGB LED GPIO8.
ESP32 board LCD cu/legacy: Diymore/ESP32 2.8 inch TFT 240x320 touch, gan voi nhom ESP32-2432S028R / E32R28T.

Pin LCD theo tai lieu:
- TFT ILI9341: MISO GPIO12, MOSI GPIO13, SCLK GPIO14, CS GPIO15, DC GPIO2, RST share EN/-1
- Backlight: GPIO21, HIGH = on
- Touch XPT2046: IRQ GPIO36, MOSI GPIO32, MISO GPIO39, CLK GPIO25, CS GPIO33
- RGB LED tren mot so board co the dung GPIO22/16/17 hoac GPIO4/16/17, khong dung GPIO16/17 vi dang lam UART2 voi STM32.

ESP32 firmware hien tai:
- Web console: http://192.168.1.140/
- Status plain text: http://192.168.1.140/status
- WiFi setup: http://192.168.1.140/wifi
- STM32 firmware update: http://192.168.1.140/firmware
- ESP32-C6 HTTP OTA: http://192.168.1.140/esp32-ota
- OTA ESP32-C6: pio run -e esp32c6_supermini_ota -t upload
- OTA ESP32 LCD cu: pio run -e esp32dev_ota -t upload
- Telnet UART debug: nc 192.168.1.140 23
- C6 RGB test endpoint: POST http://192.168.1.140/lcd-test
- LCD legacy test endpoint: POST http://192.168.1.140/lcd-test

Manual UI:
- Upload + Start: upload file G-code va stream ngay.
- Run Stored: chay lai file /job.gcode da upload gan nhat.
- Home + Center: gui H roi C, home tat ca truc xong dua X/Y ve giua.
- Center: gui C, dua X/Y ve X164 Y142 dong thoi, Z giu nguyen.
- Jog X/Y: gui lenh STM32 J X.. Y.. F3000, jog tuong doi va khong ha Z.
- Z Up/Z Down: gui lenh Z/z debug 400 pulse.
- Work + Laser: gui M3 S1200, STM32 ha Z ve job-mid roi bat laser PD14.
- Laser Off: gui M5, tat laser.
- Stop: gui ! va dung job tren ESP32.

Van de da gap:
- Man hinh khong thay sang sau ban dau khi dung LovyanGFX Light_PWM. Da doi sang keo cung GPIO21 HIGH trong setupLcdUi va moi lan redraw.
- Neu /status bao lcd=ready, backlight_gpio=21 va /lcd-test tra ok nhung man van toi thui:
  1. Do GPIO21/P3 xem co len 3.3V khong.
  2. Kiem tra board co dung bien the GPIO21 backlight khong.
  3. Kiem tra nguon 5V/USB-C, cap FFC LCD/backlight, hoac thu firmware demo goc cua board.
```
