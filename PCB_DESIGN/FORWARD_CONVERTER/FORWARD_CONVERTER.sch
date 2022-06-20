EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "DC-DC FORWARD CONVERTER"
Date "10.06.2022"
Rev "V1"
Comp "ANKARA YILDIRIM BEYAZIT UNIVERSITY"
Comment1 "Omer Karslioglu"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 62A39DB5
P 1450 1750
F 0 "J1" H 1368 1967 50  0000 C CNN
F 1 "Vin" H 1368 1876 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 1450 1750 50  0001 C CNN
F 3 "~" H 1450 1750 50  0001 C CNN
	1    1450 1750
	-1   0    0    -1  
$EndComp
$Comp
L Transistor_FET:IRF540N Q1
U 1 1 62A3D098
P 3350 2700
F 0 "Q1" H 3554 2746 50  0000 L CNN
F 1 "IRFZ44N" H 3554 2655 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 3600 2625 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf540n.pdf" H 3350 2700 50  0001 L CNN
	1    3350 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 1850 3450 1850
Wire Wire Line
	3450 1850 3450 2300
$Comp
L Device:R R5
U 1 1 62A4351E
P 2950 2950
F 0 "R5" H 3020 2996 50  0000 L CNN
F 1 "47K" H 3020 2905 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0411_L9.9mm_D3.6mm_P12.70mm_Horizontal" V 2880 2950 50  0001 C CNN
F 3 "~" H 2950 2950 50  0001 C CNN
	1    2950 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 2700 2950 2700
Wire Wire Line
	2950 2700 2950 2800
Wire Wire Line
	2950 3100 2950 3200
Wire Wire Line
	2950 3200 3450 3200
Wire Wire Line
	3450 3200 3450 2900
Wire Wire Line
	3450 3200 3450 3450
Wire Wire Line
	3450 3450 1800 3450
Connection ~ 3450 3200
Text GLabel 2200 2700 0    50   Input ~ 0
GATE
Wire Wire Line
	1650 1850 1800 1850
Wire Wire Line
	1800 1850 1800 3450
Wire Wire Line
	1650 1750 3550 1750
$Comp
L Diode:1N4007 D1
U 1 1 62A46BEC
P 4750 1550
F 0 "D1" H 4750 1333 50  0000 C CNN
F 1 "UF4007" H 4750 1424 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P12.70mm_Horizontal" H 4750 1375 50  0001 C CNN
F 3 "https://www.vishay.com/docs/88755/uf4001.pdf" H 4750 1550 50  0001 C CNN
	1    4750 1550
	-1   0    0    1   
$EndComp
$Comp
L Diode:1N4007 D2
U 1 1 62A48432
P 5250 1850
F 0 "D2" V 5204 1930 50  0000 L CNN
F 1 "UF4007" V 5295 1930 50  0000 L CNN
F 2 "Diode_THT:D_DO-41_SOD81_P12.70mm_Horizontal" H 5250 1675 50  0001 C CNN
F 3 "https://www.vishay.com/docs/88755/uf4001.pdf" H 5250 1850 50  0001 C CNN
	1    5250 1850
	0    1    1    0   
$EndComp
Wire Wire Line
	4900 1550 5250 1550
Wire Wire Line
	5250 1550 5250 1700
Wire Wire Line
	4250 1550 4600 1550
Wire Wire Line
	4250 1650 4450 1650
Wire Wire Line
	4450 1650 4450 2250
Wire Wire Line
	4450 2250 5250 2250
Wire Wire Line
	5250 2250 5250 2000
$Comp
L FORWARD_CONVERTER:FORWARD_TRAFO TF1
U 1 1 62A3FBCC
P 3650 1500
F 0 "TF1" H 3900 1653 50  0000 C CNN
F 1 "FORWARD_TRAFO" H 3900 1573 35  0000 C CNN
F 2 "FORWARD_CONVERTER:FORWARD_TRAFO" H 3800 1500 50  0001 C CNN
F 3 "" H 3800 1500 50  0001 C CNN
	1    3650 1500
	1    0    0    -1  
$EndComp
NoConn ~ 4250 1850
NoConn ~ 4250 1950
$Comp
L pspice:INDUCTOR L1
U 1 1 62A4ABFE
P 5700 1550
F 0 "L1" H 5700 1765 50  0000 C CNN
F 1 "INDUCTOR" H 5700 1674 50  0000 C CNN
F 2 "Inductor_THT:L_Axial_L26.7mm_D14.0mm_P7.62mm_Vertical_Vishay_IHA-104" H 5700 1550 50  0001 C CNN
F 3 "~" H 5700 1550 50  0001 C CNN
	1    5700 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 1550 5450 1550
Connection ~ 5250 1550
$Comp
L Device:CP C1
U 1 1 62A4C096
P 6850 1900
F 0 "C1" H 6968 1946 50  0000 L CNN
F 1 "CP" H 6968 1855 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D12.5mm_P7.50mm" H 6888 1750 50  0001 C CNN
F 3 "~" H 6850 1900 50  0001 C CNN
	1    6850 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 1750 6850 1550
Wire Wire Line
	5250 2250 6850 2250
Wire Wire Line
	6850 2250 6850 2050
Connection ~ 5250 2250
Connection ~ 6850 2250
$Comp
L Device:CP C2
U 1 1 62A4F842
P 7550 1900
F 0 "C2" H 7668 1946 50  0000 L CNN
F 1 "CP" H 7668 1855 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D12.5mm_P7.50mm" H 7588 1750 50  0001 C CNN
F 3 "~" H 7550 1900 50  0001 C CNN
	1    7550 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 1750 7550 1550
Wire Wire Line
	6850 1550 7550 1550
Wire Wire Line
	7550 2250 7550 2050
Wire Wire Line
	6850 2250 7550 2250
$Comp
L Connector:Conn_01x05_Male J2
U 1 1 62A50A0C
P 8300 1900
F 0 "J2" H 8272 1878 50  0000 R CNN
F 1 "Conn_01x05_Male" H 8272 1833 50  0001 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 8300 1900 50  0001 C CNN
F 3 "~" H 8300 1900 50  0001 C CNN
	1    8300 1900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8100 1700 8050 1700
Wire Wire Line
	8050 1700 8050 1800
Wire Wire Line
	8050 1800 8100 1800
Wire Wire Line
	8050 1800 8050 1900
Wire Wire Line
	8050 1900 8100 1900
Connection ~ 8050 1800
Wire Wire Line
	8100 2000 8050 2000
Wire Wire Line
	8050 2000 8050 2100
Wire Wire Line
	8050 2100 8100 2100
Wire Wire Line
	8050 2100 8050 2250
Wire Wire Line
	8050 2250 7550 2250
Connection ~ 8050 2100
Connection ~ 7550 2250
Wire Wire Line
	8050 1700 8050 1550
Wire Wire Line
	8050 1550 7550 1550
Connection ~ 8050 1700
Connection ~ 7550 1550
$Comp
L Driver_FET:IR2110 U2
U 1 1 62A55FFE
P 7550 4350
F 0 "U2" H 7700 4800 50  0000 C CNN
F 1 "IR2110" V 7300 4600 50  0000 C CNN
F 2 "Package_DIP:DIP-14_W7.62mm" H 7550 4350 50  0001 C CIN
F 3 "https://www.infineon.com/dgdl/ir2110.pdf?fileId=5546d462533600a4015355c80333167e" H 7550 4350 50  0001 C CNN
	1    7550 4350
	1    0    0    -1  
$EndComp
NoConn ~ 7850 4050
NoConn ~ 7850 4150
$Comp
L power:+5V #PWR0103
U 1 1 62A5E77F
P 7550 3400
F 0 "#PWR0103" H 7550 3250 50  0001 C CNN
F 1 "+5V" H 7565 3573 50  0000 C CNN
F 2 "" H 7550 3400 50  0001 C CNN
F 3 "" H 7550 3400 50  0001 C CNN
	1    7550 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 3400 7550 3500
Wire Wire Line
	7100 4550 7250 4550
Wire Wire Line
	7100 4550 7100 5000
Wire Wire Line
	7100 5000 7250 5000
Wire Wire Line
	7450 5000 7450 4850
Wire Wire Line
	7450 5000 7550 5000
Wire Wire Line
	7550 5000 7550 4850
Connection ~ 7450 5000
Wire Wire Line
	7250 5100 7250 5000
Connection ~ 7250 5000
Wire Wire Line
	7250 5000 7450 5000
Wire Wire Line
	7850 4450 7950 4450
$Comp
L Device:R R4
U 1 1 62A6C07D
P 2550 2700
F 0 "R4" H 2450 2750 50  0000 C CNN
F 1 "10" H 2450 2650 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0411_L9.9mm_D3.6mm_P12.70mm_Horizontal" V 2480 2700 50  0001 C CNN
F 3 "~" H 2550 2700 50  0001 C CNN
	1    2550 2700
	0    -1   -1   0   
$EndComp
Connection ~ 2950 2700
$Comp
L Diode:1N4007 D3
U 1 1 62A6E508
P 2550 2500
F 0 "D3" H 2550 2717 50  0000 C CNN
F 1 "UF4007" H 2550 2626 50  0000 C CNN
F 2 "Diode_THT:D_DO-41_SOD81_P12.70mm_Horizontal" H 2550 2325 50  0001 C CNN
F 3 "https://www.vishay.com/docs/88755/uf4001.pdf" H 2550 2500 50  0001 C CNN
	1    2550 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 2700 2300 2700
Wire Wire Line
	2700 2700 2850 2700
Wire Wire Line
	2400 2500 2300 2500
Wire Wire Line
	2300 2500 2300 2700
Connection ~ 2300 2700
Wire Wire Line
	2300 2700 2400 2700
Wire Wire Line
	2700 2500 2850 2500
Wire Wire Line
	2850 2500 2850 2700
Connection ~ 2850 2700
Wire Wire Line
	2850 2700 2950 2700
Text GLabel 8000 4650 2    50   Input ~ 0
GATE
Wire Wire Line
	7850 4650 8000 4650
$Comp
L power:+12V #PWR0106
U 1 1 62A7926D
P 8450 4400
F 0 "#PWR0106" H 8450 4250 50  0001 C CNN
F 1 "+12V" H 8465 4573 50  0000 C CNN
F 2 "" H 8450 4400 50  0001 C CNN
F 3 "" H 8450 4400 50  0001 C CNN
	1    8450 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C6
U 1 1 62A7CD86
P 8450 4850
F 0 "C6" H 8568 4896 50  0000 L CNN
F 1 "470uF" H 8568 4805 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D12.5mm_P7.50mm" H 8488 4700 50  0001 C CNN
F 3 "~" H 8450 4850 50  0001 C CNN
	1    8450 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 4600 8450 4700
Wire Wire Line
	8450 5100 8450 5000
Wire Wire Line
	8450 4600 9000 4600
Wire Wire Line
	9000 4600 9000 4700
Wire Wire Line
	8450 5100 8750 5100
Wire Wire Line
	9000 5100 9000 5000
Wire Wire Line
	8450 4400 8450 4550
Connection ~ 8450 4600
Connection ~ 8450 4550
Wire Wire Line
	8450 4550 8450 4600
Wire Wire Line
	7850 4550 8450 4550
Wire Wire Line
	8750 5100 8750 5150
Connection ~ 8750 5100
Wire Wire Line
	8750 5100 9000 5100
$Comp
L Device:C C5
U 1 1 62AB5453
P 7300 3750
F 0 "C5" V 7250 3900 50  0000 C CNN
F 1 "100nF" V 7250 3600 43  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 7338 3600 50  0001 C CNN
F 3 "~" H 7300 3750 50  0001 C CNN
	1    7300 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	7450 3750 7550 3750
Connection ~ 7550 3750
Wire Wire Line
	7550 3750 7550 3850
Wire Wire Line
	7050 3800 7050 3750
Wire Wire Line
	7050 3750 7150 3750
Wire Wire Line
	6950 4450 7250 4450
Wire Wire Line
	7050 4350 7250 4350
Text GLabel 8250 1450 2    50   Input ~ 0
Vout+
Text GLabel 8250 2350 2    50   Input ~ 0
Vout-
Wire Wire Line
	8050 2250 8050 2350
Wire Wire Line
	8050 2350 8250 2350
Connection ~ 8050 2250
Wire Wire Line
	8250 1450 8050 1450
Wire Wire Line
	8050 1450 8050 1550
Connection ~ 8050 1550
$Comp
L Device:CP C4
U 1 1 62AEB5BC
P 7300 3500
F 0 "C4" V 7045 3500 50  0000 C CNN
F 1 "10uF" V 7136 3500 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D12.5mm_P5.00mm" H 7338 3350 50  0001 C CNN
F 3 "~" H 7300 3500 50  0001 C CNN
	1    7300 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	7450 3500 7550 3500
Connection ~ 7550 3500
Wire Wire Line
	7550 3500 7550 3750
$Comp
L Device:R R1
U 1 1 62AFA2CB
P 8750 1700
F 0 "R1" H 8820 1746 50  0000 L CNN
F 1 "100k" H 8820 1655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0411_L9.9mm_D3.6mm_P12.70mm_Horizontal" V 8680 1700 50  0001 C CNN
F 3 "~" H 8750 1700 50  0001 C CNN
	1    8750 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 62AFB0A1
P 8750 2100
F 0 "R2" H 8820 2146 50  0000 L CNN
F 1 "10k" H 8820 2055 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0411_L9.9mm_D3.6mm_P12.70mm_Horizontal" V 8680 2100 50  0001 C CNN
F 3 "~" H 8750 2100 50  0001 C CNN
	1    8750 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 1850 8750 1900
Text GLabel 8900 1500 2    50   Input ~ 0
Vout+
Wire Wire Line
	8750 1550 8750 1500
Wire Wire Line
	8750 1500 8900 1500
Text GLabel 10100 1900 2    50   Input ~ 0
OUTPUT_TO_ADC
Wire Wire Line
	8750 1900 9500 1900
Connection ~ 8750 1900
Wire Wire Line
	8750 1900 8750 1950
Text GLabel 8900 2300 2    50   Input ~ 0
Vout-
Wire Wire Line
	8900 2300 8750 2300
Wire Wire Line
	8750 2300 8750 2250
$Comp
L Device:R_POT RV1
U 1 1 62B119A8
P 2450 4700
F 0 "RV1" H 2381 4746 50  0000 R CNN
F 1 "R_POT" H 2381 4655 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Alps_RK163_Single_Horizontal" H 2450 4700 50  0001 C CNN
F 3 "~" H 2450 4700 50  0001 C CNN
	1    2450 4700
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0112
U 1 1 62B133D2
P 2450 4300
F 0 "#PWR0112" H 2450 4150 50  0001 C CNN
F 1 "+3.3V" H 2465 4473 50  0000 C CNN
F 2 "" H 2450 4300 50  0001 C CNN
F 3 "" H 2450 4300 50  0001 C CNN
	1    2450 4300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 62B1FAC7
P 2450 4950
F 0 "#PWR0113" H 2450 4700 50  0001 C CNN
F 1 "GND" H 2455 4777 50  0000 C CNN
F 2 "" H 2450 4950 50  0001 C CNN
F 3 "" H 2450 4950 50  0001 C CNN
	1    2450 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 4850 2450 4950
Text GLabel 2850 4700 2    50   Input ~ 0
P_ADC
Wire Wire Line
	2600 4700 2850 4700
$Comp
L Device:R_POT RV2
U 1 1 62B3116B
P 3550 4700
F 0 "RV2" H 3481 4746 50  0000 R CNN
F 1 "R_POT" H 3481 4655 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Alps_RK163_Single_Horizontal" H 3550 4700 50  0001 C CNN
F 3 "~" H 3550 4700 50  0001 C CNN
	1    3550 4700
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0114
U 1 1 62B31171
P 3550 4050
F 0 "#PWR0114" H 3550 3900 50  0001 C CNN
F 1 "+3.3V" H 3565 4223 50  0000 C CNN
F 2 "" H 3550 4050 50  0001 C CNN
F 3 "" H 3550 4050 50  0001 C CNN
	1    3550 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 62B31177
P 3550 4300
F 0 "R10" H 3620 4346 50  0000 L CNN
F 1 "220" H 3620 4255 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0411_L9.9mm_D3.6mm_P12.70mm_Horizontal" V 3480 4300 50  0001 C CNN
F 3 "~" H 3550 4300 50  0001 C CNN
	1    3550 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 4450 3550 4550
Wire Wire Line
	3550 4050 3550 4150
$Comp
L power:GND #PWR0115
U 1 1 62B3117F
P 3550 4950
F 0 "#PWR0115" H 3550 4700 50  0001 C CNN
F 1 "GND" H 3555 4777 50  0000 C CNN
F 2 "" H 3550 4950 50  0001 C CNN
F 3 "" H 3550 4950 50  0001 C CNN
	1    3550 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 4850 3550 4950
Wire Wire Line
	3700 4700 3950 4700
Text GLabel 6950 4450 0    50   Input ~ 0
LIN
Connection ~ 7050 3750
$Comp
L Device:R R6
U 1 1 62A5919C
P 6350 3650
F 0 "R6" H 6420 3696 50  0000 L CNN
F 1 "47K" H 6420 3605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0411_L9.9mm_D3.6mm_P12.70mm_Horizontal" V 6280 3650 50  0001 C CNN
F 3 "~" H 6350 3650 50  0001 C CNN
	1    6350 3650
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0118
U 1 1 62A58BD1
P 6250 3100
F 0 "#PWR0118" H 6250 2950 50  0001 C CNN
F 1 "+12V" H 6265 3273 50  0000 C CNN
F 2 "" H 6250 3100 50  0001 C CNN
F 3 "" H 6250 3100 50  0001 C CNN
	1    6250 3100
	1    0    0    -1  
$EndComp
Text GLabel 6450 3400 2    50   Input ~ 0
LIN
$Comp
L power:+3.3V #PWR0119
U 1 1 62A50416
P 5450 2600
F 0 "#PWR0119" H 5450 2450 50  0001 C CNN
F 1 "+3.3V" H 5465 2773 50  0000 C CNN
F 2 "" H 5450 2600 50  0001 C CNN
F 3 "" H 5450 2600 50  0001 C CNN
	1    5450 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 62A48961
P 5450 3650
F 0 "R7" H 5520 3696 50  0000 L CNN
F 1 "220" H 5520 3605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0411_L9.9mm_D3.6mm_P12.70mm_Horizontal" V 5380 3650 50  0001 C CNN
F 3 "~" H 5450 3650 50  0001 C CNN
	1    5450 3650
	-1   0    0    1   
$EndComp
$Comp
L Isolator:PC817 U1
U 1 1 62A41DD4
P 5900 3300
F 0 "U1" H 5900 3625 50  0000 C CNN
F 1 "PC817" H 5900 3534 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W10.16mm" H 5700 3100 50  0001 L CIN
F 3 "http://www.soselectronic.cz/a_info/resource/d/pc817.pdf" H 5900 3300 50  0001 L CNN
	1    5900 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 3500 7050 3750
Wire Wire Line
	7150 3500 7050 3500
Text GLabel 5050 2900 0    50   Input ~ 0
LOW_PWM_IN
$Comp
L Device:R R3
U 1 1 62A933AA
P 4050 2500
F 0 "R3" H 4120 2546 50  0000 L CNN
F 1 "10" H 4120 2455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0411_L9.9mm_D3.6mm_P12.70mm_Horizontal" V 3980 2500 50  0001 C CNN
F 3 "~" H 4050 2500 50  0001 C CNN
	1    4050 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 2650 4050 2850
Wire Wire Line
	4050 2350 4050 2300
Wire Wire Line
	4050 2300 3450 2300
Connection ~ 3450 2300
Wire Wire Line
	3450 2300 3450 2500
Wire Wire Line
	4050 3150 4050 3450
Wire Wire Line
	4050 3450 3450 3450
Connection ~ 3450 3450
$Comp
L power:GND1 #PWR0102
U 1 1 62A45600
P 7250 5100
F 0 "#PWR0102" H 7250 4850 50  0001 C CNN
F 1 "GND1" H 7255 4927 50  0000 C CNN
F 2 "" H 7250 5100 50  0001 C CNN
F 3 "" H 7250 5100 50  0001 C CNN
	1    7250 5100
	1    0    0    -1  
$EndComp
$Comp
L power:GND1 #PWR0104
U 1 1 62A4606F
P 7050 4350
F 0 "#PWR0104" H 7050 4100 50  0001 C CNN
F 1 "GND1" V 7055 4222 50  0000 R CNN
F 2 "" H 7050 4350 50  0001 C CNN
F 3 "" H 7050 4350 50  0001 C CNN
	1    7050 4350
	0    1    1    0   
$EndComp
$Comp
L power:GND1 #PWR0105
U 1 1 62A465B2
P 7950 4450
F 0 "#PWR0105" H 7950 4200 50  0001 C CNN
F 1 "GND1" V 7955 4322 50  0000 R CNN
F 2 "" H 7950 4450 50  0001 C CNN
F 3 "" H 7950 4450 50  0001 C CNN
	1    7950 4450
	0    -1   -1   0   
$EndComp
$Comp
L power:GND1 #PWR0107
U 1 1 62A46DC8
P 8750 5150
F 0 "#PWR0107" H 8750 4900 50  0001 C CNN
F 1 "GND1" H 8755 4977 50  0000 C CNN
F 2 "" H 8750 5150 50  0001 C CNN
F 3 "" H 8750 5150 50  0001 C CNN
	1    8750 5150
	1    0    0    -1  
$EndComp
$Comp
L power:GND1 #PWR0109
U 1 1 62A47840
P 7050 3800
F 0 "#PWR0109" H 7050 3550 50  0001 C CNN
F 1 "GND1" H 7055 3627 50  0000 C CNN
F 2 "" H 7050 3800 50  0001 C CNN
F 3 "" H 7050 3800 50  0001 C CNN
	1    7050 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND1 #PWR0110
U 1 1 62A47F4F
P 4050 3550
F 0 "#PWR0110" H 4050 3300 50  0001 C CNN
F 1 "GND1" H 4055 3377 50  0000 C CNN
F 2 "" H 4050 3550 50  0001 C CNN
F 3 "" H 4050 3550 50  0001 C CNN
	1    4050 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3550 4050 3450
Connection ~ 4050 3450
$Comp
L power:GND #PWR0111
U 1 1 62A4C8BA
P 8750 2550
F 0 "#PWR0111" H 8750 2300 50  0001 C CNN
F 1 "GND" H 8755 2377 50  0000 C CNN
F 2 "" H 8750 2550 50  0001 C CNN
F 3 "" H 8750 2550 50  0001 C CNN
	1    8750 2550
	1    0    0    -1  
$EndComp
Connection ~ 8750 2300
$Comp
L Connector:Conn_01x08_Male J5
U 1 1 62A523FF
P 9200 3500
F 0 "J5" H 9308 3981 50  0000 C CNN
F 1 "Conn_01x08_Male" V 9100 3500 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 9200 3500 50  0001 C CNN
F 3 "~" H 9200 3500 50  0001 C CNN
	1    9200 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 62A5AF9C
P 9500 4000
F 0 "#PWR0121" H 9500 3750 50  0001 C CNN
F 1 "GND" H 9505 3827 50  0000 C CNN
F 2 "" H 9500 4000 50  0001 C CNN
F 3 "" H 9500 4000 50  0001 C CNN
	1    9500 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 3900 9500 3900
Wire Wire Line
	9500 3900 9500 4000
Text GLabel 9500 3300 2    50   Input ~ 0
OUTPUT_TO_ADC
Wire Wire Line
	9400 3300 9500 3300
Text GLabel 9500 3700 2    50   Input ~ 0
P_ADC
Text GLabel 9500 3800 2    50   Input ~ 0
I_ADC
Text GLabel 9500 3500 2    50   Input ~ 0
LOW_PWM_IN
Wire Wire Line
	9400 3500 9500 3500
Wire Wire Line
	9400 3700 9500 3700
Wire Wire Line
	9400 3800 9500 3800
$Comp
L Device:D_Zener D5
U 1 1 62A603B0
P 9850 2200
F 0 "D5" V 9804 2280 50  0000 L CNN
F 1 "D_Zener" V 9895 2280 50  0000 L CNN
F 2 "Diode_THT:D_5W_P12.70mm_Horizontal" H 9850 2200 50  0001 C CNN
F 3 "~" H 9850 2200 50  0001 C CNN
	1    9850 2200
	0    1    1    0   
$EndComp
Wire Wire Line
	9850 2050 9850 1900
Connection ~ 9850 1900
Wire Wire Line
	9850 1900 10100 1900
Wire Wire Line
	9850 2450 9850 2350
Wire Wire Line
	8750 2300 8750 2450
Wire Wire Line
	8750 2450 9500 2450
Connection ~ 8750 2450
Wire Wire Line
	8750 2450 8750 2550
Wire Wire Line
	9500 2050 9500 1900
Connection ~ 9500 1900
Wire Wire Line
	9500 1900 9850 1900
Wire Wire Line
	9500 2350 9500 2450
Connection ~ 9500 2450
Wire Wire Line
	9500 2450 9850 2450
$Comp
L Connector:Screw_Terminal_01x02 J4
U 1 1 62A952BC
P 5400 4750
F 0 "J4" H 5318 4967 50  0000 C CNN
F 1 "Vcc_IR2110" H 5318 4876 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 5400 4750 50  0001 C CNN
F 3 "~" H 5400 4750 50  0001 C CNN
	1    5400 4750
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5750 4750 5600 4750
Wire Wire Line
	5600 4850 5750 4850
Wire Wire Line
	5750 4850 5750 4950
$Comp
L power:+12V #PWR0108
U 1 1 62A97516
P 5750 4600
F 0 "#PWR0108" H 5750 4450 50  0001 C CNN
F 1 "+12V" H 5765 4773 50  0000 C CNN
F 2 "" H 5750 4600 50  0001 C CNN
F 3 "" H 5750 4600 50  0001 C CNN
	1    5750 4600
	1    0    0    -1  
$EndComp
Text GLabel 3950 4700 2    50   Input ~ 0
I_ADC
$Comp
L power:GND1 #PWR0122
U 1 1 62A45B3A
P 5750 4950
F 0 "#PWR0122" H 5750 4700 50  0001 C CNN
F 1 "GND1" H 5755 4777 50  0000 C CNN
F 2 "" H 5750 4950 50  0001 C CNN
F 3 "" H 5750 4950 50  0001 C CNN
	1    5750 4950
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM7805_TO220 U3
U 1 1 62AB02CD
P 2800 6150
F 0 "U3" H 2800 6392 50  0000 C CNN
F 1 "LM7805_TO220" H 2800 6301 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 2800 6375 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 2800 6100 50  0001 C CNN
	1    2800 6150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0124
U 1 1 62AB6ACF
P 3650 6050
F 0 "#PWR0124" H 3650 5900 50  0001 C CNN
F 1 "+5V" H 3665 6223 50  0000 C CNN
F 2 "" H 3650 6050 50  0001 C CNN
F 3 "" H 3650 6050 50  0001 C CNN
	1    3650 6050
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C9
U 1 1 62AEAEE4
P 1700 6400
F 0 "C9" V 1445 6400 50  0000 C CNN
F 1 "10uF" V 1536 6400 50  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D12.5mm_P5.00mm" H 1738 6250 50  0001 C CNN
F 3 "~" H 1700 6400 50  0001 C CNN
	1    1700 6400
	1    0    0    -1  
$EndComp
$Comp
L power:GND1 #PWR0125
U 1 1 62AF267A
P 1700 6850
F 0 "#PWR0125" H 1700 6600 50  0001 C CNN
F 1 "GND1" H 1705 6677 50  0000 C CNN
F 2 "" H 1700 6850 50  0001 C CNN
F 3 "" H 1700 6850 50  0001 C CNN
	1    1700 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 6550 1700 6850
Wire Wire Line
	3100 6150 3400 6150
Wire Wire Line
	3650 6150 3650 6050
Wire Wire Line
	3400 6250 3400 6150
Connection ~ 3400 6150
Wire Wire Line
	3400 6150 3650 6150
Wire Wire Line
	3650 6250 3650 6150
Connection ~ 3650 6150
$Comp
L power:GND1 #PWR0126
U 1 1 62B1A7B2
P 3400 6850
F 0 "#PWR0126" H 3400 6600 50  0001 C CNN
F 1 "GND1" H 3405 6677 50  0000 C CNN
F 2 "" H 3400 6850 50  0001 C CNN
F 3 "" H 3400 6850 50  0001 C CNN
	1    3400 6850
	1    0    0    -1  
$EndComp
$Comp
L power:GND1 #PWR0127
U 1 1 62B1AC71
P 3650 6850
F 0 "#PWR0127" H 3650 6600 50  0001 C CNN
F 1 "GND1" H 3655 6677 50  0000 C CNN
F 2 "" H 3650 6850 50  0001 C CNN
F 3 "" H 3650 6850 50  0001 C CNN
	1    3650 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 6550 3400 6750
Wire Wire Line
	3650 6550 3650 6850
Wire Wire Line
	2800 6450 2800 6750
Wire Wire Line
	2800 6750 3400 6750
Connection ~ 3400 6750
Wire Wire Line
	3400 6750 3400 6850
Wire Wire Line
	1700 6250 1700 6150
Wire Wire Line
	1700 6150 2150 6150
Wire Wire Line
	2150 6250 2150 6150
Connection ~ 2150 6150
Wire Wire Line
	2150 6150 2500 6150
$Comp
L power:GND1 #PWR0128
U 1 1 62B4CBE0
P 2150 6850
F 0 "#PWR0128" H 2150 6600 50  0001 C CNN
F 1 "GND1" H 2155 6677 50  0000 C CNN
F 2 "" H 2150 6850 50  0001 C CNN
F 3 "" H 2150 6850 50  0001 C CNN
	1    2150 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 6550 2150 6850
$Comp
L power:+12V #PWR0129
U 1 1 62B543D5
P 1700 6100
F 0 "#PWR0129" H 1700 5950 50  0001 C CNN
F 1 "+12V" H 1715 6273 50  0000 C CNN
F 2 "" H 1700 6100 50  0001 C CNN
F 3 "" H 1700 6100 50  0001 C CNN
	1    1700 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 6100 1700 6150
Connection ~ 1700 6150
$Comp
L pspice:INDUCTOR L2
U 1 1 62B8F05D
P 6400 1550
F 0 "L2" H 6400 1765 50  0000 C CNN
F 1 "INDUCTOR" H 6400 1674 50  0000 C CNN
F 2 "Inductor_THT:L_Axial_L26.0mm_D11.0mm_P5.08mm_Vertical_Fastron_77A" H 6400 1550 50  0001 C CNN
F 3 "~" H 6400 1550 50  0001 C CNN
	1    6400 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 1550 6150 1550
Wire Wire Line
	6650 1550 6850 1550
Connection ~ 6850 1550
$Comp
L Device:C C8
U 1 1 62A6C807
P 9000 4850
F 0 "C8" V 8950 5000 50  0000 C CNN
F 1 "100nF" V 8950 4700 43  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 9038 4700 50  0001 C CNN
F 3 "~" H 9000 4850 50  0001 C CNN
	1    9000 4850
	-1   0    0    1   
$EndComp
$Comp
L Device:C C7
U 1 1 62A6CE9F
P 9500 2200
F 0 "C7" V 9450 2350 50  0000 C CNN
F 1 "100nF" V 9450 2050 43  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 9538 2050 50  0001 C CNN
F 3 "~" H 9500 2200 50  0001 C CNN
	1    9500 2200
	-1   0    0    1   
$EndComp
$Comp
L Device:C C11
U 1 1 62A6E8F1
P 3400 6400
F 0 "C11" V 3350 6550 50  0000 C CNN
F 1 "100nF" V 3350 6250 43  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 3438 6250 50  0001 C CNN
F 3 "~" H 3400 6400 50  0001 C CNN
	1    3400 6400
	-1   0    0    1   
$EndComp
$Comp
L Device:C C12
U 1 1 62A6F03D
P 3650 6400
F 0 "C12" V 3600 6550 50  0000 C CNN
F 1 "100nF" V 3600 6250 43  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 3688 6250 50  0001 C CNN
F 3 "~" H 3650 6400 50  0001 C CNN
	1    3650 6400
	-1   0    0    1   
$EndComp
$Comp
L Device:C C10
U 1 1 62A6F424
P 2150 6400
F 0 "C10" V 2100 6550 50  0000 C CNN
F 1 "100nF" V 2100 6250 43  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 2188 6250 50  0001 C CNN
F 3 "~" H 2150 6400 50  0001 C CNN
	1    2150 6400
	-1   0    0    1   
$EndComp
$Comp
L Device:C C3
U 1 1 62A8A21F
P 4050 3000
F 0 "C3" V 4000 3150 50  0000 C CNN
F 1 "1nF" V 4000 2850 43  0000 C CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 4088 2850 50  0001 C CNN
F 3 "~" H 4050 3000 50  0001 C CNN
	1    4050 3000
	-1   0    0    1   
$EndComp
$Comp
L Transistor_BJT:BC338 Q2
U 1 1 62ABD36D
P 5350 2900
F 0 "Q2" H 5541 2946 50  0000 L CNN
F 1 "BC317" H 5541 2855 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 5550 2825 50  0001 L CIN
F 3 "http://diotec.com/tl_files/diotec/files/pdf/datasheets/bc337" H 5350 2900 50  0001 L CNN
	1    5350 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 2900 5150 2900
Wire Wire Line
	5450 2600 5450 2700
Wire Wire Line
	5450 3100 5450 3200
Wire Wire Line
	5450 3200 5600 3200
Wire Wire Line
	5450 3500 5450 3400
Wire Wire Line
	5450 3400 5600 3400
$Comp
L power:GND #PWR01
U 1 1 62ADEA04
P 5450 3900
F 0 "#PWR01" H 5450 3650 50  0001 C CNN
F 1 "GND" H 5455 3727 50  0000 C CNN
F 2 "" H 5450 3900 50  0001 C CNN
F 3 "" H 5450 3900 50  0001 C CNN
	1    5450 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 3800 5450 3900
Wire Wire Line
	6200 3400 6350 3400
Wire Wire Line
	6350 3400 6350 3500
$Comp
L power:GND1 #PWR02
U 1 1 62AF42FD
P 6350 3900
F 0 "#PWR02" H 6350 3650 50  0001 C CNN
F 1 "GND1" H 6355 3727 50  0000 C CNN
F 2 "" H 6350 3900 50  0001 C CNN
F 3 "" H 6350 3900 50  0001 C CNN
	1    6350 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 3800 6350 3900
Wire Wire Line
	6350 3400 6450 3400
Connection ~ 6350 3400
Wire Wire Line
	6250 3100 6250 3200
Wire Wire Line
	6250 3200 6200 3200
$Comp
L power:+3.3V #PWR0120
U 1 1 62A54DA9
P 9500 3600
F 0 "#PWR0120" H 9500 3450 50  0001 C CNN
F 1 "+3.3V" V 9500 3850 50  0000 C CNN
F 2 "" H 9500 3600 50  0001 C CNN
F 3 "" H 9500 3600 50  0001 C CNN
	1    9500 3600
	0    1    1    0   
$EndComp
NoConn ~ 9400 3200
Wire Wire Line
	9400 3600 9500 3600
NoConn ~ 9400 3400
Wire Wire Line
	5750 4600 5750 4750
Wire Wire Line
	2450 4300 2450 4550
$EndSCHEMATC
