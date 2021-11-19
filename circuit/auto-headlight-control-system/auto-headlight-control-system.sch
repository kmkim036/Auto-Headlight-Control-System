EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 2500 2650 0    50   Input ~ 0
SCL
Text GLabel 2500 2750 0    50   Input ~ 0
SDA
Text GLabel 2500 3150 0    50   Input ~ 0
GND_1
Text GLabel 2500 3250 0    50   Input ~ 0
3V3_1
Text GLabel 2500 2850 0    50   Input ~ 0
CAN_RX_1
Text GLabel 2500 2950 0    50   Input ~ 0
CAN_TX_1
Text GLabel 2550 3950 2    50   Input ~ 0
CAN_RX_1
Text GLabel 2550 4050 2    50   Input ~ 0
CAN_TX_1
Text GLabel 2550 3850 2    50   Input ~ 0
GND_1
Text GLabel 3900 3700 2    50   Input ~ 0
SCL
Text GLabel 3900 4750 2    50   Input ~ 0
SCL
Text GLabel 3900 4850 2    50   Input ~ 0
SDA
Text GLabel 3900 3800 2    50   Input ~ 0
SDA
Text GLabel 3900 4200 2    50   Input ~ 0
3V3_1
Text GLabel 3900 4100 2    50   Input ~ 0
GND_1
Text GLabel 3900 4550 2    50   Input ~ 0
GND_1
Text GLabel 3900 4650 2    50   Input ~ 0
3V3_1
Wire Wire Line
	2500 2650 2700 2650
Wire Wire Line
	2500 2750 2700 2750
Wire Wire Line
	2700 2850 2500 2850
Wire Wire Line
	2500 2950 2700 2950
Wire Wire Line
	2500 3150 2700 3150
Wire Wire Line
	2500 3250 2700 3250
Wire Wire Line
	2350 3750 2550 3750
Wire Wire Line
	2350 3850 2550 3850
Wire Wire Line
	2350 3950 2550 3950
Wire Wire Line
	2350 4050 2550 4050
Wire Wire Line
	3750 3700 3900 3700
Wire Wire Line
	3750 3800 3900 3800
Wire Wire Line
	3750 4100 3900 4100
Wire Wire Line
	3750 4200 3900 4200
Wire Wire Line
	3750 4550 3900 4550
Wire Wire Line
	3750 4650 3900 4650
Wire Wire Line
	3750 4750 3900 4750
Wire Wire Line
	3750 4850 3900 4850
Text GLabel 7100 2100 2    50   Input ~ 0
PWM
Text GLabel 6200 3000 0    50   Input ~ 0
GND_2
Text GLabel 6200 2900 0    50   Input ~ 0
5V
Wire Wire Line
	7000 2100 7100 2100
Wire Wire Line
	6200 2900 6300 2900
Wire Wire Line
	6200 3000 6300 3000
Text GLabel 6200 2700 0    50   Input ~ 0
CAN_RX_2
Text GLabel 6200 2800 0    50   Input ~ 0
CAN_TX_2
Wire Wire Line
	6200 2700 6300 2700
Wire Wire Line
	6200 2800 6300 2800
Text GLabel 6050 3750 2    50   Input ~ 0
CAN_RX_2
Text GLabel 6050 3850 2    50   Input ~ 0
CAN_TX_2
Text GLabel 6050 3650 2    50   Input ~ 0
GND_2
Text GLabel 7400 3800 2    50   Input ~ 0
GND_2
Text GLabel 7400 3700 2    50   Input ~ 0
5V
Text GLabel 7400 3600 2    50   Input ~ 0
PWM
Wire Wire Line
	5900 3550 6050 3550
Wire Wire Line
	5900 3650 6050 3650
Wire Wire Line
	5900 3750 6050 3750
Wire Wire Line
	5900 3850 6050 3850
Wire Wire Line
	7250 3600 7400 3600
Wire Wire Line
	7250 3700 7400 3700
Wire Wire Line
	7400 3800 7250 3800
$Comp
L Device:R R1
U 1 1 619587F7
P 5400 2500
F 0 "R1" H 5470 2546 50  0000 L CNN
F 1 "100ohm" H 5470 2455 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 5330 2500 50  0001 C CNN
F 3 "~" H 5400 2500 50  0001 C CNN
	1    5400 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 2200 6300 2200
Text GLabel 5400 2850 0    50   Input ~ 0
GND_2
Wire Wire Line
	5400 2350 5400 2200
Wire Wire Line
	5400 2650 5400 2850
Text GLabel 2550 3750 2    50   Input ~ 0
3V3_1
Text GLabel 6200 3100 0    50   Input ~ 0
3V3_2
Wire Wire Line
	6200 3100 6300 3100
Text GLabel 6050 3550 2    50   Input ~ 0
3V3_2
Text GLabel 6150 1700 0    50   Input ~ 0
Serial_Tx_2
Wire Wire Line
	6150 1700 6300 1700
Text GLabel 2500 1850 0    50   Input ~ 0
Serial_Tx_1
Wire Wire Line
	2500 1850 2700 1850
$Comp
L Connector:Conn_01x01_Female J2
U 1 1 6198EC01
P 1650 1650
F 0 "J2" H 1678 1676 50  0000 L CNN
F 1 "Serial_GND_1" H 1678 1585 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Vertical" H 1650 1650 50  0001 C CNN
F 3 "~" H 1650 1650 50  0001 C CNN
	1    1650 1650
	-1   0    0    1   
$EndComp
Text GLabel 2250 1650 2    50   Input ~ 0
GND_1
$Comp
L Connector:Conn_01x01_Female J4
U 1 1 6199A079
P 5350 1500
F 0 "J4" H 5378 1526 50  0000 L CNN
F 1 "Serial_GND_2" H 5378 1435 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Vertical" H 5350 1500 50  0001 C CNN
F 3 "~" H 5350 1500 50  0001 C CNN
	1    5350 1500
	-1   0    0    1   
$EndComp
Text GLabel 5850 1500 2    50   Input ~ 0
GND_2
Wire Wire Line
	5550 1500 5850 1500
$Comp
L Connector:Conn_01x01_Female J3
U 1 1 619A20B6
P 5350 1250
F 0 "J3" H 5378 1276 50  0000 L CNN
F 1 "Serial_Tx_2" H 5378 1185 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Vertical" H 5350 1250 50  0001 C CNN
F 3 "~" H 5350 1250 50  0001 C CNN
	1    5350 1250
	-1   0    0    1   
$EndComp
Text GLabel 5700 1250 2    50   Input ~ 0
Serial_Tx_2
Wire Wire Line
	5550 1250 5700 1250
$Comp
L Connector:Conn_01x01_Female J1
U 1 1 619A7053
P 1650 1400
F 0 "J1" H 1678 1426 50  0000 L CNN
F 1 "Serial_Tx_1" H 1678 1335 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x01_P2.54mm_Vertical" H 1650 1400 50  0001 C CNN
F 3 "~" H 1650 1400 50  0001 C CNN
	1    1650 1400
	-1   0    0    1   
$EndComp
Text GLabel 2050 1400 2    50   Input ~ 0
Serial_Tx_1
Wire Wire Line
	1850 1650 2250 1650
Wire Wire Line
	2050 1400 1850 1400
$Comp
L Connector:Conn_01x01_Female D1
U 1 1 619C9579
P 5950 2200
F 0 "D1" H 5842 1975 50  0000 C CNN
F 1 "LED_anode" H 5842 2066 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 5950 2200 50  0001 C CNN
F 3 "~" H 5950 2200 50  0001 C CNN
	1    5950 2200
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x01_Female D2
U 1 1 619CC2C9
P 5600 2200
F 0 "D2" H 5492 1975 50  0000 C CNN
F 1 "LED_cathode" H 5492 2066 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 5600 2200 50  0001 C CNN
F 3 "~" H 5600 2200 50  0001 C CNN
	1    5600 2200
	1    0    0    1   
$EndComp
$Comp
L auto-headlight-control-system:ADXL345 S1
U 1 1 619D057B
P 3700 3600
F 0 "S1" H 3612 3725 50  0000 C CNN
F 1 "ADXL345" H 3612 3634 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 3700 3600 50  0001 C CNN
F 3 "" H 3700 3600 50  0001 C CNN
	1    3700 3600
	1    0    0    -1  
$EndComp
$Comp
L auto-headlight-control-system:STM32_L U1
U 1 1 619D226F
P 2750 1250
F 0 "U1" H 2750 1400 50  0000 L CNN
F 1 "STM32_L" H 2650 1300 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical" H 2750 1250 50  0001 C CNN
F 3 "" H 2750 1250 50  0001 C CNN
	1    2750 1250
	1    0    0    -1  
$EndComp
$Comp
L auto-headlight-control-system:STM32_R U2
U 1 1 619D37A5
P 3350 1250
F 0 "U2" H 3262 1375 50  0000 C CNN
F 1 "STM32_R" H 3262 1284 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical" H 3350 1250 50  0001 C CNN
F 3 "" H 3350 1250 50  0001 C CNN
	1    3350 1250
	1    0    0    -1  
$EndComp
$Comp
L auto-headlight-control-system:STM32_L U3
U 1 1 619D5904
P 6350 1100
F 0 "U3" H 6350 1250 50  0000 L CNN
F 1 "STM32_L" H 6250 1150 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical" H 6350 1100 50  0001 C CNN
F 3 "" H 6350 1100 50  0001 C CNN
	1    6350 1100
	1    0    0    -1  
$EndComp
$Comp
L auto-headlight-control-system:STM32_R U4
U 1 1 619D7A8A
P 6950 1100
F 0 "U4" H 6862 1225 50  0000 C CNN
F 1 "STM32_R" H 6862 1134 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical" H 6950 1100 50  0001 C CNN
F 3 "" H 6950 1100 50  0001 C CNN
	1    6950 1100
	1    0    0    -1  
$EndComp
$Comp
L auto-headlight-control-system:AP3216 S2
U 1 1 619DE2BC
P 3700 4450
F 0 "S2" H 3612 4575 50  0000 C CNN
F 1 "AP3216" H 3612 4484 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3700 4450 50  0001 C CNN
F 3 "" H 3700 4450 50  0001 C CNN
	1    3700 4450
	1    0    0    -1  
$EndComp
$Comp
L auto-headlight-control-system:SN65HVD230 T1
U 1 1 619DEAF4
P 2300 3650
F 0 "T1" H 2137 3775 50  0000 C CNN
F 1 "SN65HVD230" H 2137 3684 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 2300 3650 50  0001 C CNN
F 3 "" H 2300 3650 50  0001 C CNN
	1    2300 3650
	1    0    0    -1  
$EndComp
$Comp
L auto-headlight-control-system:SN65HVD230 T2
U 1 1 619DF1FF
P 5850 3450
F 0 "T2" H 5687 3575 50  0000 C CNN
F 1 "SN65HVD230" H 5687 3484 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 5850 3450 50  0001 C CNN
F 3 "" H 5850 3450 50  0001 C CNN
	1    5850 3450
	1    0    0    -1  
$EndComp
$Comp
L auto-headlight-control-system:SG90 M1
U 1 1 619E0DA1
P 7200 3500
F 0 "M1" H 7112 3625 50  0000 C CNN
F 1 "SG90" H 7112 3534 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 7200 3500 50  0001 C CNN
F 3 "" H 7200 3500 50  0001 C CNN
	1    7200 3500
	1    0    0    -1  
$EndComp
$EndSCHEMATC
