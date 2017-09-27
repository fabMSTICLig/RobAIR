EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:cartehaute-cache
EELAYER 25 0
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
$Comp
L CONN_01X02 P3
U 1 1 581A1274
P 3500 3750
F 0 "P3" H 3500 3900 50  0000 C CNN
F 1 "LED" V 3600 3750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 3500 3750 50  0001 C CNN
F 3 "" H 3500 3750 50  0000 C CNN
	1    3500 3750
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 P1
U 1 1 581A12AF
P 3500 2700
F 0 "P1" H 3500 2900 50  0000 C CNN
F 1 "Servo" V 3600 2700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3500 2700 50  0001 C CNN
F 3 "" H 3500 2700 50  0000 C CNN
	1    3500 2700
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 P2
U 1 1 581A12FA
P 3500 3250
F 0 "P2" H 3500 3450 50  0000 C CNN
F 1 "Yeux" V 3600 3250 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3500 3250 50  0001 C CNN
F 3 "" H 3500 3250 50  0000 C CNN
	1    3500 3250
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P4
U 1 1 581A1378
P 3500 4150
F 0 "P4" H 3500 4300 50  0000 C CNN
F 1 "HUB" V 3600 4150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 3500 4150 50  0001 C CNN
F 3 "" H 3500 4150 50  0000 C CNN
	1    3500 4150
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P6
U 1 1 581A13A0
P 5100 3750
F 0 "P6" H 5100 3900 50  0000 C CNN
F 1 "5V" V 5200 3750 50  0000 C CNN
F 2 "robair:Bornier5mm" H 5100 3750 50  0001 C CNN
F 3 "" H 5100 3750 50  0000 C CNN
	1    5100 3750
	1    0    0    1   
$EndComp
$Comp
L CONN_01X02 P11
U 1 1 581AF215
P 3500 2000
F 0 "P11" H 3500 2150 50  0000 C CNN
F 1 "AN2" V 3600 2000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 3500 2000 50  0001 C CNN
F 3 "" H 3500 2000 50  0000 C CNN
	1    3500 2000
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P9
U 1 1 581AF69C
P 3500 1700
F 0 "P9" H 3500 1850 50  0000 C CNN
F 1 "AN1" V 3600 1700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 3500 1700 50  0001 C CNN
F 3 "" H 3500 1700 50  0000 C CNN
	1    3500 1700
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P8
U 1 1 581AF6D0
P 3500 1400
F 0 "P8" H 3500 1550 50  0000 C CNN
F 1 "ARU" V 3600 1400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 3500 1400 50  0001 C CNN
F 3 "" H 3500 1400 50  0000 C CNN
	1    3500 1400
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X04 P10
U 1 1 581AF810
P 4600 1400
F 0 "P10" H 4600 1650 50  0000 C CNN
F 1 "Analog" V 4700 1400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 4600 1400 50  0001 C CNN
F 3 "" H 4600 1400 50  0000 C CNN
	1    4600 1400
	1    0    0    -1  
$EndComp
Text Label 4300 3700 0    60   ~ 0
5V
Text Label 4300 3800 0    60   ~ 0
GND
Wire Wire Line
	3900 2450 3900 4200
Wire Wire Line
	3900 4200 3700 4200
Connection ~ 3900 3800
Wire Wire Line
	3800 4100 3700 4100
Wire Wire Line
	3700 3700 4900 3700
Wire Wire Line
	3900 3350 3700 3350
Wire Wire Line
	3900 2800 3700 2800
Connection ~ 3900 3350
Wire Wire Line
	3700 3800 4900 3800
Connection ~ 3800 3700
Wire Wire Line
	3800 3250 3700 3250
Wire Wire Line
	3800 2700 3700 2700
Connection ~ 3800 3250
Wire Wire Line
	3700 3150 4950 3150
Wire Wire Line
	4900 3050 4900 2600
Wire Wire Line
	4900 2600 3700 2600
Wire Wire Line
	3800 2050 3700 2050
Connection ~ 3800 2700
Wire Wire Line
	3800 1750 3700 1750
Connection ~ 3800 2050
Connection ~ 3800 1750
Wire Wire Line
	3800 1450 3700 1450
Connection ~ 3800 1450
Wire Wire Line
	3700 1350 4400 1350
Wire Wire Line
	4400 1450 3950 1450
Wire Wire Line
	3950 1450 3950 1650
Wire Wire Line
	3950 1650 3700 1650
Wire Wire Line
	4400 1550 4400 1950
Wire Wire Line
	4400 1950 3700 1950
Wire Wire Line
	4400 1250 3800 1250
Wire Wire Line
	3800 1250 3800 2050
Wire Wire Line
	3800 2350 3800 4100
$Comp
L CONN_01X03 P5
U 1 1 58CB0AEB
P 3500 2350
F 0 "P5" H 3500 2550 50  0000 C CNN
F 1 "Servo2" V 3600 2350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 3500 2350 50  0001 C CNN
F 3 "" H 3500 2350 50  0000 C CNN
	1    3500 2350
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 P7
U 1 1 58CB0F48
P 5150 3050
F 0 "P7" H 5150 3250 50  0000 C CNN
F 1 "data" V 5250 3050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 5150 3050 50  0001 C CNN
F 3 "" H 5150 3050 50  0000 C CNN
	1    5150 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 2350 3800 2350
Wire Wire Line
	3700 2450 3900 2450
Connection ~ 3900 2800
Wire Wire Line
	4950 3050 4900 3050
Wire Wire Line
	4950 2950 4950 2250
Wire Wire Line
	4950 2250 3700 2250
$EndSCHEMATC
