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
P 3500 3450
F 0 "P3" H 3500 3600 50  0000 C CNN
F 1 "LED" V 3600 3450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3500 3450 50  0001 C CNN
F 3 "" H 3500 3450 50  0000 C CNN
	1    3500 3450
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 P1
U 1 1 581A12AF
P 3500 2400
F 0 "P1" H 3500 2600 50  0000 C CNN
F 1 "Servo" V 3600 2400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 3500 2400 50  0001 C CNN
F 3 "" H 3500 2400 50  0000 C CNN
	1    3500 2400
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X03 P2
U 1 1 581A12FA
P 3500 2950
F 0 "P2" H 3500 3150 50  0000 C CNN
F 1 "Yeux" V 3600 2950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 3500 2950 50  0001 C CNN
F 3 "" H 3500 2950 50  0000 C CNN
	1    3500 2950
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P4
U 1 1 581A1378
P 3500 3850
F 0 "P4" H 3500 4000 50  0000 C CNN
F 1 "HUB" V 3600 3850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3500 3850 50  0001 C CNN
F 3 "" H 3500 3850 50  0000 C CNN
	1    3500 3850
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P6
U 1 1 581A13A0
P 5100 3450
F 0 "P6" H 5100 3600 50  0000 C CNN
F 1 "5V" V 5200 3450 50  0000 C CNN
F 2 "robair:Bornier5mm" H 5100 3450 50  0001 C CNN
F 3 "" H 5100 3450 50  0000 C CNN
	1    5100 3450
	1    0    0    1   
$EndComp
$Comp
L CONN_01X02 P11
U 1 1 581AF215
P 3500 2000
F 0 "P11" H 3500 2150 50  0000 C CNN
F 1 "ARU" V 3600 2000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3500 2000 50  0001 C CNN
F 3 "" H 3500 2000 50  0000 C CNN
	1    3500 2000
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P5
U 1 1 581A14B9
P 5100 2800
F 0 "P5" H 5100 2950 50  0000 C CNN
F 1 "data" V 5200 2800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 5100 2800 50  0001 C CNN
F 3 "" H 5100 2800 50  0000 C CNN
	1    5100 2800
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P9
U 1 1 581AF69C
P 3500 1700
F 0 "P9" H 3500 1850 50  0000 C CNN
F 1 "AN1" V 3600 1700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3500 1700 50  0001 C CNN
F 3 "" H 3500 1700 50  0000 C CNN
	1    3500 1700
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 P8
U 1 1 581AF6D0
P 3500 1400
F 0 "P8" H 3500 1550 50  0000 C CNN
F 1 "AN2" V 3600 1400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 3500 1400 50  0001 C CNN
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
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 4600 1400 50  0001 C CNN
F 3 "" H 4600 1400 50  0000 C CNN
	1    4600 1400
	1    0    0    -1  
$EndComp
Text Label 4300 3400 0    60   ~ 0
5V
Text Label 4300 3500 0    60   ~ 0
GND
Wire Wire Line
	3900 2500 3900 3900
Wire Wire Line
	3900 3900 3700 3900
Connection ~ 3900 3500
Wire Wire Line
	3800 3800 3700 3800
Wire Wire Line
	3700 3400 4900 3400
Wire Wire Line
	3900 3050 3700 3050
Wire Wire Line
	3900 2500 3700 2500
Connection ~ 3900 3050
Wire Wire Line
	3700 3500 4900 3500
Connection ~ 3800 3400
Wire Wire Line
	3800 2950 3700 2950
Wire Wire Line
	3800 2400 3700 2400
Connection ~ 3800 2950
Wire Wire Line
	4900 2850 3700 2850
Wire Wire Line
	4900 2750 4900 2300
Wire Wire Line
	4900 2300 3700 2300
Wire Wire Line
	3800 2050 3700 2050
Connection ~ 3800 2400
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
	3800 2400 3800 3800
$Comp
L R R1
U 1 1 582EF7E3
P 4100 1000
F 0 "R1" V 4180 1000 50  0000 C CNN
F 1 "R" V 4100 1000 50  0000 C CNN
F 2 "Wire_Connections_Bridges:WireConnection_1.00mmDrill" V 4030 1000 50  0001 C CNN
F 3 "" H 4100 1000 50  0000 C CNN
	1    4100 1000
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 582EF867
P 4300 1000
F 0 "R2" V 4380 1000 50  0000 C CNN
F 1 "R" V 4300 1000 50  0000 C CNN
F 2 "Wire_Connections_Bridges:WireConnection_1.00mmDrill" V 4230 1000 50  0001 C CNN
F 3 "" H 4300 1000 50  0000 C CNN
	1    4300 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 850  4300 850 
Wire Wire Line
	4100 1150 4100 1950
Connection ~ 4100 1950
Wire Wire Line
	4300 1150 4300 1450
Connection ~ 4300 1450
Text Label 4200 850  0    60   ~ 0
GND
$EndSCHEMATC
