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
LIBS:special
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
LIBS:oni
LIBS:rbcrash-cache
EELAYER 27 0
EELAYER END
$Descr User 8268 5827
encoding utf-8
Sheet 1 1
Title "Rainbow Crash"
Date "23 mar 2014"
Rev "13"
Comp "Copyright (c) 2012-2014 Gregor Riepl"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L +12V #PWR01
U 1 1 531ED9DB
P 5200 800
F 0 "#PWR01" H 5200 750 20  0001 C CNN
F 1 "+12V" H 5200 900 30  0000 C CNN
F 2 "" H 5200 800 60  0000 C CNN
F 3 "" H 5200 800 60  0000 C CNN
	1    5200 800 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 531ED9EA
P 5200 1400
F 0 "#PWR02" H 5200 1400 30  0001 C CNN
F 1 "GND" H 5200 1330 30  0001 C CNN
F 2 "" H 5200 1400 60  0000 C CNN
F 3 "" H 5200 1400 60  0000 C CNN
	1    5200 1400
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR03
U 1 1 531ED9F9
P 6300 800
F 0 "#PWR03" H 6300 900 30  0001 C CNN
F 1 "VCC" H 6300 900 30  0000 C CNN
F 2 "" H 6300 800 60  0000 C CNN
F 3 "" H 6300 800 60  0000 C CNN
	1    6300 800 
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P4
U 1 1 531EDA08
P 4650 1000
F 0 "P4" V 4600 1000 40  0000 C CNN
F 1 "12V" V 4700 1000 40  0000 C CNN
F 2 "" H 4650 1000 60  0000 C CNN
F 3 "" H 4650 1000 60  0000 C CNN
	1    4650 1000
	-1   0    0    -1  
$EndComp
$Comp
L CONN_3 P5
U 1 1 531EDA17
P 5150 2400
F 0 "P5" V 5100 2400 50  0000 C CNN
F 1 "TEMP" V 5200 2400 40  0000 C CNN
F 2 "" H 5150 2400 60  0000 C CNN
F 3 "" H 5150 2400 60  0000 C CNN
	1    5150 2400
	1    0    0    -1  
$EndComp
$Comp
L CONN_3X2 P6
U 1 1 531EDA2A
P 5500 3250
F 0 "P6" H 5500 3500 50  0000 C CNN
F 1 "ISP" V 5500 3300 40  0000 C CNN
F 2 "" H 5500 3250 60  0000 C CNN
F 3 "" H 5500 3250 60  0000 C CNN
	1    5500 3250
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 531EDA44
P 4100 4000
F 0 "C13" H 4100 4100 40  0000 L CNN
F 1 "100n" H 4106 3915 40  0000 L CNN
F 2 "SM0805" H 4138 3850 30  0001 C CNN
F 3 "~" H 4100 4000 60  0000 C CNN
	1    4100 4000
	-1   0    0    -1  
$EndComp
$Comp
L INDUCTOR L1
U 1 1 531EDA53
P 1800 900
F 0 "L1" V 1750 900 40  0000 C CNN
F 1 "22u" V 1900 900 40  0000 C CNN
F 2 "~" H 1800 900 60  0000 C CNN
F 3 "~" H 1800 900 60  0000 C CNN
	1    1800 900 
	0    -1   -1   0   
$EndComp
$Comp
L R R6
U 1 1 531EDA62
P 6600 1950
F 0 "R6" V 6680 1950 40  0000 C CNN
F 1 "47" V 6607 1951 40  0000 C CNN
F 2 "SM0805" V 6530 1950 30  0001 C CNN
F 3 "~" H 6600 1950 30  0000 C CNN
	1    6600 1950
	1    0    0    -1  
$EndComp
$Comp
L DIODESCH D1
U 1 1 531EDA72
P 2400 900
F 0 "D1" H 2400 1000 40  0000 C CNN
F 1 "SS16" H 2400 800 40  0000 C CNN
F 2 "~" H 2400 900 60  0000 C CNN
F 3 "~" H 2400 900 60  0000 C CNN
	1    2400 900 
	1    0    0    -1  
$EndComp
$Comp
L LT1129CST-3.3 U5
U 1 1 531EDC11
P 5750 950
F 0 "U5" H 5500 1150 40  0000 C CNN
F 1 "LM2937IMP-3.3" H 5900 1150 40  0000 C CNN
F 2 "SOT-223" H 5750 1050 35  0001 C CIN
F 3 "" H 5750 950 60  0000 C CNN
	1    5750 950 
	1    0    0    -1  
$EndComp
$Comp
L ATTINY25-S U4
U 1 1 531EDC32
P 5550 4050
F 0 "U4" H 4400 4450 40  0000 C CNN
F 1 "ATTINY25-S" H 6550 3650 40  0000 C CNN
F 2 "SO8-200" H 6500 4050 35  0001 C CIN
F 3 "" H 5550 4050 60  0000 C CNN
	1    5550 4050
	-1   0    0    -1  
$EndComp
$Comp
L C C15
U 1 1 531EDF6D
P 5200 1100
F 0 "C15" H 5200 1200 40  0000 L CNN
F 1 "100n" H 5206 1015 40  0000 L CNN
F 2 "SM0805" H 5238 950 30  0001 C CNN
F 3 "~" H 5200 1100 60  0000 C CNN
	1    5200 1100
	1    0    0    -1  
$EndComp
$Comp
L C C16
U 1 1 531EDF9F
P 6300 1100
F 0 "C16" H 6300 1200 40  0000 L CNN
F 1 "10u" H 6306 1015 40  0000 L CNN
F 2 "SM0805" H 6338 950 30  0001 C CNN
F 3 "~" H 6300 1100 60  0000 C CNN
	1    6300 1100
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR04
U 1 1 531EE006
P 4100 3700
F 0 "#PWR04" H 4100 3800 30  0001 C CNN
F 1 "VCC" H 4100 3800 30  0000 C CNN
F 2 "" H 4100 3700 60  0000 C CNN
F 3 "" H 4100 3700 60  0000 C CNN
	1    4100 3700
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 531EE019
P 4100 4400
F 0 "#PWR05" H 4100 4400 30  0001 C CNN
F 1 "GND" H 4100 4330 30  0001 C CNN
F 2 "" H 4100 4400 60  0000 C CNN
F 3 "" H 4100 4400 60  0000 C CNN
	1    4100 4400
	-1   0    0    -1  
$EndComp
Text GLabel 7000 3800 2    60   Input ~ 0
DIMR
Text GLabel 7000 3900 2    60   Input ~ 0
DIMG
Text GLabel 7000 4000 2    60   Input ~ 0
IR
Text GLabel 7000 4100 2    60   Input ~ 0
TEMP
Text GLabel 7000 4200 2    60   Input ~ 0
DIMB
Text GLabel 7000 4300 2    60   Input ~ 0
RESET
$Comp
L TPS61165 U1
U 1 1 531EEFB2
P 1800 1300
F 0 "U1" H 1650 1500 50  0000 C CNN
F 1 "TPS61165" H 1800 1100 50  0000 C CNN
F 2 "" H 1800 1300 60  0000 C CNN
F 3 "" H 1800 1300 60  0000 C CNN
	1    1800 1300
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P1
U 1 1 531EF010
P 3150 1000
F 0 "P1" V 3100 1000 40  0000 C CNN
F 1 "RED" V 3200 1000 40  0000 C CNN
F 2 "" H 3150 1000 60  0000 C CNN
F 3 "" H 3150 1000 60  0000 C CNN
	1    3150 1000
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 531EF04C
P 2600 1100
F 0 "C10" H 2600 1200 40  0000 L CNN
F 1 "10u" H 2606 1015 40  0000 L CNN
F 2 "SM0805" H 2638 950 30  0001 C CNN
F 3 "~" H 2600 1100 60  0000 C CNN
	1    2600 1100
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 531EF079
P 2800 1750
F 0 "R1" V 2880 1750 40  0000 C CNN
F 1 "0.57" V 2807 1751 40  0000 C CNN
F 2 "SM0805" V 2730 1750 30  0001 C CNN
F 3 "~" H 2800 1750 30  0000 C CNN
	1    2800 1750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 531EF098
P 2600 1400
F 0 "#PWR06" H 2600 1400 30  0001 C CNN
F 1 "GND" H 2600 1330 30  0001 C CNN
F 2 "" H 2600 1400 60  0000 C CNN
F 3 "" H 2600 1400 60  0000 C CNN
	1    2600 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 531EF101
P 2800 2100
F 0 "#PWR07" H 2800 2100 30  0001 C CNN
F 1 "GND" H 2800 2030 30  0001 C CNN
F 2 "" H 2800 2100 60  0000 C CNN
F 3 "" H 2800 2100 60  0000 C CNN
	1    2800 2100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 531EF107
P 2100 1500
F 0 "#PWR08" H 2100 1500 30  0001 C CNN
F 1 "GND" H 2100 1430 30  0001 C CNN
F 2 "" H 2100 1500 60  0000 C CNN
F 3 "" H 2100 1500 60  0000 C CNN
	1    2100 1500
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 531EF19E
P 900 1200
F 0 "C2" H 900 1300 40  0000 L CNN
F 1 "100n" H 906 1115 40  0000 L CNN
F 2 "SM0805" H 938 1050 30  0001 C CNN
F 3 "~" H 900 1200 60  0000 C CNN
	1    900  1200
	0    1    -1   0   
$EndComp
$Comp
L C C3
U 1 1 531EF1A4
P 900 1400
F 0 "C3" H 900 1500 40  0000 L CNN
F 1 "220n" H 906 1315 40  0000 L CNN
F 2 "SM0805" H 938 1250 30  0001 C CNN
F 3 "~" H 900 1400 60  0000 C CNN
	1    900  1400
	0    1    -1   0   
$EndComp
Text GLabel 1400 1300 0    60   Input ~ 0
DIMR
Wire Wire Line
	5200 900  5200 800 
Connection ~ 5200 900 
Wire Wire Line
	5200 1400 5200 1300
Wire Wire Line
	5000 1300 6300 1300
Wire Wire Line
	5750 1300 5750 1250
Connection ~ 5200 1300
Connection ~ 5750 1300
Wire Wire Line
	6200 900  6300 900 
Wire Wire Line
	6300 900  6300 800 
Connection ~ 6300 900 
Wire Wire Line
	5000 1300 5000 1100
Wire Wire Line
	4200 3800 4100 3800
Wire Wire Line
	4100 3800 4100 3700
Connection ~ 4100 3800
Wire Wire Line
	4200 4300 4100 4300
Wire Wire Line
	4100 4200 4100 4400
Connection ~ 4100 4300
Wire Wire Line
	6900 3800 7000 3800
Wire Wire Line
	6900 3900 7000 3900
Wire Wire Line
	6900 4000 7000 4000
Wire Wire Line
	6900 4100 7000 4100
Wire Wire Line
	6900 4200 7000 4200
Wire Wire Line
	6900 4300 7000 4300
Wire Wire Line
	2100 900  2200 900 
Wire Wire Line
	2600 900  2800 900 
Connection ~ 2600 900 
Wire Wire Line
	2600 1400 2600 1300
Wire Wire Line
	2100 1300 2400 1300
Wire Wire Line
	2400 1300 2400 1500
Wire Wire Line
	2400 1500 2800 1500
Wire Wire Line
	2800 1500 2800 1100
Connection ~ 2800 1500
Wire Wire Line
	2100 1500 2100 1400
Wire Wire Line
	1500 1300 1400 1300
Wire Wire Line
	1500 1200 1100 1200
Wire Wire Line
	1500 1400 1100 1400
$Comp
L VCC #PWR09
U 1 1 531EF241
P 1500 1100
F 0 "#PWR09" H 1500 1200 30  0001 C CNN
F 1 "VCC" H 1500 1200 30  0000 C CNN
F 2 "" H 1500 1100 60  0000 C CNN
F 3 "" H 1500 1100 60  0000 C CNN
	1    1500 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 1100 1500 1200
Connection ~ 1500 1200
$Comp
L C C1
U 1 1 531EF2AC
P 900 900
F 0 "C1" H 900 1000 40  0000 L CNN
F 1 "4.7u" H 906 815 40  0000 L CNN
F 2 "SM0805" H 938 750 30  0001 C CNN
F 3 "~" H 900 900 60  0000 C CNN
	1    900  900 
	0    1    1    0   
$EndComp
Wire Wire Line
	1100 900  1500 900 
Wire Wire Line
	700  900  700  1500
$Comp
L GND #PWR010
U 1 1 531EF318
P 700 1500
F 0 "#PWR010" H 700 1500 30  0001 C CNN
F 1 "GND" H 700 1430 30  0001 C CNN
F 2 "" H 700 1500 60  0000 C CNN
F 3 "" H 700 1500 60  0000 C CNN
	1    700  1500
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR011
U 1 1 531EF31E
P 1500 800
F 0 "#PWR011" H 1500 750 20  0001 C CNN
F 1 "+12V" H 1500 900 30  0000 C CNN
F 2 "" H 1500 800 60  0000 C CNN
F 3 "" H 1500 800 60  0000 C CNN
	1    1500 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 900  1500 800 
Connection ~ 1500 900 
Wire Wire Line
	2100 1200 2100 900 
Connection ~ 2100 900 
$Comp
L INDUCTOR L2
U 1 1 531EF6DC
P 1800 2400
F 0 "L2" V 1750 2400 40  0000 C CNN
F 1 "22u" V 1900 2400 40  0000 C CNN
F 2 "~" H 1800 2400 60  0000 C CNN
F 3 "~" H 1800 2400 60  0000 C CNN
	1    1800 2400
	0    -1   -1   0   
$EndComp
$Comp
L DIODESCH D2
U 1 1 531EF6E2
P 2400 2400
F 0 "D2" H 2400 2500 40  0000 C CNN
F 1 "SS16" H 2400 2300 40  0000 C CNN
F 2 "~" H 2400 2400 60  0000 C CNN
F 3 "~" H 2400 2400 60  0000 C CNN
	1    2400 2400
	1    0    0    -1  
$EndComp
$Comp
L TPS61165 U2
U 1 1 531EF6E8
P 1800 2800
F 0 "U2" H 1650 3000 50  0000 C CNN
F 1 "TPS61165" H 1800 2600 50  0000 C CNN
F 2 "" H 1800 2800 60  0000 C CNN
F 3 "" H 1800 2800 60  0000 C CNN
	1    1800 2800
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P2
U 1 1 531EF6EE
P 3150 2500
F 0 "P2" V 3100 2500 40  0000 C CNN
F 1 "GREEN" V 3200 2500 40  0000 C CNN
F 2 "" H 3150 2500 60  0000 C CNN
F 3 "" H 3150 2500 60  0000 C CNN
	1    3150 2500
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 531EF6F4
P 2600 2600
F 0 "C11" H 2600 2700 40  0000 L CNN
F 1 "10u" H 2606 2515 40  0000 L CNN
F 2 "SM0805" H 2638 2450 30  0001 C CNN
F 3 "~" H 2600 2600 60  0000 C CNN
	1    2600 2600
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 531EF6FA
P 2800 3250
F 0 "R2" V 2880 3250 40  0000 C CNN
F 1 "0.57" V 2807 3251 40  0000 C CNN
F 2 "SM0805" V 2730 3250 30  0001 C CNN
F 3 "~" H 2800 3250 30  0000 C CNN
	1    2800 3250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 531EF700
P 2600 2900
F 0 "#PWR012" H 2600 2900 30  0001 C CNN
F 1 "GND" H 2600 2830 30  0001 C CNN
F 2 "" H 2600 2900 60  0000 C CNN
F 3 "" H 2600 2900 60  0000 C CNN
	1    2600 2900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 531EF706
P 2800 3600
F 0 "#PWR013" H 2800 3600 30  0001 C CNN
F 1 "GND" H 2800 3530 30  0001 C CNN
F 2 "" H 2800 3600 60  0000 C CNN
F 3 "" H 2800 3600 60  0000 C CNN
	1    2800 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 531EF70C
P 2100 3000
F 0 "#PWR014" H 2100 3000 30  0001 C CNN
F 1 "GND" H 2100 2930 30  0001 C CNN
F 2 "" H 2100 3000 60  0000 C CNN
F 3 "" H 2100 3000 60  0000 C CNN
	1    2100 3000
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 531EF712
P 900 2700
F 0 "C5" H 900 2800 40  0000 L CNN
F 1 "100n" H 906 2615 40  0000 L CNN
F 2 "SM0805" H 938 2550 30  0001 C CNN
F 3 "~" H 900 2700 60  0000 C CNN
	1    900  2700
	0    1    -1   0   
$EndComp
$Comp
L C C6
U 1 1 531EF718
P 900 2900
F 0 "C6" H 900 3000 40  0000 L CNN
F 1 "220n" H 906 2815 40  0000 L CNN
F 2 "SM0805" H 938 2750 30  0001 C CNN
F 3 "~" H 900 2900 60  0000 C CNN
	1    900  2900
	0    1    -1   0   
$EndComp
Text GLabel 1400 2800 0    60   Input ~ 0
DIMG
Wire Wire Line
	2100 2400 2200 2400
Wire Wire Line
	2600 2400 2800 2400
Connection ~ 2600 2400
Wire Wire Line
	2600 2900 2600 2800
Wire Wire Line
	2100 2800 2400 2800
Wire Wire Line
	2400 2800 2400 3000
Wire Wire Line
	2400 3000 2800 3000
Wire Wire Line
	2800 3000 2800 2600
Connection ~ 2800 3000
Wire Wire Line
	2100 3000 2100 2900
Wire Wire Line
	2800 3600 2800 3500
Wire Wire Line
	1500 2800 1400 2800
Wire Wire Line
	1500 2700 1100 2700
Wire Wire Line
	1500 2900 1100 2900
$Comp
L VCC #PWR015
U 1 1 531EF72E
P 1500 2600
F 0 "#PWR015" H 1500 2700 30  0001 C CNN
F 1 "VCC" H 1500 2700 30  0000 C CNN
F 2 "" H 1500 2600 60  0000 C CNN
F 3 "" H 1500 2600 60  0000 C CNN
	1    1500 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 2600 1500 2700
Connection ~ 1500 2700
$Comp
L C C4
U 1 1 531EF736
P 900 2400
F 0 "C4" H 900 2500 40  0000 L CNN
F 1 "4.7u" H 906 2315 40  0000 L CNN
F 2 "SM0805" H 938 2250 30  0001 C CNN
F 3 "~" H 900 2400 60  0000 C CNN
	1    900  2400
	0    1    1    0   
$EndComp
Wire Wire Line
	1100 2400 1500 2400
Wire Wire Line
	700  2400 700  3000
$Comp
L GND #PWR016
U 1 1 531EF73E
P 700 3000
F 0 "#PWR016" H 700 3000 30  0001 C CNN
F 1 "GND" H 700 2930 30  0001 C CNN
F 2 "" H 700 3000 60  0000 C CNN
F 3 "" H 700 3000 60  0000 C CNN
	1    700  3000
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR017
U 1 1 531EF744
P 1500 2300
F 0 "#PWR017" H 1500 2250 20  0001 C CNN
F 1 "+12V" H 1500 2400 30  0000 C CNN
F 2 "" H 1500 2300 60  0000 C CNN
F 3 "" H 1500 2300 60  0000 C CNN
	1    1500 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 2400 1500 2300
Connection ~ 1500 2400
Wire Wire Line
	2100 2700 2100 2400
Connection ~ 2100 2400
$Comp
L INDUCTOR L3
U 1 1 531EF74E
P 1800 3900
F 0 "L3" V 1750 3900 40  0000 C CNN
F 1 "22u" V 1900 3900 40  0000 C CNN
F 2 "~" H 1800 3900 60  0000 C CNN
F 3 "~" H 1800 3900 60  0000 C CNN
	1    1800 3900
	0    -1   -1   0   
$EndComp
$Comp
L DIODESCH D3
U 1 1 531EF754
P 2400 3900
F 0 "D3" H 2400 4000 40  0000 C CNN
F 1 "SS16" H 2400 3800 40  0000 C CNN
F 2 "~" H 2400 3900 60  0000 C CNN
F 3 "~" H 2400 3900 60  0000 C CNN
	1    2400 3900
	1    0    0    -1  
$EndComp
$Comp
L TPS61165 U3
U 1 1 531EF75A
P 1800 4300
F 0 "U3" H 1650 4500 50  0000 C CNN
F 1 "TPS61165" H 1800 4100 50  0000 C CNN
F 2 "" H 1800 4300 60  0000 C CNN
F 3 "" H 1800 4300 60  0000 C CNN
	1    1800 4300
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P3
U 1 1 531EF760
P 3150 4000
F 0 "P3" V 3100 4000 40  0000 C CNN
F 1 "BLUE" V 3200 4000 40  0000 C CNN
F 2 "" H 3150 4000 60  0000 C CNN
F 3 "" H 3150 4000 60  0000 C CNN
	1    3150 4000
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 531EF766
P 2600 4100
F 0 "C12" H 2600 4200 40  0000 L CNN
F 1 "10u" H 2606 4015 40  0000 L CNN
F 2 "SM0805" H 2638 3950 30  0001 C CNN
F 3 "~" H 2600 4100 60  0000 C CNN
	1    2600 4100
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 531EF76C
P 2800 4750
F 0 "R3" V 2880 4750 40  0000 C CNN
F 1 "0.57" V 2807 4751 40  0000 C CNN
F 2 "SM0805" V 2730 4750 30  0001 C CNN
F 3 "~" H 2800 4750 30  0000 C CNN
	1    2800 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 531EF772
P 2600 4400
F 0 "#PWR018" H 2600 4400 30  0001 C CNN
F 1 "GND" H 2600 4330 30  0001 C CNN
F 2 "" H 2600 4400 60  0000 C CNN
F 3 "" H 2600 4400 60  0000 C CNN
	1    2600 4400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 531EF778
P 2800 5100
F 0 "#PWR019" H 2800 5100 30  0001 C CNN
F 1 "GND" H 2800 5030 30  0001 C CNN
F 2 "" H 2800 5100 60  0000 C CNN
F 3 "" H 2800 5100 60  0000 C CNN
	1    2800 5100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 531EF77E
P 2100 4500
F 0 "#PWR020" H 2100 4500 30  0001 C CNN
F 1 "GND" H 2100 4430 30  0001 C CNN
F 2 "" H 2100 4500 60  0000 C CNN
F 3 "" H 2100 4500 60  0000 C CNN
	1    2100 4500
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 531EF784
P 900 4200
F 0 "C8" H 900 4300 40  0000 L CNN
F 1 "100n" H 906 4115 40  0000 L CNN
F 2 "SM0805" H 938 4050 30  0001 C CNN
F 3 "~" H 900 4200 60  0000 C CNN
	1    900  4200
	0    1    -1   0   
$EndComp
$Comp
L C C9
U 1 1 531EF78A
P 900 4400
F 0 "C9" H 900 4500 40  0000 L CNN
F 1 "220n" H 906 4315 40  0000 L CNN
F 2 "SM0805" H 938 4250 30  0001 C CNN
F 3 "~" H 900 4400 60  0000 C CNN
	1    900  4400
	0    1    -1   0   
$EndComp
Text GLabel 1400 4300 0    60   Input ~ 0
DIMB
Wire Wire Line
	2100 3900 2200 3900
Wire Wire Line
	2600 3900 2800 3900
Connection ~ 2600 3900
Wire Wire Line
	2600 4400 2600 4300
Wire Wire Line
	2100 4300 2400 4300
Wire Wire Line
	2400 4300 2400 4500
Wire Wire Line
	2400 4500 2800 4500
Wire Wire Line
	2800 4500 2800 4100
Connection ~ 2800 4500
Wire Wire Line
	2100 4500 2100 4400
Wire Wire Line
	2800 5100 2800 5000
Wire Wire Line
	1500 4300 1400 4300
Wire Wire Line
	1500 4200 1100 4200
Wire Wire Line
	1500 4400 1100 4400
$Comp
L VCC #PWR021
U 1 1 531EF7A0
P 1500 4100
F 0 "#PWR021" H 1500 4200 30  0001 C CNN
F 1 "VCC" H 1500 4200 30  0000 C CNN
F 2 "" H 1500 4100 60  0000 C CNN
F 3 "" H 1500 4100 60  0000 C CNN
	1    1500 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 4100 1500 4200
Connection ~ 1500 4200
$Comp
L C C7
U 1 1 531EF7A8
P 900 3900
F 0 "C7" H 900 4000 40  0000 L CNN
F 1 "4.7u" H 906 3815 40  0000 L CNN
F 2 "SM0805" H 938 3750 30  0001 C CNN
F 3 "~" H 900 3900 60  0000 C CNN
	1    900  3900
	0    1    1    0   
$EndComp
Wire Wire Line
	1100 3900 1500 3900
Wire Wire Line
	700  3900 700  4500
$Comp
L GND #PWR022
U 1 1 531EF7B0
P 700 4500
F 0 "#PWR022" H 700 4500 30  0001 C CNN
F 1 "GND" H 700 4430 30  0001 C CNN
F 2 "" H 700 4500 60  0000 C CNN
F 3 "" H 700 4500 60  0000 C CNN
	1    700  4500
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR023
U 1 1 531EF7B6
P 1500 3800
F 0 "#PWR023" H 1500 3750 20  0001 C CNN
F 1 "+12V" H 1500 3900 30  0000 C CNN
F 2 "" H 1500 3800 60  0000 C CNN
F 3 "" H 1500 3800 60  0000 C CNN
	1    1500 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 3900 1500 3800
Connection ~ 1500 3900
Wire Wire Line
	2100 4200 2100 3900
Connection ~ 2100 3900
$Comp
L CONN_3 P7
U 1 1 531EF7FE
P 6950 2400
F 0 "P7" V 6900 2400 50  0000 C CNN
F 1 "IR" V 7000 2400 40  0000 C CNN
F 2 "" H 6950 2400 60  0000 C CNN
F 3 "" H 6950 2400 60  0000 C CNN
	1    6950 2400
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 531EF810
P 6250 2400
F 0 "R5" V 6330 2400 40  0000 C CNN
F 1 "10k" V 6257 2401 40  0000 C CNN
F 2 "SM0805" V 6180 2400 30  0001 C CNN
F 3 "~" H 6250 2400 30  0000 C CNN
	1    6250 2400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6600 2400 6500 2400
$Comp
L VCC #PWR024
U 1 1 531EF882
P 6600 1600
F 0 "#PWR024" H 6600 1700 30  0001 C CNN
F 1 "VCC" H 6600 1700 30  0000 C CNN
F 2 "" H 6600 1600 60  0000 C CNN
F 3 "" H 6600 1600 60  0000 C CNN
	1    6600 1600
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR025
U 1 1 531EF888
P 6600 2700
F 0 "#PWR025" H 6600 2700 30  0001 C CNN
F 1 "GND" H 6600 2630 30  0001 C CNN
F 2 "" H 6600 2700 60  0000 C CNN
F 3 "" H 6600 2700 60  0000 C CNN
	1    6600 2700
	-1   0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 531EF88E
P 4450 2400
F 0 "R4" V 4530 2400 40  0000 C CNN
F 1 "100" V 4457 2401 40  0000 C CNN
F 2 "SM0805" V 4380 2400 30  0001 C CNN
F 3 "~" H 4450 2400 30  0000 C CNN
	1    4450 2400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4800 2400 4700 2400
$Comp
L VCC #PWR026
U 1 1 531EF8FA
P 4800 2100
F 0 "#PWR026" H 4800 2200 30  0001 C CNN
F 1 "VCC" H 4800 2200 30  0000 C CNN
F 2 "" H 4800 2100 60  0000 C CNN
F 3 "" H 4800 2100 60  0000 C CNN
	1    4800 2100
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 531EF900
P 4800 2700
F 0 "#PWR027" H 4800 2700 30  0001 C CNN
F 1 "GND" H 4800 2630 30  0001 C CNN
F 2 "" H 4800 2700 60  0000 C CNN
F 3 "" H 4800 2700 60  0000 C CNN
	1    4800 2700
	-1   0    0    -1  
$EndComp
$Comp
L C C17
U 1 1 531EF90B
P 7300 2400
F 0 "C17" H 7300 2500 40  0000 L CNN
F 1 "100n" H 7306 2315 40  0000 L CNN
F 2 "SM0805" H 7338 2250 30  0001 C CNN
F 3 "~" H 7300 2400 60  0000 C CNN
	1    7300 2400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6600 2300 6600 2200
Wire Wire Line
	6600 2500 6600 2700
Wire Wire Line
	7300 2600 6600 2600
Connection ~ 6600 2600
Wire Wire Line
	6600 2200 7300 2200
Connection ~ 6600 2200
Wire Wire Line
	6600 1700 6600 1600
$Comp
L C C14
U 1 1 531EFB81
P 5500 2400
F 0 "C14" H 5500 2500 40  0000 L CNN
F 1 "100n" H 5506 2315 40  0000 L CNN
F 2 "SM0805" H 5538 2250 30  0001 C CNN
F 3 "~" H 5500 2400 60  0000 C CNN
	1    5500 2400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4800 2100 4800 2300
Wire Wire Line
	4800 2700 4800 2500
Wire Wire Line
	4800 2200 5500 2200
Connection ~ 4800 2200
Wire Wire Line
	5500 2600 4800 2600
Connection ~ 4800 2600
Text GLabel 4100 2400 0    60   Input ~ 0
TEMP
Wire Wire Line
	4200 2400 4100 2400
Text GLabel 5900 2400 0    60   Input ~ 0
IR
Wire Wire Line
	5900 2400 6000 2400
$Comp
L VCC #PWR028
U 1 1 531F001D
P 5900 3000
F 0 "#PWR028" H 5900 3100 30  0001 C CNN
F 1 "VCC" H 5900 3100 30  0000 C CNN
F 2 "" H 5900 3000 60  0000 C CNN
F 3 "" H 5900 3000 60  0000 C CNN
	1    5900 3000
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR029
U 1 1 531F0023
P 5900 3400
F 0 "#PWR029" H 5900 3400 30  0001 C CNN
F 1 "GND" H 5900 3330 30  0001 C CNN
F 2 "" H 5900 3400 60  0000 C CNN
F 3 "" H 5900 3400 60  0000 C CNN
	1    5900 3400
	-1   0    0    -1  
$EndComp
Text GLabel 5000 3100 0    60   Input ~ 0
DIMG
Text GLabel 5000 3200 0    60   Input ~ 0
IR
Text GLabel 5000 3300 0    60   Input ~ 0
RESET
Text GLabel 6000 3200 2    60   Input ~ 0
DIMR
Wire Wire Line
	5900 3100 5900 3000
Wire Wire Line
	6000 3200 5900 3200
Wire Wire Line
	5900 3300 5900 3400
Wire Wire Line
	5100 3100 5000 3100
Wire Wire Line
	5000 3200 5100 3200
Wire Wire Line
	5000 3300 5100 3300
Connection ~ 700  2900
Connection ~ 700  2700
Connection ~ 700  4200
Connection ~ 700  4400
Connection ~ 700  1200
Connection ~ 700  1400
Wire Wire Line
	2800 2000 2800 2100
Wire Wire Line
	5000 900  5300 900 
$EndSCHEMATC
