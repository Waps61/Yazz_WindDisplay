# Yazz_WindDisplay
This project is all about building a wind display with a waterproofed Enhanced Nextion 3,5" display and and Arduino Nano clone.

In addition to my completed NMEAtor project I'm building a waterproofed wind display. It has to replace the current 25 year old Robertson Wind display.

The main goal is to read the NMEA0183 data from the wired network, interpret the data SOG, COG, AWA and AWS and show this in a wind display accordingly. Since the display is part of a daisy chained network it has to write the data it has read to the wired netwrok again.

It is build with an Nextion Enhanced 3,5" touch display and an Arduino Nano V3 clone.

Project:  WindDisplay.cpp, Copyright 2020, Roy Wassili
  Contact:  waps61 @gmail.com
  URL:      https://www.hackster.io/waps61
  VERSION:  1.0
  Date:     31-07-2020
  Last
  Update:   15-08-2020 v1.0
            Fixed some minor issues and prepared for relase 1.0
            13-08-2020 v0.12
            Optimized displayData function to fix 000 values on HMI
            10-08-2020 v0.11
            using return values in communication with Nextion
            fixed angle calculation in HMI (now version 0.3)
            04-08-2020 v 0.08
            changed display update via 32-bitvalue var's in HMI and timer
            so we have to send only 1 command to update the complete display
  Achieved: Pre-Fat on board was successfull with v0.1          
  Purpose:  Build an NMEA0183 wind display to replace the old Robertsen wind displays
            supporting following types of tasks:
            - Reading NMEA0183 v2.x data without,
            - Parse the for wind and course and send datat to display
            - 
  
  NOTES:    
        1)  NMEA encoding conventions in short
            An NMEA sentence consists of a start delimiter, followed by a comma-separated sequence
            of fields, followed by the character '*' (ASCII 42), the checksum and an end-of-line marker.
            i.e. <start delimiter><field 0>,<field 1>,,,<field n>*<checksum><end-of-linemarker>
            The start delimiter is either $ or !. <field 0> contains the tag and the remaining fields
            the values. The tag is normaly a 5 character wide identifier where the 1st 2 characters
            identify the talker ID and the last 3 identify the sentence ID.
            Maximum sentence length, including the $ and <CR><LF> is 82 bytes.

            Source: https://gpsd.gitlab.io/gpsd/NMEA.html#_nmea_0183_physical_protocol_layer


        2)  Rx1 and TX1 are reserved for the display communication 38400Bd
            Digital pin 10 (and 11) are reserved for NMEA talker via
            SoftSerial on 38400 Bd
  
  Hardware setup:
  The Arduino Nano V3 has only 1 Rx/Tx port so in NexConfig.h the DEBUG_SERIAL_ENABLE 
  defninition should outcommented and NexSerial has to be set to Serial (instead of Serial2)
  
  Wiring Diagram (for NMEA0183 to NMEA0183 device):
  Arduino   | NMEA device
     Pin 10 |  RX +   
     Pin 11 |  TX + 
  
  Set the pins to the correct one for your development shield or breakout board.
  This program uses these data lines to the Nextion 3,5" Enhanced LCD,
  pin usage as follow:
  Nextion           Rx  | Tx  | 5V  | GND
  Arduino Nano V3   Tx1 | Rx1 | 5V  | GND

  ! Remember to set the pins to suit your display module!


  ---------------
  Terms of use:
  ---------------
  The software is provided "AS IS", without any warranty of any kind, express or implied,
  including but not limited to the warranties of mechantability, fitness for a particular
  purpose and noninfringement. In no event shall the authors or copyright holders be liable
  for any claim, damages or other liability, whether in an action of contract, tort or
  otherwise, arising from, out of or in connection with the software or the use or other
  dealings in the software.

  -----------
  Warning:
  -----------
  Do NOT use this compass in situations involving safety to life
  such as navigation at sea.  
        
  TO DO:    Implement 2-way communication so that incomming NMEA data can be relayed
            to other devices. An NMEA0183 network is typically a daisy chained network

  LIMITATIONS: 
            No relay of NMEA0183 data possible currently. So display needs to be
            implemented as the last node in the daisy chain.
 
  
