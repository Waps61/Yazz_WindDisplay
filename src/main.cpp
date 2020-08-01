#include <Arduino.h>
/*
  Project:  WindDisplay.cpp, Copyright 2020, Roy Wassili
  Contact:  waps61 @gmail.com
  URL:      https://www.hackster.io/waps61
  VERSION:  0.01
  Date:     31-07-2020
  Last
  Update:   31-07-2020 v 0.01
            Initialization of the project
  Achieved:           
  Purpose:  Build an NMEA0183 wind display to replace the old Robertsen wind displays
            supporting following types of tasks:
            - Reading NMEA0183 v2.x data without,
            - Parse the for wind and course and send datat to display
            - 

  NOTE:     NMEA encoding conventions in short
            An NMEA sentence consists of a start delimiter, followed by a comma-separated sequence
            of fields, followed by the character '*' (ASCII 42), the checksum and an end-of-line marker.
            i.e. <start delimiter><field 0>,<field 1>,,,<field n>*<checksum><end-of-linemarker>
            The start delimiter is either $ or !. <field 0> contains the tag and the remaining fields
            the values. The tag is normaly a 5 character wide identifier where the 1st 2 characters
            identify the talker ID and the last 3 identify the sentence ID.
            Maximum sentence length, including the $ and <CR><LF> is 82 bytes.

  Source: https://gpsd.gitlab.io/gpsd/NMEA.html#_nmea_0183_physical_protocol_layer


  Rx1 and TX1 are reserved for the display communication 38400Bd
  Digital pin 10 (and 11) are reserved for NMEA talker via
  SoftSerial on 38400 Bd
  
  Hardware setup:
  The Arduino Nano V3 has only 1 Rx/Tx port so in NexConfig.h the DEBUG_SERIAL_ENABLE defninition
  should outcommented and NexSerial has to be set to Serial (instead of Serial2)
  
  Wiring Diagram (for NMEA0183 to NMEA0183 device)
  Arduino   | NMEA device
     Pin 10 |  RX +   
     Pin 11 |  RX - 
  
  Set the pins to the correct ones for your development shield or breakout board.
  This program use these data lines to the Nextion 3,5" Enhanced LCD,
  pin usage as follow:
                    Rx  Tx  5V  GND
  Arduino Nano V3   Tx1 Rx1 5V  GND

  *Remember to set the pins to suit your display module!


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
        
  TODO: 
  Credit:   
*/


//*** Include the Nextion Display files here
#include <Nextion.h> //All other Nextion classes come with this libray

//*** Since the Arduino Nano V3 has only one Rx/Tx port we need SofSerial to
//*** setup the serial communciation with the NMEA0183 network
#include <SoftwareSerial.h>

//*** Definitions goes here
#define NMEA_BAUD 4800 //baudrate for NMEA communciation 
#define HMI_BAUD 9600 //baudrate for HMI communciation

//*** The NMEA defines in totl 82 characters including the starting 
//*** characters $ or ! and the checksum character *, the checksum
//*** AND last but not least the <CR><LF> chacters.
//*** we define one more for the terminating '\0' character for char buffers
#define NMEA_BUFFER_SIZE 82 // According NEA0183 specs the max char is 82
#define NMEA_TERMINATOR "\r\n"

//*** The maximum number of fields in an NMEA string
//*** The number is based on the largest sentence MDA,
//***  the Meteorological Composite sentence
#define MAX_NMEA_FIELDS 21

#define STACKSIZE 5  // Size of the stack; adjust according use

#define RED  63488
#define GREEN 2016

#define FTM  0.3048        // feet to meters
#define MTF  3.28084       // meters to feet
#define NTK  1.852         // nautical mile to km
#define KTN  0.5399569     // km to nautical mile

#define TALKER_ID "AO"
#define VARIATION "1.57,E" //Varition in Lemmer on 12-05-2020, change 0.11 per year
//*** On my boat there is an ofsett of 0.2V between the battery monitor and what 
//*** is measured by the Robertson Databox
#define BATTERY_OFFSET 0.2 //Volts

//*** define NMEA tags to be used
//*** make sure you know your Talker ID used in the sentences
//*** In my case next to GP for navigation related sentences
//*** II is used for Integrated Instruments and
//*** PS is used for vendor specific tags like Stowe Marine
//*** AO is used for my Andruino generated sentences

/* for lab testing with an NMEA simulator tool
#define _DBK "$SDDBK"   // Depth below keel
#define _DBS "$SDDBS"   // Depth below surface
#define _DBT "$SDDBT"   // Depth below transducer
*/
#define _DBK "$IIDBK"   // Depth below keel
#define _DBS "$IIDBS"   // Depth below surface
#define _DBT "$IIDBT"   // Depth below transducer
#define _HDG "$IIHDG"   // Heading  Deviation & Variation
#define _HDM "$IIHDM"   // Heading Magnetic
#define _HDT "$IIHDT"  // Heading True
#define _MWD "$IIMWD"  // Wind Direction & Speed
#define _MTW "$IIMTW"  // Water Temperature
/* for lab testing with an NMEA simulator tool
#define _MWV "$WIMWV"  // Wind Speed and Angle
*/
#define _MWV "$IIMWV"  // Wind Speed and Angle
#define _ROT "$IIROT"  // Rate of Turn
#define _RPM "$IIRPM"  // Revolutions
#define _RSA "$IIRSA"  // Rudder sensor angle
#define _VDR "$IIVDR"  // Set and Drift
#define _VHW "$IIVHW"  // Water Speed and Heading
#define _VLW "$IIVLW"  //  Distance Traveled through Water
#define _VTG "$IIVTG"  //  Track Made Good and Ground Speed
#define _VWR "$IIVWR"  //  Relative Wind Speed and Angle
#define _XDR "$IIXDR"  //  Cross Track Error  Dead Reckoning
#define _XTE "$IIXTE"  //  Cross-Track Error  Measured
#define _XTR "$IIXTR"  //  Cross Track Error  Dead Reckoning
#define _ZDA "$IIZDA"  //  Time & Date - UTC, day, month, year and local time zone
//*** Some specific GPS sentences
#define _GLL "$GPGLL"   // Geographic Position  Latitude/Longitude
#define _GGA "$GPGGA"   // GPS Fix Data. Time, Position and fix related data for a GPS receiver
#define _GSA "$GPGSA"   // GPS DOP and active satellites
#define _GSV "$GPGSV"   // Satellites in view
#define _RMA "$GPRMA"  // Recommended Minimum Navigation Information
#define _RMB "$GPRMB"  // Recommended Minimum Navigation Information
#define _RMC "$GPRMC"  // Recommended Minimum Navigation Information

//*** Some specific Robertson / Stowe Marine tags below
#define _TON "$PSTON"  // Distance Nautical since reset
#define _TOE "$PSTOE"  // Engine hours
#define _TOB "$PSTOB"  // Battery voltage
#define _TOD "$PSTOD"  // depth transducer below waterline in feet
//*** Arduino generated TAGS
#define _xDR "$" TALKER_ID "" "XDR" // Arduino Transducer measurement
#define _dBT "$" TALKER_ID "" "DBT" // Arduino Transducer measurement
#define _hDG "$" TALKER_ID "" "HDG" // Arduino Transducer measurement
/* SPECIAL NOTE:
  XDR - Transducer Measurement
        1 2   3 4            n
        | |   | |            |
  $--XDR,a,x.x,a,c--c, ..... *hh<CR><LF>
  Field Number:   1:Transducer Type
                2:Measurement Data
                3:Units of measurement
                4:Name of transducer

  There may be any number of quadruplets like this, each describing a sensor. The last field will be a checksum as usual.
  Example:
  $HCXDR,A,171,D,PITCH,A,-37,D,ROLL,G,367,,MAGX,G,2420,,MAGY,G,-8984,,MAGZ*41
*/
/*
   If there is some special treatment needed for some NMEA sentences then
   add the their definitions to the NMEA_SPECIALTY definition
   The pre-compiler concatenates string literals by using "" in between
*/
#define NMEA_SPECIALTY "" _DBK "" _TOB

//#define TEST0 1

#ifdef TEST0
//*** Strucure definietions go here
//*** A structure to hold the NMEA data
typedef struct {
  String fields[ MAX_NMEA_FIELDS ];
  byte nrOfFields=0;
  String sentence="";

}NMEAData;

char nmeaBuffer[NMEA_BUFFER_SIZE+1]={0};

enum NMEAReceiveStatus { INVALID, VALID, RECEIVING, CHECKSUMMING, TERMINATING, NMEA_READY};
byte nmeaStatus = INVALID;
byte nmeaIndex=0;
bool nmeaDataReady = false;




//*** Class definitions go here
 
 /*
  Purpose:  Helper class stacking NMEA data as a part of the multiplexer application
            - Pushin and popping NMEAData structure on the stack for buffer purposes
 */
class NMEAStack
 {
  public:
  NMEAStack();  // Constructor with the size of the stack
  int push( NMEAData _nmea );   // put an NMEAData struct on the stack and returns the lastIndex or -1
  NMEAData pop();               // get an NMEAData struct from the stack and decreases the lastIndex
  int getIndex();               // returns the position of the next free postion in the stack

  private:
  NMEAData stack[STACKSIZE]; // the array containg the structs
  int lastIndex=0;    // an index pointng to the first free psotiion in the stack
 };
 
  NMEAStack::NMEAStack()
  {
    this->lastIndex = 0;
    for(int i=0; i< STACKSIZE; i++ )
    {
      for(int j=0; j<MAX_NMEA_FIELDS; j++ ){
        stack[i].fields[j]="";
      }
      stack[i].nrOfFields = 0;
      stack[i].sentence = "";
    }
  }
  
  int NMEAStack::push( NMEAData _nmea )
  {
    #ifdef DEBUG
    debugWrite( "Pushing on index:"+ String(this->lastIndex ));
    #endif
    if( this->lastIndex < STACKSIZE )
    {
      stack[ this->lastIndex++ ] = _nmea;
      return this->lastIndex;
    } else
    {
      this->lastIndex = STACKSIZE;
      return -1;    // of stack is full
    }
  }

  NMEAData NMEAStack::pop()
  {
    NMEAData nmeaOut;
    nmeaOut.sentence = "";
    if( this->lastIndex>0)
    {
      this->lastIndex--;
      nmeaOut=stack[ this->lastIndex ];
    }
    #ifdef DEBUG
    debugWrite("Popped from index: "+String(lastIndex ));
    #endif
    
    return nmeaOut;   
  }

  int NMEAStack::getIndex()
  {
    return this->lastIndex;
  }

 /*
    Purpose:  An NMEA0183 parser to convert old to new version NMEA sentences
            - Reading NMEA0183 v1.5 data without a checksum,
            - Filtering out current heading data causing incorrect course in fo in navigation app
              i.e. HDG, HDM and VHW messages
            
          
  NOTE:     NMEA encoding conventions in short
            An NMEA sentence consists of a start delimiter, followed by a comma-separated sequence
            of fields, followed by the character '*' (ASCII 42), the checksum and an end-of-line marker.
            i.e. <start delimiter><field 0>,<field 1>,,,<field n>*<checksum><end-of-linemarker>
            The start delimiter is either $ or !. <field 0> contains the tag and the remaining fields
            the values. The tag is normaly a 5 character wide identifier where the 1st 2 characters
            identify the talker ID and the last 3 identify the sentence ID.
            Maximum sentence length, including the $ and <CR><LF> is 82 bytes.

  Source: https://gpsd.gitlab.io/gpsd/NMEA.html#_nmea_0183_physical_protocol_layer
  */
class NMEAParser 
{
 public:
    NMEAParser(NMEAStack *_ptrNMEAStack);
    
    void parseNMEASentence(String nmeaIn ); // parse an NMEA sentence with each part stored in the array
    
    unsigned long getCounter(); //return nr of sentences parsed since switched on

  private:
    NMEAStack *ptrNMEAStack;
    String NMEAFilter = NMEA_SPECIALTY;
    NMEAData nmeaData;  // self explaining
    String nmeaSentence = "";
    void reset(); // clears the nmeaData struct;
    String checksum( String str ); //calculate the checksum for str
    NMEAData nmeaSpecialty( NMEAData nmeaIn ); // special treatment function
    unsigned long counter=0;
};

// ***
// *** NMEAParser Constructor
// *** input parameters:
// *** reference to the debugger object
NMEAParser::NMEAParser(NMEAStack *_ptrNMEAStack)
  : ptrNMEAStack(_ptrNMEAStack)
{
  //*** initialize the NMEAData struct.
reset();

}

/*
 * Clear the nmeaData attribute
 */
void NMEAParser::reset(){
  nmeaData.nrOfFields = 0;
  nmeaData.sentence = "";
  for(int i=0; i< MAX_NMEA_FIELDS; i++){
    nmeaData.fields[i]="";
  }
}

/*

*/
NMEAData NMEAParser::nmeaSpecialty( NMEAData nmeaIn )
{
  String filter = NMEA_SPECIALTY;
  
  NMEAData nmeaOut ;//= nmeaIn;
  #ifdef DEBUG
  debugWrite( " Specialty found... for filter"+filter);
  #endif
  if ( filter.indexOf( nmeaIn.fields[0]) > -1 )
  {
    /* In my on-board Robertson data network some sentences
       are not NMEA0183 compliant. So these sentences need
       to be converted to compliant sentences
    */
    //*** $IIDBK is not NMEA0183 compliant and needs conversion
    //*** Since DBK/DBS sentences are obsolete DBT is used 
    if ( nmeaIn.fields[0] == _DBK ) {
      #ifdef DEBUG
      debugWrite("Found "+String(_DBK));
      #endif
      // a typical non standard DBK message I receive is
      // $IIDBK,A,0017.6,f,,,,
      // Char A can also be a V if invalid and shoul be removed
      // All fields after the tag shift 1 position to the left
      // Since we modify the sentence we'll also put our talker ID in place
      nmeaOut.fields[0]="$AODBT";
      for( int i=1; i<nmeaIn.nrOfFields-2; i++ )
      {
        nmeaOut.fields[i] = nmeaIn.fields[i+1];
      }
      // We need the the feet  to convert to meters and add to string
      float ft = nmeaOut.fields[1].toFloat();
      nmeaOut.fields[3] = String( ft * FTM, 1);
      nmeaOut.fields[4] = "M";
      nmeaOut.fields[5] = "";
      nmeaOut.fields[6] = "";
      nmeaOut.nrOfFields = 7;
      for( int i=0; i< nmeaOut.nrOfFields ; i++)
      {
        #ifdef DEBUG
        debugWrite("Field["+String(i)+"] = "+nmeaOut.fields[i]);
        #endif
        if(i>0) nmeaOut.sentence+=",";
        nmeaOut.sentence += nmeaOut.fields[i];
      }
      nmeaOut.sentence += checksum( nmeaOut.sentence );
      
      #ifdef DEBUG
      debugWrite( " Modified to:"+nmeaOut.sentence);
      #endif
      return nmeaOut;
    }

    //*** current Battery info is in a non NMEA0183 format 
    //*** i.e. $PSTOB,13.2,V
    //*** will be converted to $AOXDR,U,13.2,V,BATT,*CS
    if( nmeaIn.fields[0] == _TOB )
    {
      reset();
      float batt = (nmeaIn.fields[1]).toFloat()+BATTERY_OFFSET;
      nmeaOut.nrOfFields = 5;
      nmeaOut.fields[0] = "$AOXDR";
      nmeaOut.fields[1] ="U";  // the transducer unit
      nmeaOut.fields[2] = String(batt,1);  // the actual measurement value
      nmeaOut.fields[3] = nmeaIn.fields[2]; // unit of measure
      nmeaOut.fields[3].toUpperCase();
      nmeaOut.fields[4] ="BATT";
      for( int i=0; i< nmeaOut.nrOfFields ; i++)
      {
        #ifdef DEBUG
        debugWrite("Field["+String(i)+"] = "+nmeaOut.fields[i]);
        #endif
        if(i>0) nmeaOut.sentence+=",";
        nmeaOut.sentence += nmeaOut.fields[i];
      }
      nmeaOut.sentence += checksum( nmeaOut.sentence );
      return nmeaOut;
    }
  }
  return nmeaOut;
}



// calculate checksum function (thanks to https://mechinations.wordpress.com)
String NMEAParser::checksum( String str )
{
  byte cs = 0;
  for (unsigned int n = 1; n < str.length(); n++)
  {
    if ( str[n] != '$' || str[n] != '!' || str[n] != '*' )
    {
      cs ^= str[n];
    }
  }

  if (cs < 0x10) return "*0" + String(cs, HEX);
  else return "*"+String(cs, HEX);
}

/*
   parse an NMEA sentence into into an NMEAData structure.
*/
void NMEAParser::parseNMEASentence(String nmeaStr)
{
  reset();
  int currentIndex = 0;
  int lastIndex = -1;
  int sentenceLength = nmeaStr.length();
  
  //*** check for a valid NMEA sentence
  #ifdef DEBUG
    debugWrite(" In te loop to parse for "+String(sentenceLength)+" chars");
    #endif
  if ( nmeaStr[0] == '$' || nmeaStr[0] == '!' || nmeaStr[0] == '~' )
  {
    
    //*** parse the fields from the NMEA string
    //*** keeping in mind that indeOf() can return -1 if not found!
    currentIndex = nmeaStr.indexOf( ',',0);
    while ( lastIndex < sentenceLength  )
    {
      
      //*** remember to sepatrate fields with the ',' character
      //*** but do not end with one!
      if ( lastIndex>0 ) nmeaData.sentence += ',';

      //*** we want the data without the ',' in fields array
      if( currentIndex-lastIndex >1 ) // check for an empty field
      {
        nmeaData.fields[ nmeaData.nrOfFields ] = nmeaStr.substring(lastIndex+1, currentIndex );
        nmeaData.sentence += nmeaData.fields[ nmeaData.nrOfFields];
      } else nmeaData.fields[ nmeaData.nrOfFields ] = "0";
      nmeaData.nrOfFields++;
      lastIndex = currentIndex; // searching from next char of ',' !
      currentIndex = nmeaStr.indexOf( ',', lastIndex+1);
      //*** check if we found the last seperator
      if( currentIndex < 0 )
      {
        //*** and make sure we parse the last part of the string too!
        currentIndex = sentenceLength;
      }
      
    }
    if ( NMEAFilter.indexOf( nmeaData.fields[0] ) > -1)
    {
      nmeaData = nmeaSpecialty( nmeaData );
    } else if(nmeaData.sentence.indexOf('*')<1)  //Check for checksum in sentence
    {
      nmeaData.sentence += checksum( nmeaData.sentence);
    }
    #ifdef DEBUG
    debugWrite("Parsed : "+nmeaData.sentence );
    #endif
    nmeaData.sentence += NMEA_TERMINATOR;
    #ifdef DEBUG
    debugWrite("Parsed & terminated: "+nmeaData.sentence );
    #endif
    ptrNMEAStack->push( nmeaData );   //push the struct to the stack for later use; i.e. buffer it
    counter++; // for every sentence pushed the counter increments
  }

  return;
}

unsigned long NMEAParser::getCounter()
{
  return counter;
}

//*** Global scope variable declaration goes here

NMEAStack       NmeaStack;
NMEAParser      NmeaParser(&NmeaStack);
NMEAData        NmeaData;
#endif
NexGauge windGauge = NexGauge(0,1,"gauge");
NexText awa = NexText(0,2,"awa");
NexText aws = NexText(0,3,"aws");
NexText sog = NexText(0,4,"sog");
NexText cog = NexText(0,5,"cog");
NexText wdir = NexText(0,10,"wdir");
NexPicture dispStatus = NexPicture(0,9,"status");

//*** set Baudrates for NMEA and HIM
SoftwareSerial nmeaSerial = SoftwareSerial(10,11,NMEA_BAUD); //

enum nextionStatus {SELFTEST=3,HMI_OK=4,HMI_READY=5};

//*** Test if we can communicate with the HMI by putting the winddsiaply in
//*** resp 90,180,270 and 260 degrees ans showinf some fake results for
//*** sog, aws, cog and awa and finilize with a green led
void hmiCommtest(){
  char tmpTxt[4]={0};
  for(int i=0;i<360;i+=90){
    windGauge.setValue( i+90);
    itoa(i,tmpTxt,10);
    cog.setText( tmpTxt);
    itoa(i%360,tmpTxt,10);
    awa.setText(tmpTxt);
    itoa(i/3,tmpTxt,10);
    sog.setText(tmpTxt);
    aws.setText(tmpTxt);
    if( i<180 ) {
      wdir.setText("<<<");
      wdir.setFgColor(GREEN);
    }
    else {
      wdir.setText(">>>");
      wdir.setFgColor(RED);
    }
    delay(1000);
  }
  dispStatus.setPic(HMI_READY);

}

#ifdef TEST0
/*
 * Start reading converted NNMEA sentences from the stack
 * and write them to Serial Port 2 to send them to the 
 * external NMEA device.
 * Update the display with the value(s) send
 */
byte startTalking(){
  NMEAData nmeaOut;
  
  
  #ifdef DISPLAY_ATTACHED
  double tmpVal=0.0;
  #endif
  
  //*** for all  NMEAData opjects on the stack
  //*** NOTE; the stack has a buffer of NMEA_BUFFER_SIZE objects
  //***       normaly only 1 or 2 should be on the stack
  //***       if the stack is full your timing is out of control

  if( NmeaStack.getIndex()>0 ){
      nmeaOut = NmeaStack.pop();
      //outStr =nmeaOut.sentence;
      

      for(int i=0; i< (int) nmeaOut.sentence.length(); i++){
        //outChar[i]=outStr[i];
        nmeaSerial.write( nmeaOut.sentence[i]);
        
      }
      
      #ifdef DEBUG
      debugWrite(" Sending :" + outStr );
      #endif
  }
  #ifdef DISPLAY_ATTACHED
  // check which screens is active and update with data
  switch (active_menu_button){
  
    case SPEED:
      // speeds are checked for values <100; Higher is non existant
      if(nmeaOut.fields[0]== _RMC){
        tmpVal=nmeaOut.fields[7].toDouble();
        if(tmpVal<100) update_display( tmpVal,screen_units[SPEED],"SOG",Q1);
      }
      if(nmeaOut.fields[0]== _VHW){
        tmpVal=nmeaOut.fields[5].toDouble();
        if(tmpVal<100) update_display( tmpVal,screen_units[SPEED],"STW",Q2);
      }
      if(nmeaOut.fields[0]== _VWR){
        tmpVal=nmeaOut.fields[3].toDouble();
        if(tmpVal<100) update_display( tmpVal,screen_units[SPEED],"AWS",Q3);
        tmpVal=nmeaOut.fields[1].toDouble();
        //char tmpChr[(nmeaOut.fields[2].length())];
        //(nmeaOut.fields[2]).toCharArray(tmpChr,nmeaOut.fields[2].length(),0);
      if(tmpVal<360 && nmeaOut.fields[2]=="R")update_display( tmpVal,screen_units[DEGR],"AWA",Q4);
      else if(tmpVal<360 && nmeaOut.fields[2]=="L")update_display( tmpVal,screen_units[DEGL],"AWA",Q4);
      }
    break;
    case CRS:
        if(nmeaOut.fields[0]== _RMC){
        tmpVal=nmeaOut.fields[8].toDouble();
          if(tmpVal<360)update_display( tmpVal,screen_units[DEG],"TRU",Q1);
        }
        if(nmeaOut.fields[0]== _hDG){
          tmpVal=nmeaOut.fields[1].toDouble();
          if(tmpVal<360)update_display( tmpVal,screen_units[DEG],"MAG",Q2);
        }
        /*
        if(nmeaOut.fields[0]== _xDR ){
          if(nmeaOut.fields[4]== "PITCH"){
            tmpVal=nmeaOut.fields[2].toDouble();
            update_display( tmpVal,screen_units[DEGR],"PITCH",Q3);
          }
          //if we found PITCH we also have ROLL
          if(nmeaOut.fields[8]== "ROLL"){
            tmpVal=nmeaOut.fields[6].toDouble();
            update_display( tmpVal,screen_units[DEGR],"ROLL",Q4);
          }
        }
        */
        if(nmeaOut.fields[0]== _dBT){
          tmpVal=nmeaOut.fields[3].toDouble();
          update_display( tmpVal,screen_units[MTRS],"DPT",Q3);
        }
        if(nmeaOut.fields[0]== _VLW){
          tmpVal=nmeaOut.fields[3].toDouble();
          update_display( tmpVal,screen_units[DIST],"TRP" ,Q4);
        }
        
    break;
    case LOG:
        // Voltage an Temperature are checked <100; Higher is non exsitant.
        if(nmeaOut.fields[0]== _xDR ){
          if(nmeaOut.fields[4]== "BATT"){
            tmpVal=nmeaOut.fields[2].toDouble();
            if(tmpVal<100) update_display( tmpVal,screen_units[VOLT],"BAT",Q1);
          }
        }
        if(nmeaOut.fields[0]== _MTW){
            tmpVal=nmeaOut.fields[1].toDouble();
            if(tmpVal<100) update_display( tmpVal,screen_units[TEMP],"WTR",Q2);
          }
          if(nmeaOut.fields[0]== _VLW){
            tmpVal=nmeaOut.fields[1].toDouble();
            update_display( tmpVal,screen_units[DIST],"LOG",Q3);
          }
          if(nmeaOut.fields[0]== _VLW){
            tmpVal=nmeaOut.fields[3].toDouble();
            update_display( tmpVal,screen_units[DIST],"TRP",Q4);
          }
    break;
    case MEM:
      default:
      if ( (micros() - Stop2)>Timer2 )
      {
        Stop2 = micros();// + Timer2;                                    // Reset timer

    
  
        tmpVal=getFreeSram();
        update_display( tmpVal,"Byte","FREE",Q1);
        
        tmpVal=0;
        update_display( tmpVal,"V.",PROGRAM_VERSION,Q2);

        tmpVal=NmeaStack.getIndex();
        update_display( tmpVal," ","STACK",Q3);
      
        
        tmpVal= NmeaParser.getCounter();
        update_display(tmpVal,"nr","MSG",Q4);
      }
            /*
      if( show_flag){
        // One time instruction for logging when LOG button pressed
        debugWrite( "Connect a cable to the serial port on" ); 
        debugWrite("115200 Baud!");
        debugWrite( String(PROGRAM_NAME)+" "+String(PROGRAM_VERSION));
        debugWrite("Free SRAM:"+ String(getFreeSram()));
        show_flag = false;
      }
      */
      
      Serial.print(nmeaOut.sentence);
      
    break;
  }
  #endif
  
  
  return 1;
}
  

/*
  Decode the incomming character and test if it is valid NMEA data.
  If true than put it the NMEA buffer and call NMEAParser object
  to process incomming and complete MNEA sentence
*/
void decodeNMEAInput(char cIn){
  switch( cIn ){
    case '~':
      // reserved by NMEA
    case '!':
      //for AIS info
    case '$':
      // for general NMEA info
      nmeaStatus = RECEIVING;
      nmeaIndex=0;
      break;
    case '*':
      if(nmeaStatus==RECEIVING){
      nmeaStatus = CHECKSUMMING;
      }
      break;
    case '\n':
    case '\r':
      // in old v1.5 version, NMEA Data may not be checksummed!
      if(nmeaStatus== RECEIVING || nmeaStatus==CHECKSUMMING){
        nmeaDataReady = true;
        nmeaStatus = TERMINATING;
      } else nmeaStatus = INVALID;
      
      break;
    
  }
  switch(nmeaStatus){
    case INVALID:
      // do nothing
      nmeaIndex=0;
      nmeaDataReady = false;
      break;
    case RECEIVING:
    case CHECKSUMMING:
      nmeaBuffer[nmeaIndex] = cIn;
      nmeaIndex++;
      break;
    case TERMINATING:
    
    nmeaStatus = INVALID;
    if( nmeaDataReady){
      nmeaDataReady = false;
      
      // Clear the remaining buffer content with '\0'
      for(int y=nmeaIndex+1; y<NMEA_BUFFER_SIZE+1; y++){
        nmeaBuffer[y]='\0';
      }
      #ifdef DEBUG
      debugWrite( nmeaBuffer);
      #endif
      NmeaParser.parseNMEASentence( nmeaBuffer );

      //clear the NMEAbuffer with 0
      memset( nmeaBuffer, 0, NMEA_BUFFER_SIZE+1);
      nmeaIndex=0;
    }
    
      break;
  }
}

/*
 * Start listeneing for incomming NNMEA sentences
 */
void startListening()
{
  #ifdef DEBUG
    debugWrite("Listening....");
    #endif
  
  while(  nmeaSerial.available()>0 && nmeaStatus != TERMINATING){
    decodeNMEAInput( nmeaSerial.read());
  }
  
  
}
#endif

void setup() {
  // put your setup code here, to run once:
  //Initialize the Nextion Display; the display will run a "selftest" and takes
  // about 15 seconds to finish
  
  nexInit();  
  //nexSerial.begin(HMI_BAUD);
  nmeaSerial.begin(NMEA_BAUD);
  delay(1500);
  sendCommand("page 1");
  uint32_t displayReady=SELFTEST;
  
  // wait until the display status is OK
  while(displayReady!=HMI_OK){
    dispStatus.getPic(&displayReady);
    delay(100);
  }
  hmiCommtest();
  windGauge.setValue(90);
  wdir.setText("<<<");
  wdir.setFgColor(GREEN);
  sog.setText("0.0");
  aws.setText("00");
  cog.setText("000");
  awa.setText("000");
  

}

void loop() {
  // put your main code here, to run repeatedly:
}