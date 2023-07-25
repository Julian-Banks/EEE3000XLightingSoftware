

//Version Notes
/*Version 1.11
 * 
 * Testing Audio library
 * 
 * Version 1.10
 * taking out FastLED completely (DONT REMOVE THE LIBRARY CAUSE I AM USING THE SATURATING MATHS)
 */
/*Version 1.9 
 * BIg changes! 
 * MOVING OVER TO OCTOWS2811!
 * lots of small tweaks to functions to improve them
 */
/*
 * Version 1.8
 *Changing over to Teensy 4.1 
 *Switched from AdaFruit GFX to the U8Glib in order to get the OLED working. 
 *Switched CS to pin 10 and DC to pin 9 for the 4.1 pins
 *
 */
 
//Libraries
#include <U8glib.h>
#include <FastLED.h>
#include <arduinoFFT.h>
#include <OctoWS2811.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputAnalog         adc1;           //xy=227.99999618530273,267.9999885559082
AudioAnalyzePeak         peak1;          //xy=461.99999237060547,264.99999618530273
AudioConnection          patchCord1(adc1, peak1);
// GUItool: end automatically generated code



#define WINDOW_SIZE 8

//STATE VARIABLES
  #define MainMenu 0
  #define FuncMenu 1
  #define DefaultDisplay 2
  #define SetCycleSpeed 3
  uint8_t STATE = DefaultDisplay;
//Global Current Function
  int current_function = 0;
  bool new_call = false;
  bool liveMixingStage ;
  bool liveMixingFloor;
  bool Auto_Mixing = false;
  bool Auto_Cycle = true;
  int  CycleSpeed = 5;
  bool btnClickFunc =false;
  
  const int numMappings = 5;
  int mode[numMappings] = {0,1,5,3,4};

//Fire variables
  int FRAMES_PER_SECOND =60;
  int COOLING = 55;
  int SPARKING =120;
  bool gReverseDirection = false;
  CRGBPalette16 gPal;

  int INDEX = 0;
  int VALUE = 0;
  int SUM = 0;
  int READINGS[WINDOW_SIZE];
  int AVERAGED = 0; 

//POTS 
  const int NPots = 4;
  const int potPin[NPots] = {A9,A7,A6,A8};

//BUTTONS 
  const int NButtons = 8;
  const int buttonPin[NButtons] = {5,6,7,4,3,2,1,0};

//TOGGLES
  const int NToggles = 4;
  const int togglePin[NToggles] = {34,35,36,37};

//SLIDERS 
  const int NSliders = 4;
  const int sliderPin[NSliders] = {A16,A17,A0,A1};

//LED DISPLAY
  const int LED_disp = 8;

//BARS PIN OUT
  const int Bar1 = 31;
  const int Bar2 = 28;
  const int Bar3 = 29;
  const int Bar4 = 30;
  const int Bar5 = 25;
  const int Bar6 = 32;

//ENCODER 
  const int SW = 33;
  const int DT = 24;
  const int CLK = 12;
  int RotPosition = 0; 
  int rotation;  
  int value;
  boolean LeftRight;

//OLED 
U8GLIB_SSD1309_128X64 u8g(19, 18, 10, 9, 11);
 uint8_t perfMode ;
//String FuncName;
//
//const int numParam  = 3;
//char param[numParam];


//AUDIO INPUT 
  const int Audio_in = A2;
  const int Audio_in_RCA = A2;
  
  float peakValue ; 
//MILLIS
  unsigned long colourStartTime;
  unsigned long startTime;
  unsigned long b2fStartTime;
  unsigned long fireStartTime;
  unsigned long pulseStartTime ;
  unsigned long cycleStartTime;
  unsigned long currentTime;
   unsigned long debounce;
   unsigned long RandomAITime = 0;
   unsigned long RandomAITimeOFF = 0;
    unsigned long SolidAllFade = 0;
    unsigned long waveStart = 0;
//GLOBAL COLOUR VARIABLES
  uint8_t  BLUE_STAGE     = 0;
  uint8_t GREEN_STAGE    = 0;
  uint8_t RED_STAGE      = 0;
  uint8_t BLUE_FLOOR     = 0;
  uint8_t GREEN_FLOOR    = 0;
  uint8_t RED_FLOOR      = 0;

  uint8_t Auto_Hue       = 0;

//Variables for solid all 
 uint8_t fadeCounter;

//Variables for Back to Front mode (WHoosh)
  int i_b2f =0;


//DEFINE PULSE SETTINGS
 int  pulseCount =0;
 



//DISPLAY LED
  const int LED_DISPLAY = 8;
  #define NUM_LEDS        6
  #define COLOR_ORDER     GRB           // If colours look wrong, play with this
  #define CHIPSET         WS2812B   


//LED BARS
  #define NUM_LEDS_BAR    140
  #define NUM_BARS        6
  #define BAR_COLOR_ORDER RGB   // Might be different to the display LEDs
  #define CHIPSET         WS2812B






//FOURIER SETTINGS
    #define SAMPLES         128         // Must be a power of 2
    #define NUM_BANDS       20          // Number of frequency bands after the FFT
    #define SAMPLING_FREQ   8000       // Hz, Determines maximum frequency that can be analysed by the FFT: Fmax=sampleF/2. (MUST BE LESS THAN 40 000)
    #define AMPLITUDE       20          // Depending on your audio source level, you may need to alter this value. Can be used as a 'sensitivity' control.

    unsigned int sampling_period_us;
    byte peak[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};              // The length of these arrays must be >= NUM_BANDS
    int oldBarBrightness[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int bandValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    double vReal[SAMPLES];
    double vImag[SAMPLES];
    double dataIn[SAMPLES];
 
//GRIFFINS ARRAYS
  double fftCalc[16];
  int fftResult[16]; 
  double fftBin[512];
  double gmin = 100000;
  double gave = 0;
  double gmax = 0;
//Griffins variables
  int count =0;
  unsigned long old ;
  arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);
////OCTO STUFF! :)))))
const int numPins = 7;
byte pinList[numPins] = {Bar6, Bar5, Bar4, Bar3, Bar2, Bar1 ,LED_DISPLAY};
const int numStrips = 7;
const int bytesPerLED = 3;  // change to 4 if using RGBW
DMAMEM int displayMemory[NUM_LEDS_BAR * numPins * bytesPerLED / 4];
int drawingMemory[NUM_LEDS_BAR * numPins * bytesPerLED / 4];
const int config = WS2811_GRB | WS2811_800kHz;
OctoWS2811 ledsOcto(NUM_LEDS_BAR, displayMemory, drawingMemory, config, numPins, pinList);

//smoothing stuff for sliders
const int numReadings = 10;
const int numItems = 4;
int readings[numReadings][numItems];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int itemIndex = 0; 
int total[numItems] ;              // the running total




void setup(){
  current_function = 4;
  STATE = DefaultDisplay; 
  Serial.begin(9600);

//FFT STUFF
 sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));

//Button Pull-up
  for (int i = 0; i < NButtons; i++) {
    pinMode(buttonPin[i], INPUT_PULLUP);
  }

//Toggle Pull-up
   for (int i = 0; i < NToggles; i++) {
    pinMode(togglePin[i], INPUT_PULLUP);
   }
  
//Display setup
  //uint8_t U8GLIB::begin(void);
  u8g.begin();

  // picture loop
  u8g.firstPage();
  do {
  drawSetup();
  } while( u8g.nextPage() );

//OCTO initialise 
  ledsOcto.begin();
  ledsOcto.show();
//Initilise variables for Smoothing pots
  for (int thisItem = 0; thisItem< numItems ; thisItem++){
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading][thisItem] = 0;
   }
  }
  
//ADDING ALL OF THE BARS

    for (int i = 0; i < NUM_LEDS_BAR*6+6; i++) {  // RED_STAGE  GREEN_FLOOR
       if(i<NUM_LEDS_BAR*6){
         ledsOcto.setPixel(i, 0, 100,50);
       }else{
         ledsOcto.setPixel(i, 100, 0,50);
       }
  }
   ledsOcto.show(); 
 delay(2000);
 
//ENCODER 
   pinMode (CLK,INPUT);
   pinMode (DT,INPUT);
   pinMode (SW,INPUT_PULLUP);
  b2fStartTime = millis();
  colourStartTime = millis();
  startTime = millis();
  pulseStartTime = millis();
  debounce=millis();
  
 
//Turn them all off again
    for (int i = 0; i < NUM_LEDS_BAR*6+6; i++) {  // RED_STAGE  GREEN_FLOOR
       if(i<NUM_LEDS_BAR*6){
         ledsOcto.setPixel(i, 0, 0,0);
       }else{
         ledsOcto.setPixel(i, 0, 0,0);
       }
  }
    ledsOcto.show(); 
  
    AudioMemory(12);
}

void loop(){
  long startTime = millis();    
  //Set the state variables //MUST CHANGE THESE TO INTERRUPTS WHEN I GET THE CHANCE!!!!
 if(STATE==FuncMenu){
    if(btnClickFunc){
     if(RotPosition==6){STATE=DefaultDisplay;}
     else{MapModes(RotPosition);}
    }
    
  }else{
    function_select();
    liveMixingStage = true ; //digitalRead(togglePin[3]);
    liveMixingFloor = digitalRead(togglePin[2]);
    Auto_Mixing     = digitalRead(togglePin[1]);
  }
  

 if (digitalRead(togglePin[3])){
    read_encoder();  
    perfMode = 0;
 }else{
     if (perfMode == 0){
          u8g.firstPage();
          do {
            drawPerfMode();
          } while( u8g.nextPage() );
        }    
     perfMode = 1;
 }
//Check if the system is auto mixing or not
    if (Auto_Mixing){

      auto_colour_mixing();
     
    }else{
        
      colour_mixing();
   
    }

//Auto Cycle Functions
if(Auto_Cycle){
  auto_cycle_function();
}

   
//RUN THE STOBEE BABY (only runs if the button is Being Pressed)
   if(!(digitalRead(buttonPin[1]))){
    strobe();
   }
  else{
//RUN THE FUNCTION THAT IS GOING TO THE LIGHTS
  switch (current_function) {

    //Function mapped to function button 1 (BTNPin3)
    case 0:
        call_mode(mode[0]);
    break;
    
    //Function mapped to function button 2 (BTNPin4)
    case 1:
        call_mode(mode[1]);
    break;

    //Function mapped to function button 3 (BTNPin5)
    case 2:
       call_mode(mode[2]);
    break;

    //Function mapped to function button 4 (BTNPin6)
    case 3:
        call_mode(mode[3]);
    break;

    //Function mapped to function button 5 (BTNPin7)
    case 4:
       call_mode(mode[4]);
    break;
  

    default:
    solid_all();
    
    }  
  }
 
   Serial.println(millis()-startTime); 
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SYSTEN FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void function_select (){

//Loop through all of the function buttons

  for (int i=3; i<8;i++){
    //If it has been pushed then set the current function to the corresponding number and update new call. 
    if (digitalRead(buttonPin[i])==0){
      current_function=i-3;
      new_call = true;
    }
   }
}

void MapModes(int RotPosition){

  for (int i=3; i<8;i++){
    //If it has been pushed then set the current function to the corresponding number and update new call. 
      if (digitalRead(buttonPin[i])==0){
        mode[i-3]=RotPosition;
        btnClickFunc = false;//Enable the encoder again
       }
     }
  
}
 
void colour_mixing(){

   // subtract the last reading:
   for (int i = 0; i<numItems; i++){
   total[i] = total[i] - readings[readIndex][i];
   }
    // read from the sensor:
    readings[readIndex][0] = map(analogRead(sliderPin[3]),0,1023,0,100);
    readings[readIndex][1] = map(analogRead(sliderPin[0]), 0, 1023, 0, 255);
    readings[readIndex][2] = map( analogRead(sliderPin[1]), 0, 1023, 0, 255);
    readings[readIndex][3] = map(analogRead(sliderPin[2]), 0, 1023, 0, 255);
    // add the reading to the total:
    for (int i = 0 ; i<numItems; i++){
     total[i] = total[i] + readings[readIndex][i];
    }
    // advance to the next position in the array:
    readIndex = readIndex + 1;
  
    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
      itemIndex++;
    }  
      if (itemIndex >= numItems) {
      // ...wrap around to the beginning:
      itemIndex=0;
    }  
    
    float brightnessValue =  total[0] / numReadings;
    if (brightnessValue < 3) brightnessValue = 0; 
    brightnessValue = brightnessValue/100; 
    uint8_t RED =  (total[1] / numReadings)*brightnessValue ;//    map(analogRead(sliderPin[0]), 0, 1023, 0, 255)*brightnessValue;
    if (RED < 4) RED = 0; 
    uint8_t GREEN = (total[2] / numReadings)*brightnessValue; //map( analogRead(sliderPin[1]), 0, 1023, 0, 255)*brightnessValue;
    if (GREEN < 4) GREEN = 0;
    uint8_t BLUE = (total[3] / numReadings)*brightnessValue; //map(analogRead(sliderPin[2]), 0, 1023, 0, 255)*brightnessValue;
    if (BLUE <4) BLUE = 0;


    
  for (int i = 6*NUM_LEDS_BAR; i < NUM_LEDS_BAR*6+6; i++) {
    ledsOcto.setPixel(i, RED, GREEN, BLUE);
  }
  ledsOcto.show();
     
     if(liveMixingStage){       //LIVE MIXING STAGE OR if Colour SEND (BTN0) is pushed Then update the stage colours  // Put this back if you want to change nav buttons to push buttons ||(digitalRead(buttonPin[0])==0)    
      RED_STAGE   = RED; // scale the brightness 
      GREEN_STAGE = GREEN;
      BLUE_STAGE  = BLUE;     
    }
    if(liveMixingFloor){        //LIVE MIXING FLOOR OR if Colour SEND (BTN2) is pushed Then update the stage colours // Put this back if you want to change nav buttons to push buttons ||(digitalRead(buttonPin[2])==0)
      RED_FLOOR   =RED;// scale the brightness 
      GREEN_FLOOR =GREEN;
      BLUE_FLOOR  =BLUE; 
    }
 
 }


void auto_colour_mixing(){  
   
  int mixSpeed = set_strobe();//USING THE STROBE TO CONTROLL SPEED CAUSE WE DONT HAVE ANOTHER ONE :(((((
  float bright = (map(set_brightness(),0,1023,0,255)); //CURRENTLY MAPPED TO RED SLIDER CAUSE BRIGHTNESS IS BROKEN!!!!!!!!!!!!!!
  

    //Set the display bars colour values.    
  // Serial.print("STROBE HZ: ");
 
  //Update brightness everytime. 
    CRGB mixColour  = CHSV(Auto_Hue,255,bright);
   // Serial.println(Auto_Hue);

   uint8_t RED = mixColour.red;
   uint8_t GREEN = mixColour.green;
   uint8_t BLUE  = mixColour.blue;
    for (int i = 6*NUM_LEDS_BAR; i < NUM_LEDS_BAR*6+6; i++) {
     ledsOcto.setPixel(i, RED, GREEN, BLUE);
  }
  ledsOcto.show();
   // fill_solid(leds[6], NUM_LEDS, mixColour);  
  //Only increment Hue if enought millis have passed.
  currentTime= millis();
  //mixSpeed
  if((currentTime-mixSpeed)>colourStartTime){
      if(Auto_Hue<255){
        if (Auto_Hue==15){
          Auto_Hue=160;
        }
        if ((Auto_Hue>15 )&&(Auto_Hue<160)&& (mode[current_function]==2)){
          Auto_Hue=160;
        }
        Auto_Hue++;
     //   Serial.println(Auto_Hue);
      }else{Auto_Hue=0;}
      colourStartTime = millis();
    }

   
    //Convert the HSV TO A CRGB
    CRGB mixColourCRBG = mixColour;
    //Assign all the usual variables to their value
    RED_STAGE   = mixColourCRBG.red;  
    BLUE_STAGE  = mixColourCRBG.blue;
    GREEN_STAGE = mixColourCRBG.green;
    RED_FLOOR   = mixColourCRBG.red;  
    BLUE_FLOOR  = mixColourCRBG.blue;
    GREEN_FLOOR = mixColourCRBG.green;   
  
 }


void auto_cycle_function() {

    currentTime = millis();
      //If enough time has passed then increment the function counter. 
      if((CycleSpeed*60000)<(currentTime-cycleStartTime)){
        if (current_function<4){ //WIll need to update this if we change how many buttons are mamped to buttons!!!
            current_function++;
        }else{
            current_function=0;
          }
        cycleStartTime=millis();        
      }
      
      
  }



//
//FETCH READINGS FROM POTS
  int set_speed(){
    int speed_=analogRead(potPin[0]);
    return speed_;
  }

  int set_brightness(){
    int bright_ = analogRead(sliderPin[3]);
    if (Auto_Mixing){
      bright_ = analogRead(sliderPin[3]);
    }
    return bright_;
  }

  int set_sensitivity(){
    int sens = analogRead(potPin[1]);
    return sens;
  }

  int set_fade(){
    int fade = analogRead(potPin[2]);
    return fade;
  }

  int set_strobe(){
    int strobe = analogRead(potPin[3]);
    return strobe;
  }


 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AUDIO FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Griffins FFT Function
bool fFFT (double shrink = 1.001){

  // Sample the audio pin
  for (int i = 0; i < SAMPLES; i++) {
   // currentTime = micros();
    dataIn[i] = (analogRead(Audio_in));
    vReal[i] = dataIn[i];
    vImag[i] = 0;
   // while ((micros() - currentTime) < sampling_period_us) {}
    }    
  // COMPUTE FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_BLACKMAN_HARRIS, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
    for (int i = 0; i < SAMPLES; i++) {                     
      double t = 0.0;
      t = abs(vReal[i]);
      t = t / 16.0;                                         // Reduce magnitude. Want end result to be linear and ~4096 max.
      fftBin[i] = t;
    }

  //ANALYSE FFT RESULTS
      fftCalc[0] = (fftAdd(1,2))/1 ;        // 60 - 100
      fftCalc[1] = (fftAdd(2,3))/1 ;        // 80 - 120
      fftCalc[2] = (fftAdd(3,4))/1 ;        // 100 - 160
      fftCalc[3] = (fftAdd(4,5))/1 ;        // 140 - 200
      fftCalc[4] = (fftAdd(5,6))/1;       // 180 - 260
      fftCalc[5] = (fftAdd(6,7))/2 ;      // 240 - 340
      fftCalc[6] = (fftAdd(7,8))/3 ;      // 320 - 440
      fftCalc[7] = (fftAdd(8,9))/4 ;      // 420 - 600
      fftCalc[8] = (fftAdd(9,10))/6 ;     // 580 - 760
      fftCalc[9] = (fftAdd(10,11)) /8;     // 740 - 980
      fftCalc[10] = (fftAdd(11,12))/12 ;    // 960 - 1300
      fftCalc[11] = (fftAdd(12,13))/15 ;    // 1280 - 1700
      fftCalc[12] = (fftAdd(13,14))/25 ;   // 1680 - 2240
      fftCalc[13] = (fftAdd(14,15)) /35;  // 2220 - 2960
      fftCalc[14] = (fftAdd(15,16)) /45;  // 2940 - 3900
      fftCalc[15] = (fftAdd(15,16))/60 ;
  gmin = gmin*1.00001;
  if ((fftCalc[0]+ fftCalc[1]+fftCalc[2]+fftCalc[3]) < gmin) {gmin = (fftCalc[0]+ fftCalc[1]+fftCalc[2]+fftCalc[3]);}

  gmax = gmax*(2-shrink);
  if ((fftCalc[0]*4) > gmax) {gmax = (fftCalc[0]*7+ fftCalc[1]+fftCalc[2]+fftCalc[3])/2;}
  if ((fftCalc[1]*4) > gmax) {gmax = (fftCalc[0]+ fftCalc[1]*7+fftCalc[2]+fftCalc[3])/2;}
  if ((fftCalc[2]*4) > gmax) {gmax = (fftCalc[0]+ fftCalc[1]+fftCalc[2]*7+fftCalc[3])/2;}
  if ((fftCalc[3]*4) > gmax) {gmax = (fftCalc[0]+ fftCalc[1]+fftCalc[2]+fftCalc[3]*7)/2;}


    for (int i=0; i < 16; i++) {
        //fftCalc[i] = fftCalc[i] * sampleGain / 400 + fftCalc[i]/160.0;
        fftCalc[i] = (fftCalc[i]-gmin/4)*(254/(gmax/4-gmin/4));   
    }
        for (int i=0; i < 16; i++) {
        // fftResult[i] = (int)fftCalc[i];
        fftResult[i] = constrain((int)fftCalc[i],0,254);
      }
     
      int fftMax=0;
      for (int i =0; i<16;i++){
        if (fftResult[i]>fftMax){
          fftMax = fftResult[i];
        }
      }

  int sensitivity = map(analogRead(potPin[1]),0,1023,0,300);

  if (fftMax>sensitivity){
     return true;
  }
  else{
    return false;
  }

  }
 
double fftAdd( int from, int to) {
  int i = from;
  double result = 0;
  while ( i <= to) {
    result += fftBin[i++];
  }
  
  return result;
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MENU CODE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Setup DisplayCode
void drawSetup(void) {
u8g.setFont( u8g_font_8x13r );
u8g.drawStr( 0,10, "FRANK AND BANKS");
u8g.drawStr( 0,29,"FUCKEN SICK");
u8g.drawStr( 0,48,"LIGHTS!!!");
}
//Setup Performance Mode
void drawPerfMode(void) {
u8g.setFont( u8g_font_8x13r );
u8g.drawStr( 0,10, "PERFORMANCE");
u8g.drawStr( 0,29,"MODE");
u8g.drawStr( 0,48,"BABBYYYY!!!");
}

//Encoder Menu Navigation Code
//*********************************************************************************************************************************

void read_encoder(){
 
bool btnEncoder;
int readDT;
btnEncoder = digitalRead(SW);
value = digitalRead(CLK);
 readDT   = digitalRead(DT);

  
  u8g.setFont( u8g_font_6x10r );
  //Read STATE and display current menu
    switch(STATE){
        case DefaultDisplay:
          // picture loop
            u8g.firstPage();  
            do {
              DefaultMenu();
            } while( u8g.nextPage() );
         break;
        case MainMenu:
            // picture loop
            u8g.firstPage();  
            do {
                MainMenuDisplay(RotPosition); 
            } while( u8g.nextPage() );              
        break;
        case FuncMenu:
            // picture loop
            u8g.firstPage();  
            do {
                MapFuncMenu(RotPosition);
            } while( u8g.nextPage() );     
        break;
        case SetCycleSpeed:
            // picture loop
            u8g.firstPage();  
            do {
                SetCycleSpeedMenu(RotPosition);
            } while( u8g.nextPage() );                 
            CycleSpeed = RotPosition;
        break;
        default:
            // picture loop
            u8g.firstPage();  
            do {
              DefaultMenu();
            } while( u8g.nextPage() );
        break;             
    }
    
 if ((debounce+150)<millis()){
if (!btnClickFunc){
if (value !=rotation){ // we use the DT pin to find out which way we turning.
  if (readDT !=value) {  // Clockwise
      LeftRight = true;
      RotPosition ++;
  } else {    //Counterclockwise
      LeftRight = false;
      RotPosition--;
  }
  rotation = value;
  }


  if (digitalRead(buttonPin[0])==0) {
    RotPosition ++;
  }
  if(digitalRead(buttonPin[2])==0) {
      RotPosition--;
    }
  

 //Serial.print("ENCODER:");
// Serial.println(RotPosition);
}


  
  //Mod Position 
if (STATE==MainMenu){
   if (RotPosition>2){
   RotPosition=0;
  }
  if (RotPosition<0){
   RotPosition=2;
  }
}
if (STATE==FuncMenu){
   if (RotPosition>6){
   RotPosition=0;
  }
  if (RotPosition<0){
   RotPosition=6;
  }
}
if (STATE==SetCycleSpeed){
  if(RotPosition<1){
    RotPosition=1;
  }
}

  


  
//Perform an action on the encoder click

    if (!btnEncoder)  {
    switch (STATE){
      case DefaultDisplay: 
        STATE = MainMenu;
      break;

      case MainMenu:
        switch(RotPosition){
     
          case 0:
            Auto_Cycle=!Auto_Cycle;  
            if(Auto_Cycle){STATE = SetCycleSpeed;}         
          break;

          case 1:
            STATE=FuncMenu;
          break;

           case 2:
            STATE=DefaultDisplay;
          break;
          default:
            STATE = DefaultDisplay;
           break;
        }
      break;

      case FuncMenu:
        btnClickFunc=!btnClickFunc;
      break;

      case SetCycleSpeed:
        STATE=MainMenu;
      break;
      
      default:
        STATE = DefaultDisplay;
      break;        
      }
    
    
    }
  debounce = millis();
}

}


//DEFULT DISPLAY
void DefaultMenu(){
  
  int speedy = map(set_speed(),0,1023,0,100);
  int sens = map(set_sensitivity(),0,1023,0,100);
  int fade = map(set_fade(),0,1023,0,100);
  int strobe = map(set_strobe(),0,1023,1,100);
  int bright = map(set_brightness(),0,1023,0,100);
  int val = 0; 
  int lineCounter=10;

    u8g.setPrintPos(0, lineCounter);
    u8g.print(F("MODE"));
    u8g.setPrintPos(40, lineCounter);
    u8g.print(current_function);
    
    u8g.setPrintPos(60, lineCounter);
    u8g.print(F("Out1"));
    
  if(liveMixingStage){
    u8g.drawFrame(58, lineCounter-9, 30, 11);
      
  }
    u8g.setPrintPos(90, lineCounter);
    u8g.print(F("Out2"));
  if (liveMixingFloor){
    u8g.drawFrame(88, lineCounter-9, 30, 11);
  }
    
    lineCounter=lineCounter+12;
  
  
  u8g.setPrintPos(0, lineCounter);
  u8g.print(F("Speed"));
  u8g.setPrintPos(40, lineCounter);
  u8g.print(speedy);
  
  u8g.setPrintPos(60, lineCounter);
  u8g.print(F("SENS"));
  u8g.setPrintPos(105, lineCounter);
  u8g.print(sens);
  
 
  lineCounter=lineCounter+12;
  
  u8g.setPrintPos(0, lineCounter);
  u8g.print(F("Fade"));
  u8g.setPrintPos(40, lineCounter);
  u8g.print(fade);

  u8g.setPrintPos(60, lineCounter);
  u8g.print(F("Strobe"));
  u8g.setPrintPos(105, lineCounter);
  u8g.print(strobe);
  lineCounter=lineCounter+12;

  u8g.setPrintPos(0, lineCounter);
  u8g.print(F("Bright"));
  u8g.setPrintPos(40, lineCounter);
  u8g.print(bright);
  lineCounter=lineCounter+12;

  if (Auto_Mixing){
  u8g.setPrintPos(1, lineCounter);
  u8g.print(F("AutoColour"));    
  u8g.drawFrame(0, lineCounter-9, 60, 11);
  }
  if (Auto_Cycle){
  u8g.setPrintPos(62, lineCounter);
  u8g.print(F("Cycle: "));   
   u8g.print(CycleSpeed);   
    u8g.print(F("min")); 
  u8g.drawFrame(60, lineCounter-9, 68, 11);
  }
   
}

//Main Menu
//**********************************************************************************************************************
void MainMenuDisplay(int RotPosition){
  
   
  int lineCounter=10;  
   
  u8g.setPrintPos(10,lineCounter);
  u8g.print(F("AutoCycle"));
  u8g.setPrintPos(110, lineCounter);
   if(Auto_Cycle){
    u8g.print(F("ON"));
  }else{
    u8g.print(F("OFF"));
  }
  lineCounter = lineCounter+12;
  
  u8g.setPrintPos(10,lineCounter);
  u8g.print(F("Function Mapping"));
  lineCounter = lineCounter+12;
  
  u8g.setPrintPos(10,lineCounter);
  u8g.print(F("back"));

  //DisplayPointer
  u8g.setPrintPos(0,10+RotPosition*12);
  u8g.print(F(">"));

  
   
  
}

//Function Mapping Menu
//***********************************************************************************************************************
void MapFuncMenu(int RotPosition){

  
   
   int lineCounter=10;

   
  if(RotPosition>4){
    lineCounter=10+-12*(RotPosition-4);
  }
 
 
 
  u8g.setPrintPos(10, lineCounter);
  u8g.print(F("Solid Flash (AR)"));      
  for (int i=0; i<numMappings; i++){
    if(mode[i]==0){
  
      u8g.setPrintPos(110,lineCounter);
      u8g.print(i);  
    }
  }
  lineCounter = lineCounter+12;

  u8g.setPrintPos(10, lineCounter);
  u8g.print(F("Bounce (AR)"));  
  for (int i=0; i<numMappings; i++){
    if(mode[i]==1){
       // Serial.println(i);
       // Serial.println(mode[i]);
        u8g.setPrintPos(110,lineCounter);
        u8g.print(i);  
    }
  }
  lineCounter = lineCounter+12;
  
  u8g.setPrintPos(10, lineCounter);
  u8g.print(F("Pulse (AR)"));  
     for (int i=0; i<numMappings; i++){
    if(mode[i]==2){
        u8g.setPrintPos(110,lineCounter);
        u8g.print(i);  
    }
  }
  lineCounter = lineCounter+12;

  u8g.setPrintPos(10, lineCounter);
  u8g.print(F("Wave (AR)"));  
     for (int i=0; i<numMappings; i++){
    if(mode[i]==3){
        u8g.setPrintPos(110,lineCounter);
        u8g.print(i);  
    }
  }
  lineCounter = lineCounter+12;
  
  u8g.setPrintPos(10, lineCounter);
  u8g.print(F("AddOn Solid (AR)"));  
     for (int i=0; i<numMappings; i++){
    if(mode[i]==4){
        u8g.setPrintPos(110,lineCounter);
        u8g.print(i);  
    }
  }
  lineCounter = lineCounter+12;  

  u8g.setPrintPos(10, lineCounter);
  u8g.print(F("Fire"));  
     for (int i=0; i<numMappings; i++){
    if(mode[i]==5){
        u8g.setPrintPos(110,lineCounter);
        u8g.print(i);  
    }
  }
  lineCounter = lineCounter+12;   

 u8g.setPrintPos(10,lineCounter);
 u8g.print(F("back"));

  //Display Pointer
  u8g.setPrintPos(0,10+RotPosition*12);
   if(RotPosition>4){
     u8g.setPrintPos(0,10+(4)*12);
    }
  u8g.print(F(">"));

   
}

//SET CYCLE SPEED
//***************************************************************************************
void SetCycleSpeedMenu(int cycleSpeed){
    
   u8g.setPrintPos(0, 10);
   u8g.print(F("Auto Cycle "));
   u8g.setPrintPos(95, 10);
   u8g.print(cycleSpeed); 
   u8g.setPrintPos(110, 10);
   u8g.print(F("min")); 
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MODE FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void call_mode(int mode) {

    //RUN THE FUNCTION THAT IS GOING TO THE LIGHTS

    switch (mode) {

      //Function mapped to function button 1 (BTNPin3)
      case 0:
         solid_all();
      break;
      
      //Function mapped to function button 2 (BTNPin4)
      case 1:
          
          GRIFF_BOUNCE();
      break;

      //Function mapped to function button 3 (BTNPin5)
      case 2:
         //CALL PULSE
          PULSE(); 
          
      break;

      //Function mapped to function button 4 (BTNPin6)
      case 3:
          
          WAVE( RED_FLOOR,  GREEN_FLOOR,  BLUE_FLOOR);
          
      break;

      //Function mapped to function button 5 (BTNPin7)
      case 4:
           back_to_front();
          if(new_call){ //Only runs once when the button is pressed
            i_b2f=0;
            new_call = false;
          }
      break;

      case 5:
    //  RandomAI();
          Fire2012WithPalette();  
      break;    

      default:
      solid_all();
      
    }  
  }    




//PULSE
  void PULSE() {

    //Read PulseLength from POT
    int scrollSpeed ;  
    int  fade;

    fade = map(set_fade(), 0, 1023, 15, 5);//Set how many Leds are in a pulse (have to use the fade pot because we dont have any more pots available)
    scrollSpeed = map(set_speed(),0,1023,60,0);//This set the speed that the dots get dragged up the bar, 
        
    //If the audio_input functoin returns a high then set the first LED to ON (this starts a new pulse) (starts at 1 instead of 0 to avoid null indexing -1 because im lazy)
      if (fFFT()){
      //  Serial.println("true");
        for(int j=0; j<6;j++){
          if (j<2){
                    ledsOcto.setPixel(j*NUM_LEDS_BAR ,  GREEN_STAGE,RED_STAGE, BLUE_STAGE);
          }else{
                    ledsOcto.setPixel(j*NUM_LEDS_BAR, GREEN_FLOOR, RED_FLOOR, BLUE_FLOOR);
          }
        }      
        pulseCount=0;
          ledsOcto.show();
      }          
  //THIS WILL PULL WHATEVER IS BELOW THE CURRENT LED UP THE BAR at a speed determined by the scroll speed 
  currentTime = millis();
    if (currentTime-pulseStartTime > scrollSpeed) {
      pulseStartTime = millis();
      
      for(int j=0; j<6;j++){
      for (int i = NUM_LEDS_BAR - 1; i > 0; i--){
          int pixelnumber = (j*NUM_LEDS_BAR)+i;      
     
          CRGB pixel = ledsOcto.getPixel(pixelnumber-1);
      
          uint8_t prevRED = pixel.red;
          uint8_t prevGREEN = pixel.green;
          uint8_t prevBLUE  = pixel.blue;

          
          ledsOcto.setPixel(pixelnumber, prevRED,prevGREEN,prevBLUE);   
          //UNTESTED FADE!
          //Set the pulse to fade according to an exponential curve that sets it to 2% brightness by the end of its Pulse length
          //The Dim8 is supposed to take an 8 bit number so it might throw a type exception when it runs

        if(j<2){     
          prevRED = qsub8(prevRED,fade);// *exp(-(pulseCount*4)/pulseLength);
          prevGREEN = qsub8(prevGREEN,fade); //*exp(-(pulseCount*4)/pulseLength);;
          prevBLUE = qsub8(prevBLUE,fade); // *exp(-(pulseCount*4)/pulseLength);
          ledsOcto.setPixel(pixelnumber-1,prevRED, prevGREEN,prevBLUE);   
        }else{
          prevRED = qsub8(prevRED,fade);//*exp(-(pulseCount*4)/pulseLength);
          prevGREEN = qsub8(prevGREEN,fade);//*exp(-(pulseCount*4)/pulseLength);;
          prevBLUE = qsub8(prevBLUE,fade);//*exp(-(pulseCount*4)/pulseLength);
          ledsOcto.setPixel(pixelnumber-1,prevRED, prevGREEN,prevBLUE);   
        }
      }
      } 
      //Show updates and increase pulse count
      ledsOcto.show();
      pulseCount++;
    }
      

  }


//****************************************************************************************************************
//WAVE FROM THE GRIFF
// CURRENTLY BROKEN!
//********************************************************************************************************//
  CRGB Led[140]; 
  void WAVE(double RED, double GREEN, double BLUE){
  //  Serial.println("in wave");

    fFFT(1.001);
    uint8_t ex = map(set_fade(),0,1023,40,1);  
    uint8_t sens = map(set_sensitivity(),0,1023,20,100);  
    uint8_t speedy = map(set_speed(),0,1023, 100, 1); 
    
    if (waveStart + speedy < millis()){
          waveStart = millis(); 
          float gain = ((fftResult[1]+ fftResult[2] + fftResult[3] + fftResult[4] + fftResult[5] + fftResult[6] + fftResult[7]+ fftResult[8]))/sens;
           CRGB color = CRGB(scale8((RED*gain),255), scale8((GREEN*gain),255),scale8((BLUE*gain),255));
           for(int j = 0; j<6;j++){
              Led[0] = color;
          
           for (int i=139;i>0;i--){
            Led[i] = Led[i-1];
          } 
           for (int i=NUM_LEDS_BAR;i>=0;i--){
              int pixelnumber = (j*NUM_LEDS_BAR)+i;      
              uint8_t waveRED = qsub8((Led[i].red),0);
              uint8_t waveGREEN = qsub8((Led[i].green),0);
              uint8_t waveBLUE = qsub8((Led[i].blue),0);
              ledsOcto.setPixel(pixelnumber, waveGREEN, waveRED, waveBLUE);   
              }
             ledsOcto.show();
              }
    }


    }


//BOUNCE FROM THE GRIFF
//********************************************************************************************************//
  void GRIFF_BOUNCE(){
        uint8_t bounceRED ;
        uint8_t bounceGREEN ;
        uint8_t bounceBLUE  ;

  fFFT(1.001);
    int sensitivity = map(set_sensitivity(),0,1023,1,8);
    uint8_t ex = map(set_fade(),0,1023,40,5);  
    for(int j = 0; j<6;j++){
      if (j<2){        
        for (int i=0;i<NUM_LEDS_BAR;i++){
          int pixelnumber = (j*NUM_LEDS_BAR)+i;      
          if ((i)*sensitivity <((fftResult[1]+ fftResult[2] + fftResult[3] + fftResult[4] + fftResult[5] + fftResult[6] + fftResult[7]+ fftResult[8]) ) ){   
            
            ledsOcto.setPixel(pixelnumber, GREEN_STAGE, RED_STAGE, BLUE_STAGE);
          }
          else {  
            CRGB currentColor =  ledsOcto.getPixel(pixelnumber);   
             if (currentColor>0){
               bounceRED = qsub8(currentColor.green,ex);
               bounceGREEN = qsub8(currentColor.red,ex);
               bounceBLUE  = qsub8(currentColor.blue,ex);
            }else{
               bounceRED = 0;
               bounceGREEN = 0;
               bounceBLUE  = 0;
            }            
            ledsOcto.setPixel(pixelnumber, bounceGREEN,bounceRED,bounceBLUE );
          }
        }
      }else{        
        for (int i=0;i<NUM_LEDS_BAR;i++){
          int pixelnumber = (j*NUM_LEDS_BAR)+i;   
          if ((i)*sensitivity <((fftResult[1]+ fftResult[2] + fftResult[3] + fftResult[4])) ){    
            ledsOcto.setPixel(pixelnumber, GREEN_FLOOR, RED_FLOOR, BLUE_FLOOR);
          }else{ 
            CRGB currentColor =  ledsOcto.getPixel(pixelnumber);   
             if (currentColor>0){
               bounceRED = qsub8(currentColor.green,ex);
               bounceGREEN = qsub8(currentColor.red,ex);
               bounceBLUE  = qsub8(currentColor.blue,ex);
            }else{
               bounceRED = 0;
               bounceGREEN = 0;
               bounceBLUE  = 0;
            }            
            ledsOcto.setPixel(pixelnumber, bounceGREEN,bounceRED,bounceBLUE );
         }
        }
       }
}
    ledsOcto.show();
  }

//Whooosh BABYYY
//********************************************************************************************************//
//WOULD LIKE TO EDIT THIS MODE (remember to make a copy and edit the copy so we still have the original
// - Turning them off in reverse instead of all at once (kinda like a bounce)
// - Fading them off as it progresses forwad (leaving a trail) 
//  void back_to_front(){
//    uint8_t fade = map(analogRead(potPin[2]),0,1023,0,25);
//   
//    currentTime = millis();
//    bool flag;
//    if (Auto_Mixing){
//
//          flag = true;
//    }else{
//        if ((currentTime-b2fStartTime)>map(set_speed(),0,1023,300,0)){  
//          flag = true;
//        }      else{
//          flag= false;
//        }    
//    }
//
//    
//    if (flag){    //(currentTime-startTime)>map(set_speed(),0,1023,300,0)
//    
//      if (fFFT() == true){  //fFFT()
//        //If i is less than two use this to get correct stage colour
//        fadeCounter=fade;
//        if (i_b2f<2){
//                
//          //Turn the Current Bar on
//            for ( int i = 0; i<NUM_LEDS_BAR; i++){
//              int pixelNumber = (i_b2f*NUM_LEDS_BAR)+i;
//               ledsOcto.setPixel(pixelNumber, GREEN_STAGE, RED_STAGE, BLUE_STAGE);
//            }
//          
//            ledsOcto.show();
//                               
//        }else {
//            if(i_b2f<6){ //Use the floor colour values
//            //Turn Current bar on
//            for ( int i = 0; i<NUM_LEDS_BAR; i++){
//              int pixelNumber = (i_b2f*NUM_LEDS_BAR)+i;
//               ledsOcto.setPixel(pixelNumber, GREEN_FLOOR, RED_FLOOR, BLUE_FLOOR);  
//            }
//          
//            ledsOcto.show();
//            
//        }else{
//          //If i reaches 7 start again at 0
//          // Turn them all off
//           fadeCounter = qsub8(fadeCounter,1);
//           
//           for (int i = 0; i < NUM_LEDS_BAR*6; i++) {  // RED_STAGE  GREEN_FLOOR
//            if (i<NUM_LEDS_BAR*2) {
//               ledsOcto.setPixel(i,0,0,0); //ledsOcto.setPixel(i, qmul8(fadeCounter, (GREEN_STAGE/fade)), qmul8(fadeCounter, (RED_STAGE/fade)),qmul8(fadeCounter, (BLUE_STAGE/fade)));
//            }else{
//             ledsOcto.setPixel(i,0,0,0); // ledsOcto.setPixel(i, qmul8(fadeCounter, (GREEN_FLOOR/fade)), qmul8(fadeCounter, (RED_FLOOR/fade)),qmul8(fadeCounter, (BLUE_FLOOR/fade)));
//            }
//           } 
//          ledsOcto.show();
//          i_b2f=-1; // Setting i_b2f to -1 because we always inc it at the end of this function meaning it ends on 0
//          
//        }
//      
//      }}
//      else{ // If the audio is not triggering them then turn everything off
//        i_b2f=-1;// Setting i_b2f to -1 because we always inc it at the end of this function meaning it ends on 
//    
//        //Turn them all off.
//           fadeCounter = qsub8(fadeCounter,1);
//           
//           for (int i = 0; i < NUM_LEDS_BAR*6; i++) {  // RED_STAGE  GREEN_FLOOR
//            if (i<NUM_LEDS_BAR*2) {
//              ledsOcto.setPixel(i,0,0,0);// ledsOcto.setPixel(i, qmul8(fadeCounter, (GREEN_STAGE/fade)), qmul8(fadeCounter, (RED_STAGE/fade)),qmul8(fadeCounter, (BLUE_STAGE/fade)));
//            }else{
//              ledsOcto.setPixel(i,0,0,0);// ledsOcto.setPixel(i, qmul8(fadeCounter, (GREEN_FLOOR/fade)), qmul8(fadeCounter, (RED_FLOOR/fade)),qmul8(fadeCounter, (BLUE_FLOOR/fade)));
//            }
//           }   
//          ledsOcto.show();
//      }
//      
//      b2fStartTime=currentTime;
//      i_b2f++;
//    }
//  }

   void back_to_front(){
    uint8_t fade = map(analogRead(potPin[2]),0,1023,30,20);
    uint8_t ex = map(set_fade(),0,1023,50,5);  
    currentTime = millis();
   
    bool flag;

        if ((currentTime-b2fStartTime)>map(set_speed(),0,1023,200,0)){  
          flag = true;
        }      else{
          flag= false;
        }    
    

    
      //(currentTime-startTime)>map(set_speed(),0,1023,300,0)
    
      if (fFFT() == true){  //fFFT()
        //If i is less than two use this to get correct stage colour
        if (flag){
        fadeCounter=fade;
        if (i_b2f<2){
          //Turn the Current Bar on
            for ( int i = 0; i<NUM_LEDS_BAR; i++){
              int pixelNumber = (i_b2f*NUM_LEDS_BAR)+i;
               ledsOcto.setPixel(pixelNumber, GREEN_STAGE, RED_STAGE, BLUE_STAGE);
            }                            
         }else{
            //Use the floor colour values
            //Turn Current bar on
            for ( int i = 0; i<NUM_LEDS_BAR; i++){
              int pixelNumber = (i_b2f*NUM_LEDS_BAR)+i;
               ledsOcto.setPixel(pixelNumber, GREEN_FLOOR, RED_FLOOR, BLUE_FLOOR);  
            }           
         }    
         
              if (i_b2f < 5) {
                i_b2f++;
              }else{
                i_b2f = 0;}
              b2fStartTime=currentTime;            
        }}
      else{ // If the audio is not triggering them then turn everything off
      // Setting i_b2f to -1 because we always inc it at the end of this function meaning it ends on 
     
        //Turn them all off.
           
           uint8_t bounceRED ;
           uint8_t bounceGREEN ;
           uint8_t bounceBLUE  ;
           for (int i = 0; i < NUM_LEDS_BAR*6; i++) {  // RED_STAGE  GREEN_FLOOR

            CRGB currentColor =  ledsOcto.getPixel(i);   
             if (currentColor>0){
               bounceRED = qsub8(currentColor.green,ex);
               bounceGREEN = qsub8(currentColor.red,ex);
               bounceBLUE  = qsub8(currentColor.blue,ex);
            }else{
               bounceRED = 0;
               bounceGREEN = 0;
               bounceBLUE  = 0;
            }            
            ledsOcto.setPixel(i, bounceGREEN,bounceRED,bounceBLUE );
           }   
          ledsOcto.show();

           


      }
      
    ledsOcto.show();

      
    
    //Turn off previous bar
     uint8_t lastBar;
    if (i_b2f == 0) {
      lastBar = 5;
    } else {
      lastBar = i_b2f -1; 
    }
//    fadeCounter = qsub8(fadeCounter,1);
//    for ( int i = lastBar*NUM_LEDS_BAR ; i<lastBar*NUM_LEDS_BAR+NUM_LEDS_BAR; i++){
//      if (lastBar<NUM_LEDS_BAR*2){
//        ledsOcto.setPixel(i, qmul8(fadeCounter, (GREEN_STAGE/fade)), qmul8(fadeCounter, (RED_STAGE/fade)),qmul8(fadeCounter, (BLUE_STAGE/fade)));
//      }else{
//        ledsOcto.setPixel(i, qmul8(fadeCounter, (GREEN_FLOOR/fade)), qmul8(fadeCounter, (RED_FLOOR/fade)),qmul8(fadeCounter, (BLUE_FLOOR/fade)));
//      }
//    }      
  }

 

//FIRE MODE BABY
//***************************************************************************************************************

  void Fire2012WithPalette()
  {

      FRAMES_PER_SECOND =map(set_speed(),0,1023,40,5); //Suggested range 35-10 Can tweak this range (UNTESTED)
      COOLING = map(set_fade(),0,1023,120,20);          //SUggested range 20-100
      SPARKING =map(set_sensitivity(),0,1023,40,220);  // Suggested range 50 - 200                                 


    random16_add_entropy( random());
    static uint8_t hue;
  
    float bright = (map(set_brightness(),0,1023,0,255));
    
    //hue = map(analogRead(sliderPin[0]),0,1023, 0, 255);
    CRGB colour = CRGB(RED_FLOOR,GREEN_FLOOR,BLUE_FLOOR);
    
    CRGB veryDarkColour = CRGB(RED_FLOOR*0.01,GREEN_FLOOR*0.01,BLUE_FLOOR*0.01);
    CRGB darkcolor  = CRGB(RED_FLOOR*0.5,GREEN_FLOOR*0.5,BLUE_FLOOR*0.5);//CHSV(hue,255,192); // pure hue, three-quarters brightness
    CRGB lightcolor = CRGB(RED_FLOOR,GREEN_FLOOR,BLUE_FLOOR);//HSV(hue,190,255); // half 'whitened', full brightness
    CRGB veryLightColour = CHSV(Auto_Hue,100,bright);
    gPal = CRGBPalette16( veryDarkColour,   darkcolor, lightcolor, veryLightColour);

  
    if ((millis()-FRAMES_PER_SECOND)>fireStartTime){
          // Array of temperature readings at each simulation cell
      for (int barCount = 0; barCount<6;barCount++){ //Run the effect 6 times (one for each bar) 
          
          static uint8_t heat[NUM_LEDS_BAR];
          // Step 1.  Cool down every cell a little
          for( int i = 0; i < NUM_LEDS_BAR; i++) {
            heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS_BAR) + 2));
          }
          // Step 2.  Heat from each cell drifts 'up' and diffuses a little
          for( int k= NUM_LEDS_BAR - 1; k >= 2; k--) {
            heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
          }
          // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
          if( random8() < SPARKING ) {
            int y = random8(7);
            heat[y] = qadd8( heat[y], random8(160,255) );
          }
          // Step 4.  Map from heat cells to LED colors
          for( int j = 0; j < NUM_LEDS_BAR; j++) {
            // Scale the heat value from 0-255 down to 0-240
            // for best results with color palettes.
            uint8_t colorindex = scale8( heat[j], 240);
            CRGB color = ColorFromPalette( gPal, colorindex);
            int pixelnumber = (barCount*NUM_LEDS_BAR)+j;
          
                    uint8_t fireRED = color.red;
                    uint8_t fireGREEN = color.green;
                    uint8_t fireBLUE = color.blue;
                 
                 
                       ledsOcto.setPixel(pixelnumber, fireGREEN, fireRED, fireBLUE);
             
                     
          }
          ledsOcto.show();   
      }
      fireStartTime=millis();
    }
  }


//SOLID FLASH AUDIO REACTIVE
//********************************************************************************************************//
  void solid_all(){
    uint8_t fade = map(analogRead(potPin[2]),0,1023,5,40);
    uint8_t sens = map(set_sensitivity(),0,1023,0,100);
    
      if (peak1.available()){
       peakValue = peak1.read()*100;
  }
//Thiss shit looks ass  
    
    if (peakValue>=sens){
      fadeCounter=fade;

      for (int i = 0; i < NUM_LEDS_BAR*6; i++) {
        if (i<NUM_LEDS_BAR*2) {
           ledsOcto.setPixel(i, GREEN_STAGE, RED_STAGE, BLUE_STAGE);
        }else{
           ledsOcto.setPixel(i, GREEN_FLOOR, RED_FLOOR, BLUE_FLOOR);
        }
       }  
    }
    else{
      
      fadeCounter = qsub8(fadeCounter,1);
       
       for (int i = 0; i < NUM_LEDS_BAR*6; i++) {  // RED_STAGE  GREEN_FLOOR
        if (i<NUM_LEDS_BAR*2) {
           ledsOcto.setPixel(i, qmul8(fadeCounter, (GREEN_STAGE/fade)), qmul8(fadeCounter, (RED_STAGE/fade)),qmul8(fadeCounter, (BLUE_STAGE/fade)));
        }else{
           ledsOcto.setPixel(i, qmul8(fadeCounter, (GREEN_FLOOR/fade)), qmul8(fadeCounter, (RED_FLOOR/fade)),qmul8(fadeCounter, (BLUE_FLOOR/fade)));
        }
       }          
    }
  
    ledsOcto.show();   
  }

//AI Random pattern






//STROBE BABY
//********************************************************************************************************//
  void strobe(){

        if ((digitalRead(togglePin[0]))==LOW){
          int freq = map(analogRead(potPin[3]),0,1023,200,70);
          for (int i = 0; i < NUM_LEDS_BAR*6; i++) {
                 ledsOcto.setPixel(i, 255,255, 255);
             }   
            ledsOcto.show(); 
          //DELAY FOR 1/10th of the time they are off for. (NB STILL NEED TO PLAY WITH THIS AND TUNE)
          delay((4*freq)/10) ;  
          //CLEAR ALL THE DATA IN THE LEDS
          for (int i = 0; i < NUM_LEDS_BAR*6; i++) {
              if (i<NUM_LEDS_BAR*2) {
                 ledsOcto.setPixel(i, 0, 0, 0);
              }else{
                 ledsOcto.setPixel(i, 0, 0, 0);
              }
             }   
        //Show makes them all show but with no data (so they are all go off)
            ledsOcto.show(); 
          //DELAY FOR 9 times longer than they are on for (STILL NEED TO TUNE)
          delay((freq*9)/10);
        }
        if ((digitalRead(togglePin[0]))==HIGH){
          int freq = map(analogRead(potPin[3]),0,1023,200,5);
          
             for (int i = 0; i < NUM_LEDS_BAR*6; i++) {
              if (i<NUM_LEDS_BAR*2) {
                 ledsOcto.setPixel(i, GREEN_STAGE, RED_STAGE, BLUE_STAGE);
              }else{
                 ledsOcto.setPixel(i, GREEN_FLOOR, RED_FLOOR, BLUE_FLOOR);
              }
             }  
           ledsOcto.show();   
          //DELAY FOR 1/10th of the time they are off for. (NB STILL NEED TO PLAY WITH THIS AND TUNE)
          delay((4*freq)/10) ;  
          //CLEAR ALL THE DATA IN THE LEDS
          for (int i = 0; i < NUM_LEDS_BAR*7; i++) {
              if (i<NUM_LEDS_BAR*2) {
                 ledsOcto.setPixel(i, 0, 0, 0);
              }else{
                 ledsOcto.setPixel(i, 0, 0, 0);
              }
             }   
        //Show makes them all show but with no data (so they are all go off)
           ledsOcto.show();   
          //DELAY FOR 9 times longer than they are on for (STILL NEED TO TUNE)
          delay((freq*9)/10);
          }
    }

  /*
  {
    // Only Run if the Strobe button is pushed.
          int freq = map(analogRead(potPin[3]),0,1023,1000,10);
          Serial.println(freq);
          int OncurrentTime = millis();

          //TURN THEM ON
          // After 3/4 Frequency mills have gone by turn them on again
          if ((OncurrentTime-startTime)>((3*freq)/4)){
            
            //If colour strobe is on then fill bars with Floor Colours else Fill them with white
            if((digitalRead(togglePin[0]))==HIGH){
              for (int i =0; i<7;i++){
                fill_solid(leds[i], NUM_LEDS_BAR, CRGB(RED_FLOOR,GREEN_FLOOR,BLUE_FLOOR));
              }             
            }else{
            for (int i =0; i<7;i++){
                fill_solid(leds[i], NUM_LEDS_BAR, CRGB(255,255,255));
            }}
            FastLED.show();
          }

        int OffcurrentTime = millis();

        // TURN THEM OFF
        //DELAY FOR 1/4th of the time they are off for. (NB STILL NEED TO PLAY WITH THIS AND TUNE)
        if ((OffcurrentTime-startTime)>freq){
          // Clear all the  leds and then show them so they turn off.
          FastLED.clear(); 
          FastLED.show(); 
          startTime=millis();
          Serial.println(startTime);
        }       
  }
  */
