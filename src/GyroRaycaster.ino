/*
 * Project: GyroRaycaster
 * Description: Gamelike raycaster where viewer can be moved and rotated by gyroscope sensor 
 *   When the viewer has reached the exit to the outer world the games ends
 * Hardward: Arduino Uno/Nano with gyroscope sensor MPU6050 and SSD1306 OLED 128x64 pixel display
 * License: MIT License
 * Copyright (c) 2022 codingABI
 * 
 * created by codingABI https://github.com/codingABI/GyroRaycaster
 * 
 * History:
 * 14.05.2022, Initial version
 * 15.05.2022, Display elapsed time at finish
 * 15.05.2022, Fix rare glitches when angle close to 90/270 or 0/180 degrees
 * 15.05.2022, Change precalculated sins from values [0;64] to [0;128] to more precise
 * 15.05.2022, Change inital text from "Init gyro..." to "Find the exit..."
 * 16.05.2022, Fix wrong +1 when calculating crossingY
 * 21.05.2022, Remove unneeded mpu.PrintActiveOffsets();
 */
 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // dont forget to uncomment #define SSD1306_NO_SPLASH in Adafruit_SSD1306.h to free program storage
#include <I2Cdev.h>

#include <MPU6050_6Axis_MotionApps20.h> // older, but smaller (~1k) binary than <MPU6050_6Axis_MotionApps612.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define VIEWPORT_HEIGHT 56 // Height of the raycaster scene

// SSD1306 I2C 
#define OLED_RESET -1 // no reset pin
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// MPU I2C
MPU6050 mpu;

#define INTERRUPT_PIN 2
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

#define MAPWIDTH 8  // width of map
#define MAPHEIGHT 16  // height of map
#define STRIPEHEIGHT 8 // height of wall stripe
#define STRIPEHEIGHTSCALER 90
#define SIDEUNKNOWN 0
#define SIDELEFTRIGHT 1
#define SIDEUPDOWN 2  
#define HUGEBIGNUMBER 1000000
#define GRIDSIZE 800 // dimension of a wallside 
 // final exit position
#define EXITGRIDX 7
#define EXITGRIDY 14
// Start position and angle
#define STARTX 1200
#define STARTY 2000
#define STARTANGLE 0 

// raycaster map
const PROGMEM byte g_map[]= {
 0b11111111,
 0b10100001,
 0b10000001,
 0b10001001,
 0b10001001,
 0b10000101,
 0b10000001,
 0b11111011,  
 0b10001011,
 0b00001001,
 0b11101011,
 0b10100001,
 0b10100001,
 0b10000111,
 0b10000000,
 0b11111111,
};

// precalculated sin to improve performance (degree 0-90 with values [0;128])
byte g_sin128[91] {
  0,2,4,7,9,11,13,16,18,20,22,24,27,29,31,33,35,37,40,42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,73,75,77,79,81,82,84,86,87,89,91,92,94,95,97,98,99,101,102,104,105,106,107,109,110,111,112,113,114,115,116,117,118,119,119,120,121,122,122,123,124,124,125,125,126,126,126,127,127,127,128,128,128,128,128,128
};

 // initial viewer settings
int g_viewerX = STARTX;
int g_viewerY = STARTY;
int g_viewerAngle = STARTANGLE;
unsigned long g_startMS;

// Interrupthandler for mpu
void dmpDataReady() {
  mpuInterrupt = true;
}

// function to get precalculated sins (return values [-128;128])
int sin128(int degree) {
  degree = degree % 360;
  if (degree < 0) degree+=360;
  if (degree < 90) return  g_sin128[degree];
  if (degree < 180) return g_sin128[180-degree];
  if (degree < 270) return - (int) g_sin128[degree-180];
  return -(int) g_sin128[360-degree];
}

// check if box in grid is filled with wall
bool isBoxFilled(long x, long y) {
  
  if ((x<0) || (x>MAPWIDTH*GRIDSIZE-1)) return false; // out of map
  if ((y<0) || (y>MAPHEIGHT*GRIDSIZE-1)) return false; // out of map
  
  return ((pgm_read_byte(&(g_map[y/GRIDSIZE]))>>(MAPWIDTH-x/GRIDSIZE-1)) & 1);
}

// check if position is within map
bool isInMap(long x, long y) {
  return !((x < 0) || (x > MAPWIDTH*GRIDSIZE-1) || (y < 0) || (y > MAPHEIGHT*GRIDSIZE-1));
}

// dynamic texture (+ vertical, - horizontal)
bool texture(byte x, byte y, byte side) {
  if ((x > 1) && (x < 6) && (y > 2) && (y < 5)) return false;
  if ((side == SIDELEFTRIGHT) && (x > 2) && (x < 5) && (y > 1) && (y < 6)) return false;
  return true;
}

// Draw raycasted scene (inspired on raycast ideas from https://github.com/3DSage/OpenGL-Raycaster_v1 and https://github.com/3DSage/OpenGL-Raycaster_v2)
void drawScene() {
  // in general we prevent floats to improve speed. Only floats for some textures variables
  int angle;
  long finalCrossingX,finalCrossingY;
  long crossingX, crossingY;
  long deltaX, deltaY;
  long distanceX,distanceY;
  long minDistance;
  long height;
  int cachedCos128, cachedSin128; 
  long cachedTan128;
  byte side;
  byte lastSide = SIDEUNKNOWN;
  bool finalCrossingFound;
  // for textures
  bool toDraw;
  float textureDeltaY;
  float offsetTextureY;
  int beginOfStripe;
  float textureY;
  byte textureX;

  // Start angle
  if (g_viewerAngle > 32) angle = g_viewerAngle - 32; else angle = 328 + g_viewerAngle;

  // for every second pixel from the screen width
  for (byte viewPortX=0;viewPortX<SCREEN_WIDTH;viewPortX+=2) {
    angle = (angle + 1)%360; // increase raycasted angle
    
    side = SIDEUNKNOWN;
    finalCrossingFound = false;
    crossingX = g_viewerX;
    crossingY = g_viewerY;
    cachedSin128 = sin128(angle);
    cachedCos128 = sin128(90-angle);
    if (cachedCos128 != 0) cachedTan128 = ((((long)cachedSin128)<<7)/cachedCos128); else cachedTan128 = HUGEBIGNUMBER; // tan(a) = sin(a)/cos(a)
    distanceX = HUGEBIGNUMBER;
    distanceY = HUGEBIGNUMBER;

    do { // left or right (crossing vertical wall faces)
      if ((crossingX == g_viewerX) && (crossingY == g_viewerY)) { // first step
        if (cachedCos128 > 0){ // right
          crossingX=(g_viewerX/GRIDSIZE)*GRIDSIZE+GRIDSIZE; // grid on right
          crossingY=g_viewerY - (((g_viewerX - crossingX)*cachedTan128)>>7);
          // delta for the next steps
          deltaX = GRIDSIZE;
          deltaY = (deltaX*cachedTan128)>>7;
        } else if (cachedCos128 < 0){ // left
          crossingX=(g_viewerX/GRIDSIZE)*GRIDSIZE-1; // grid on left
          crossingY=g_viewerY - (((g_viewerX - crossingX)*cachedTan128)>>7);
          deltaX = -GRIDSIZE;
          deltaY = (deltaX*cachedTan128)>>7;
        } else {
          // too close to up or down
          crossingX=g_viewerX; 
          crossingY=g_viewerY; 
          finalCrossingFound = true;
        }
      } else { // following steps       
        if (isInMap(crossingX,crossingY) && isBoxFilled(crossingX,crossingY)) { // Wall found
          distanceX=(cachedCos128*(crossingX-g_viewerX)+cachedSin128*(crossingY-g_viewerY))>>7; // calculate distance between points by using transformation of Pythagorean trigonometric identity (faster then sqrt(dx^2+dy^2)).
          finalCrossingFound = true;
        } else {
          crossingX+=deltaX;
          crossingY+=deltaY;
          if (!isInMap(crossingX,crossingY)) { // out of map
            finalCrossingFound = true; 
          } 
        }
      }
    } while (!finalCrossingFound);

    finalCrossingX = crossingX;
    finalCrossingY = crossingY;
    crossingX = g_viewerX;
    crossingY = g_viewerY;
    finalCrossingFound = false;

    if (cachedTan128 == 0) { cachedTan128 = 1; }; // Prevent DIV0
    
    do { // up or down (crossing horizontal wall faces)
      if ((crossingX == g_viewerX) && (crossingY == g_viewerY)) { // first step
        if (cachedSin128 < 0){ // up
          crossingY=(g_viewerY/GRIDSIZE)*GRIDSIZE-1; // upper grid
          crossingX=g_viewerX - ((g_viewerY - crossingY)<<7)/cachedTan128;
          // delta for the next steps
          deltaY = -GRIDSIZE;
          deltaX = (deltaY<<7)/cachedTan128;
        } else if (cachedSin128 > 0){ // down
          crossingY=(g_viewerY/GRIDSIZE)*GRIDSIZE+GRIDSIZE;
          crossingX=g_viewerX - ((g_viewerY - crossingY)<<7)/cachedTan128;
          deltaY = GRIDSIZE;
          deltaX = (deltaY<<7)/cachedTan128;
        } else {
          // too close to left or right
          crossingX=g_viewerX; 
          crossingY=g_viewerY; 
          finalCrossingFound = true;
        }
       } else { // following steps       
        if (isInMap(crossingX,crossingY) && isBoxFilled(crossingX,crossingY)) { // Wall found
          distanceY=(cachedCos128*(crossingX-g_viewerX)+cachedSin128*(crossingY-g_viewerY))>>7; // calculate distance between points by using transformation of Pythagorean trigonometric identity (faster then sqrt(dx^2+dy^2)).
          finalCrossingFound = true;
        } else {
          crossingX+=deltaX;
          crossingY+=deltaY;
          if (!isInMap(crossingX,crossingY)) { // out of map
            finalCrossingFound = true; 
          } 
        }
      }
    } while (!finalCrossingFound);

    if ((distanceX != HUGEBIGNUMBER) || (distanceY != HUGEBIGNUMBER)) { // crossing in map found
      if( distanceY < distanceX -10){ 
        finalCrossingX=crossingX; 
        finalCrossingY=crossingY; 
        side = SIDEUPDOWN;
        minDistance = distanceY;
      } else if (distanceX < distanceY -10) { 
        minDistance = distanceX;
        side = SIDELEFTRIGHT;
      } else { 
        // can not determine if horizontal or vertical => use last used side
        minDistance = distanceX;
        side = lastSide;
      }
      
      minDistance= ((long)minDistance* (int) sin128(90-(g_viewerAngle-angle))) >> 7; //fisheye reduce

      height = (long) STRIPEHEIGHTSCALER*VIEWPORT_HEIGHT*STRIPEHEIGHT/minDistance; // Current stripheight

      // texture pixel height is proportional to max/real wall stripe height
      textureDeltaY = (float)STRIPEHEIGHT/height;
      offsetTextureY = 0;
      if(height>VIEWPORT_HEIGHT) { // Bigger than viewer port
        offsetTextureY=(height-VIEWPORT_HEIGHT)/2.0f; // half of "oversize"
        height=VIEWPORT_HEIGHT; // reduce wall stripe size to viewport height
      }
      beginOfStripe = VIEWPORT_HEIGHT/2 - height/2; // horizontal start of stripe

      textureY = offsetTextureY*textureDeltaY; // line in texture and take care of "oversize" to avoid glitches
      textureX; // column in texture

      if (side == SIDELEFTRIGHT) { // if horizontal wall face => calc texture column from crossing y value MOD wall width and fix column direction dependend on left/right
        textureX =((finalCrossingY)%GRIDSIZE)/(GRIDSIZE/STRIPEHEIGHT);
        if(angle<90 || angle>270) { textureX=STRIPEHEIGHT-1-textureX; };
      } 
      if (side == SIDEUPDOWN) { // if vertical wall face => calc texture column from crossing x value MOD wall height and fix column direction dependend on up/down
        textureX=((finalCrossingX)%GRIDSIZE)/(GRIDSIZE/STRIPEHEIGHT);
        if(angle>180) { textureX=STRIPEHEIGHT-1-textureX;}
      }

      for (byte k=0;k<height;k++) {
        toDraw = texture(textureX,textureY,side); // should pixel be drawn?
  
        if (toDraw) { // pixel to draw
          if (side == SIDEUPDOWN) { // simple dithering for up- and downsides
            if ((k+(height/2)%2+viewPortX)%2) {
              display.drawPixel(viewPortX,k + beginOfStripe,SSD1306_WHITE);
            } else {
              display.drawPixel(viewPortX + 1,k + beginOfStripe,SSD1306_WHITE);            
            }
          } else { // Draw two pixel, because viewPortX has a stepsize of 2
            display.drawPixel(viewPortX,k + beginOfStripe,SSD1306_WHITE);
            display.drawPixel(viewPortX + 1,k + beginOfStripe,SSD1306_WHITE);
          }
        }
        textureY += textureDeltaY; // Next texture line
      }      
      lastSide = side; // remember side for next stripes where side can not be determined (distanceX == distanceY)
    } 
  }  
}

// blink internal led
void blink(int time) {
  digitalWrite(LED_BUILTIN,HIGH);
  delay(time);
  digitalWrite(LED_BUILTIN,LOW);
  delay(time);
}

void setup(void) {
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

  pinMode(LED_BUILTIN,OUTPUT);
  
  //Start I2C
  Wire.begin();

  // MPU6050 init
  mpu.initialize();

  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  if (!mpu.testConnection()) {
    // MPU6050 connection failed
    blink(1000);
    while (true);
  }

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

   // OLED init
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Default Address is 0x3D for 128x64, but my OLED uses 0x3C 
    // SSD1306 allocation failed
    blink(1000);
    blink(1000);
    blink(1000);
    while (true);
  }

  // Font settings
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.println(F("Find the"));
  display.println(F("exit..."));
  display.display();
  display.setTextSize(1);

  // Calibration based on IMU_ZERO
  mpu.setXGyroOffset(1907);
  mpu.setYGyroOffset(130);
  mpu.setZGyroOffset(-1);
  mpu.setXAccelOffset(-647);
  mpu.setYAccelOffset(-3985);  
  mpu.setZAccelOffset(-4111);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    // DMP Initialization failed
    blink(1000);
    blink(1000);
    if (devStatus==1) blink(500);
    if (devStatus==2) { blink(500);blink(500); }
    while (true);
  }

  g_startMS = millis();
}

void loop(void) {
  char strData[24];
  unsigned long startMS, endMS;
  static unsigned int fps = 0;
  static int pitch = 0;
  static int yaw = 0;
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  uint8_t fifoBuffer[64]; // FIFO storage buffer
  uint8_t rc;
  int newX, newY;
  signed char direction = 0;
  
  // record start of frame
  startMS = millis();

  // get pitch, roll and yaw from MPU6050
  if (dmpReady) {
    // read a packet from FIFO
    rc = mpu.dmpGetCurrentFIFOPacket(fifoBuffer); 

    if (rc) { // Get the Latest packet 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      pitch = -ypr[1] * 180/M_PI;
      yaw = -ypr[0] * 180/M_PI;
    }
  }
  
  // rotate and move viewer
  g_viewerAngle = (-yaw)%360;
  if (g_viewerAngle<0) g_viewerAngle+=360;

  if (pitch > 10) direction = -1;
  if (pitch < -10) direction = 1;
  if (pitch != 0) {
    newX = g_viewerX + ((sin128(90-g_viewerAngle) * direction) >> 1); // cos(a) = sin(90-a)
    newY = g_viewerY + ((sin128(g_viewerAngle) * direction) >> 1);
    if (isInMap(newX,newY) && !isBoxFilled(newX,newY)) {
      g_viewerX = newX;
      g_viewerY = newY;
    }
    if ((g_viewerX/GRIDSIZE == EXITGRIDX) && (g_viewerY/GRIDSIZE == EXITGRIDY)) { // Exit reached
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(50,20);
      display.print(F("Fin"));
      display.setTextSize(1);
      display.setCursor(0,56);
      display.print(F("Solved in "));
      display.print((millis()-g_startMS)/1000);
      display.print(F(" seconds"));
      display.display();
      delay(10000);
      // Jump to start
      g_viewerX = STARTX;
      g_viewerY = STARTY;
      g_startMS = millis();
    }
  }

  // clear display buffer
  display.clearDisplay();

  // draw rendered scene
  drawScene();

  // draw viewer data
  snprintf(strData,24,"X%5d Y%5d%4d%c%2df",g_viewerX,g_viewerY,g_viewerAngle,(char)247,fps);
  display.setCursor(0,57);
  display.print(strData);

  // show display buffer on screen
  display.display();

  // calculate frames per second
  endMS = millis();
  if (endMS - startMS > 0) fps = 1000/(endMS - startMS);
}
