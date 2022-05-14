/*
 * Project: GyroRaycaster
 * Description: Raycaster where viewer can be moved and rotated by gyro sensor (Arduino Uno/Nano with gyro sensor MPU6050 and SSD1306 OLED 128x64 pixel display).
 * License: MIT License
 * Copyright (c) 2022 codingABI
 * 
 * created by codingABI https://github.com/codingABI/GyroRaycaster
 * 
 * History:
 * 14.05.2022, Initial version
 */
 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> / dont forget to uncomment #define SSD1306_NO_SPLASH in Adafruit_SSD1306.h to free program storage
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
#define MAPHEIGHTPIX 12800
#define STRIPEHEIGHT 8 // height of wall stripe
#define SIDEUNKNOWN 0
#define SIDELEFTRIGHT 1
#define SIDEUPDOWN 2  
#define HUGEBIGNUMBER 1000000
#define GRIDSIZE 800 // dimension of a wallside 
#define FINISHGRIDX 7
#define FINISHGRIDY 14
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
 0b00101001,
 0b10101011,
 0b10100001,
 0b10000001,
 0b10000111,
 0b10000000,
 0b11111111,
};

// precalculated sin to improve performance 
signed char g_sin64[91] {
  0,1,2,3,4,6,7,8,9,10,11,12,13,14,15,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,39,40,41,42,43,44,44,45,46,47,48,48,49,50,50,51,52,52,53,54,54,55,55,56,57,57,58,58,58,59,59,60,60,61,61,61,62,62,62,62,63,63,63,63,63,64,64,64,64,64,64,64,64
};

 // initial viewer settings
int g_viewerX = STARTX;
int g_viewerY = STARTY;
int g_viewerAngle = STARTANGLE;

// Interrupthandler for mpu
void dmpDataReady() {
  mpuInterrupt = true;
}

// function to get precalculated sins
signed char sin64(int degree) {
  degree = degree % 360;
  if (degree < 0) degree+=360;
  if (degree < 90) return g_sin64[degree];
  if (degree < 180) return g_sin64[180-degree];
  if (degree < 270) return - g_sin64[degree-180];
  return -g_sin64[360-degree];
}

// check if box in grid is filled with wall
bool isBoxFilled(long x, long y) {
  
  if ((x<0) || (x>MAPWIDTH*GRIDSIZE-1)) return false; // out of map
  if ((y<0) || (y>MAPHEIGHT*GRIDSIZE-1)) return false; // out of map
  
  return (((pgm_read_byte(&(g_map[y/GRIDSIZE]))>>(MAPWIDTH-x/GRIDSIZE-1)) & 1));
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
  signed char cachedCos64, cachedSin64; 
  long cachedTan;
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

  // loop to complete screenwidth
  for (byte viewPortX=0;viewPortX<SCREEN_WIDTH;viewPortX+=2) {
    angle = (angle + 1)%360;
    
    side = SIDEUNKNOWN;
    finalCrossingFound = false;
    crossingX = g_viewerX;
    crossingY = g_viewerY;
    cachedSin64 = sin64(angle);
    cachedCos64 = sin64(90-angle);
    if (cachedCos64 != 0) cachedTan = ((((long)cachedSin64)<<6)/cachedCos64); else cachedTan = HUGEBIGNUMBER; // tan(a) = sin(a)/cos(a)
    distanceX = HUGEBIGNUMBER;
    distanceY = HUGEBIGNUMBER;

    do { // left or right (crossing walls vertically)
      if ((crossingX == g_viewerX) && (crossingY == g_viewerY)) { // first step
        if (cachedCos64 > 1){ // right
          crossingX=(g_viewerX/GRIDSIZE)*GRIDSIZE+GRIDSIZE; // grid on right
          crossingY=g_viewerY - (((g_viewerX - crossingX)*cachedTan)>>6) + 1;
          // delta for the next steps
          deltaX = GRIDSIZE;
          deltaY = (deltaX*cachedTan)>>6;
        } else if (cachedCos64 < -1){ // left
          crossingX=(g_viewerX/GRIDSIZE)*GRIDSIZE-1; // grid on left
          crossingY=g_viewerY - (((g_viewerX - crossingX)*cachedTan)>>6);
          deltaX = -GRIDSIZE;
          deltaY = (deltaX*cachedTan)>>6;
        } else {
          // too close to up or down
          crossingX=g_viewerX; 
          crossingY=g_viewerY; 
          finalCrossingFound = true;
        }
       } else { // following steps       
        if (isInMap(crossingX,crossingY) && isBoxFilled(crossingX,crossingY)) { // Wall found
            distanceX=(cachedCos64*(crossingX-g_viewerX)+cachedSin64*(crossingY-g_viewerY))>>6; // calculate distance between points by using transformation of Pythagorean trigonometric identity (faster then sqrt(dx^2+dy^2)).
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

    if (cachedTan == 0) { cachedTan = 1; }; // Prevent DIV0
    
    do { // up or down (crossing wall horizontally
      if ((crossingX == g_viewerX) && (crossingY == g_viewerY)) { // first step
        if (cachedSin64 < -1){ // up
          crossingY=(g_viewerY/GRIDSIZE)*GRIDSIZE-1; // upper grid
          crossingX=g_viewerX - ((g_viewerY - crossingY)<<6)/cachedTan;
          // delta for the next steps
          deltaY = -GRIDSIZE;
          deltaX = (deltaY<<6)/cachedTan;
        } else if (cachedSin64 > 1){ // down
          crossingY=(g_viewerY/GRIDSIZE)*GRIDSIZE+GRIDSIZE;
          crossingX=g_viewerX - ((g_viewerY - crossingY)<<6)/cachedTan;
          deltaY = GRIDSIZE;
          deltaX = (deltaY<<6)/cachedTan;
        } else {
          // too close to left or right
          crossingX=g_viewerX; 
          crossingY=g_viewerY; 
          finalCrossingFound = true;
        }
       } else { // following steps       
        if (isInMap(crossingX,crossingY) && isBoxFilled(crossingX,crossingY)) { // Wall found
          distanceY=(cachedCos64*(crossingX-g_viewerX)+cachedSin64*(crossingY-g_viewerY))>>6; // calculate distance between points by using transformation of Pythagorean trigonometric identity (faster then sqrt(dx^2+dy^2)).
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
      if( distanceY < distanceX -1){ 
        finalCrossingX=crossingX; 
        finalCrossingY=crossingY; 
        side = SIDEUPDOWN;
        minDistance = distanceY;
      } else if (distanceX < distanceY -1) { 
        minDistance = distanceX;
        side = SIDELEFTRIGHT;
      } else { 
        // can not determine if horizontal or vertical => use last used side
        minDistance = distanceX;
        side = lastSide;
      }

      minDistance= ((long)minDistance* (signed char) sin64(90-(g_viewerAngle-angle))) >> 6; //fisheye reduce
      
      height = (long) 90*VIEWPORT_HEIGHT*STRIPEHEIGHT/minDistance; // Current stripheight

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
      if (side == SIDEUPDOWN) { // if vertical wall face => calc texture column from crossing x value MOD wall heigt and fix column direction dependend on up/down
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
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Default Address is 0x3D for 128x64, but my oled uses 0x3C 
    // SSD1306 allocation failed
    blink(1000);
    blink(1000);
    blink(1000);
    while (true);
  }

  // Font settings
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.print("Init gyro...");
  display.display();

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
    mpu.PrintActiveOffsets();
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
    newX = g_viewerX + sin64(90-g_viewerAngle) * direction;
    newY = g_viewerY + sin64(g_viewerAngle) * direction;
    if (isInMap(newX,newY) && !isBoxFilled(newX,newY)) {
      g_viewerX = newX;
      g_viewerY = newY;
    }
    if ((g_viewerX/GRIDSIZE == FINISHGRIDX) && (g_viewerY/GRIDSIZE == FINISHGRIDY)) { // Finish reached
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(50,24);
      display.print("Fin");
      display.display();
      delay(10000);
      display.setTextSize(1);
      g_viewerX = STARTX;
      g_viewerY = STARTY;
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
