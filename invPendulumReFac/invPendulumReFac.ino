/*
"Another easier inverted pendulum balancing robot"
You need only half a day to make it, if you have some Materials.
(This sketch is ver.2.0.d for a digital output gyroscope.)
No timer library is used in this version.
But stability of robot is more improved than earlier version.
Copyright (C) 2014 ArduinoDeXXX All Rights Reserved.
*/
//#include <MsTimer2.h> //01 (This line is omitted in ver.2.0 and the later.)
//int i = 0; (This line is omitted in this version.)
byte countS = 0; //03
//long zeroOmegaI = 0; (This line is omitted in this version.)

#include <SPI.h>

int ry;
int power;
int omega = 0;
int recOmegaI[10];
const int Kp_pend = 512;
const int Ki_pend = 256;

long R;
long vE5 = 0;
long xE5 = 0;
long powerScale;
long theta = 0;
long distance = 0;
long velocity = 0;
const long Kp_cart = 0;
const long Ki_cart = 0;

void
L3GD20_write(byte reg, byte val) {
	digitalWrite(10, LOW);
	SPI.transfer(reg);
	SPI.transfer(val);
	digitalWrite(10, HIGH);
}

byte
L3GD20_read(byte reg) {
	byte ret = 0;
	digitalWrite(10, LOW);
	SPI.transfer(reg | 0x80);
	ret = SPI.transfer(0);
	digitalWrite(10, HIGH);
	return ret;
}

void
chkAndCtl() {
  /**
   * Taking the avereage over 45 values of the register holding values of the Y-axis
   */
  R = 0;
  for ( int i = 0 ; i < 20 ; i++ ) {
    ry = ( (L3GD20_read(0x2B) << 8) | L3GD20_read(0x2A) );
    R = R + ry;
    delayMicroseconds(90);
  }
  omega = (R / 20)*0.00875 + 4; //+4 to correct for g-scope glitchy offset
  /**
   * If the angular velocit is small enough (2 in this case), just assume it's zero
   */
  if ( abs( omega ) < 3 ) {
    omega = 0;
  }

  /**
   * Integration
   */
  recOmegaI[0] = omega;
  theta = theta + omega;
  countS = 0;

  for ( int i = 0 ; i < 10 ; i++ ) {
    if ( abs( recOmegaI[i] ) < 3 ) {
        countS++; 
    }
  }
  if ( countS > 6 ) {
    theta = 0;
    vE5 = 0;
    xE5 = 0;
    velocity = 0;
    distance = 0;
  }

  /**
   * Shift recent angular velocity values in array
   */
  for ( int i = 9 ; i > 0 ; i-- ) {
    recOmegaI[ i ] = recOmegaI[ i-1 ];
  }
  
  powerScale = 
    ( Ki_pend * theta / 100 ) +
    ( Kp_pend * omega / 100 ) +
    ( Kp_cart * vE5 / 1000 ) +
    ( Ki_cart * xE5 / 1000 );
  power = max ( min ( 5 * powerScale / 100 , 255 ) , -255 );
  
  velocity = velocity + power;
  distance = distance + velocity;
}

void 
setup () {

  Serial.begin(115200);

  /**
   * Motor 1
   */
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);

  /**
   * Motor 2
   */
	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	pinMode(9, OUTPUT);

  /**
   * SPI CS
   */
	pinMode(10, OUTPUT); //(These 8 lines, DL18-DL25, are added in this version.)
	digitalWrite(10, HIGH);

  /**
   * Initialize array of recent ang. velocity to al 0
   */
	for ( int i = 0 ; i < 10 ; i++ ) {
		recOmegaI[i] = 0;
	}

	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE3);
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	L3GD20_write(0x20, B11001111);
	L3GD20_write(0x23, B00000000);

	delay(300);
}

void
loop () {
	chkAndCtl();
  Serial.print(omega);
  Serial.print(" ");
  Serial.print(vE5);
  Serial.print(" ");
  Serial.println(power);
  
//  Serial.println(power);
	if ( power > 0 ) {
//    power /= 1.0;
		analogWrite (9, power);
		digitalWrite(7, HIGH);
		digitalWrite(8, LOW);
    analogWrite (6, power);
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
	} else {
//    power /= 1.2;
		analogWrite (6, -power);
		digitalWrite(4, LOW);
		digitalWrite(5, HIGH);
		analogWrite (9, -power);
		digitalWrite(7, LOW);
		digitalWrite(8, HIGH);
	}
  vE5 = velocity;
  xE5 = distance;
}
// Copyright (C) 2014 ArduinoDeXXX All Rights Reserved. //79
