/*
 * https://circuitdigest.com/microcontroller-projects/arduino-based-women-safety-device-for-emergency-alert-and-tracking
 * 2,3,12,4 pins are used as mentioned in the original project and in the above site
 * 7,A4,A5 PINS FOR MPU6050
 * key is for button 1 to send emergency message
 * key1 is used for button 2 for " everything is okay message" ( can be used when device is activated by mistake )
 //8th pin is used for isd1820 (voice recording module)
 */

#include <LiquidCrystal.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <Wire.h>          // MPU6050 starting
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

//int data[STORE_SIZE][5]; //array for saving past data
//byte currentIndex=0; //stores current data array index (0-255)
boolean fall = false;     //stores if a fall has occurred
boolean trigger1 = false; //stores if first trigger (lower threshold) has occurred
boolean trigger2 = false; //stores if second trigger (upper threshold) has occurred
boolean trigger3 = false; //stores if third trigger (orientation change) has occurred

byte trigger1count = 0; //stores the counts past since trigger 1 was set true
byte trigger2count = 0; //stores the counts past since trigger 2 was set true
byte trigger3count = 0; //stores the counts past since trigger 3 was set true
int angleChange = 0;    // MPU6050 ending

static const int RXPin = 7, TXPin = 3;
static const uint32_t gps_baudrate = 9600;
TinyGPSPlus gps;
SoftwareSerial soft(RXPin, TXPin);
String textMessage;
float Lat, Lon;

void setup()
{
    Wire.begin(); // MPU6050 STARTING
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);           // PWR_MGMT_1 register
    Wire.write(0);              // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true); // MPU6050 ENDING

    soft.begin(gps_baudrate);
    Serial.begin(9600);
    pinMode(12, INPUT); //FOR KEY 1
    pinMode(11, INPUT); // FOR KEY 2
    pinMode(4, OUTPUT);
    pinMode(8, OUTPUT);   //for isd1820
    digitalWrite(8, LOW); //for isd1820
}

void loop()
{

    int key = digitalRead(12);  // for button 1
    int key1 = digitalRead(11); // for button 2
    while (soft.available() > 0)
    {
        gps.encode(soft.read());
        if (gps.location.isUpdated())
        {
            Lat = gps.location.lat();
            Lon = gps.location.lng();
        }
        else
            ;
    }
    if (key == 1)
    {
        digitalWrite(4, HIGH);
        digitalWrite(8, HIGH); // isd1820
        delay(10000);          // record voice for 10 seconds
        digitalWrite(8, LOW);  //isd1820
        sendsms();
        digitalWrite(4, LOW);
    }

    if (key1 == 1) // for key1
    {

        sendsms1();
    }

    mpu_read();
    //2050, 77, 1947 are values for calibration of accelerometer
    // values may be different for you
    ax = (AcX - 2050) / 16384.00;
    ay = (AcY - 77) / 16384.00;
    az = (AcZ - 1947) / 16384.00;

    //270, 351, 136 for gyroscope
    gx = (GyX + 270) / 131.07;
    gy = (GyY - 351) / 131.07;
    gz = (GyZ + 136) / 131.07;

    // calculating Amplitute vactor for 3 axis
    float Raw_AM = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
    int AM = Raw_AM * 10; // as values are within 0 to 1, I multiplied
                          // it by for using if else conditions

    Serial.println(AM);
    //Serial.println(PM);
    //delay(500);

    if (trigger3 == true)
    {
        trigger3count++;
        //Serial.println(trigger3count);
        if (trigger3count >= 10)
        {
            angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
            //delay(10);
            Serial.println(angleChange);
            if ((angleChange >= 0) && (angleChange <= 10))
            { //if orientation changes remains between 0-10 degrees
                fall = true;
                trigger3 = false;
                trigger3count = 0;
                Serial.println(angleChange);
            }
            else
            { //user regained normal orientation
                trigger3 = false;
                trigger3count = 0;
                Serial.println("TRIGGER 3 DEACTIVATED");
            }
        }
    }
    if (fall == true)
    { //in event of a fall detection
        Serial.println("FALL DETECTED");
        digitalWrite(11, LOW);
        delay(20);
        digitalWrite(11, HIGH);
        digitalWrite(4, HIGH);
        digitalWrite(8, HIGH); // isd1820
        delay(10000);          // record voice for 10 seconds
        digitalWrite(8, LOW);  //isd1820
        sendsms();
        digitalWrite(4, LOW);
        fall = false;
        // exit(1);
    }
    if (trigger2count >= 6)
    { //allow 0.5s for orientation change
        trigger2 = false;
        trigger2count = 0;
        Serial.println("TRIGGER 2 DECACTIVATED");
    }
    if (trigger1count >= 6)
    { //allow 0.5s for AM to break upper threshold
        trigger1 = false;
        trigger1count = 0;
        Serial.println("TRIGGER 1 DECACTIVATED");
    }
    if (trigger2 == true)
    {
        trigger2count++;
        //angleChange=acos(((double)x*(double)bx+(double)y*(double)by+(double)z*(double)bz)/(double)AM/(double)BM);
        angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
        Serial.println(angleChange);
        if (angleChange >= 30 && angleChange <= 400)
        { //if orientation changes by between 80-100 degrees
            trigger3 = true;
            trigger2 = false;
            trigger2count = 0;
            Serial.println(angleChange);
            Serial.println("TRIGGER 3 ACTIVATED");
        }
    }
    if (trigger1 == true)
    {
        trigger1count++;
        if (AM >= 12)
        { //if AM breaks upper threshold (3g)
            trigger2 = true;
            Serial.println("TRIGGER 2 ACTIVATED");
            trigger1 = false;
            trigger1count = 0;
        }
    }
    if (AM <= 2 && trigger2 == false)
    { //if AM breaks lower threshold (0.4g)
        trigger1 = true;
        Serial.println("TRIGGER 1 ACTIVATED");
    }
    //It appears that delay is needed in order not to clog the port
    delay(100);
}

void mpu_read()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

// for key( button1 )
void sendsms()
{
    Serial.print("AT+CMGF=1\r");
    delay(100);
    Serial.println("AT+CMGS =\"+919870254765\"");
    delay(100);
    Serial.println("I want help !!!Location: " + String("Lat: ") + String(Lat) + " " + String("Lon: ") + String(Lon));
    delay(100);
    Serial.println((char)26);
    delay(100);
    Serial.println();
    delay(5000);
}

// for key1 ( button2 )
void sendsms1()
{
    Serial.print("AT+CMGF=1\r");
    delay(100);
    Serial.println("AT+CMGS =\"+919870254765\"");
    delay(100);
    Serial.println(" Everything is okay ");
    delay(100);
    Serial.println((char)26);
    delay(100);
    Serial.println();
    delay(5000);
}
