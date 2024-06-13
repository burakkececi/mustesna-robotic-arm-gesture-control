#include <Servo.h>

Servo servo[4];

int x;
int y;
int z;
int claw;

void setup()
{
    Serial.begin(115200);
    servo[0].write(93);
    servo[1].write(80);
    servo[2].write(115);
    servo[3].write(150);
    servo[0].attach(5); // x
    servo[1].attach(6); // y
    servo[2].attach(7); // z
    servo[3].attach(8); // claw
    setDefault();
}

byte angle[4];
byte pre_angle[4];
long t = millis();

bool open = true;

void loop()
{
    if (Serial.available())
    {
        Serial.readBytes(angle, 4);

        for (size_t i = 0; i < 4; i++)
        {
            if (angle[i] != pre_angle[i])
            {
                Serial.print(angle[i]);
                Serial.print(" ");
                servo[i].write(angle[i]);
                pre_angle[i] = angle[i];
            }
            Serial.println();
        }
        t = millis();
    }

    if (millis() - t > 4000)
    {
      setDefault();
    }

}

void setDefault(){
  Serial.println("Çıkış Yapılıyor");
  x = servo[0].read();
  y = servo[1].read();
  z = servo[2].read();
  claw = servo[3].read();
  if(x>93){
    for(;x>93; x--){
      servo[0].write(x);
      delay(30);
    }
  }
  else{
    for(;x<93; x++){
    servo[0].write(x);
    delay(30);
    }
  }
  if(y>80){
    for(;y>80; y--){
      servo[1].write(y);
      delay(30);
    }
  }
  else{
    for(;y<80; y++){
      servo[1].write(y);
      delay(30);
      }
  }

  if(z>115){
    for(;z>115 ; z--){
      servo[2].write(z);
      delay(30);
      }
    }
  else{
    for(;z<115 ; z++){
      servo[2].write(z);
      delay(30);
      }
    }

  if(claw>150){
    for(;claw>150; claw--){
      servo[3].write(claw);
      delay(30);
      }
    }
  else{
    for(;claw < 150 ; claw++){
    servo[3].write(claw);
    delay(30);
    }
  }

}
