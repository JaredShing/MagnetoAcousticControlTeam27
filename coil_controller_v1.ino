#define en_coil_1 6     // black pwm
#define en_coil_2 2     // brown pwm
#define en_coil_3 5     // green pwm
#define en_coil_4 7     // gray pwm
#define en_coil_5 3     // orange pwm
#define en_coil_6 4     // red pwm
#define en_coil_7 9     // orange pwm
#define en_coil_8 10    // green pwm
#define en_coil_9 8     // gray pwm
#define en_coil_10 11   // red pwm

#define in_a_coil_1 44
#define in_b_coil_1 45
#define in_a_coil_2 53
#define in_b_coil_2 52
#define in_a_coil_3 47
#define in_b_coil_3 46
#define in_a_coil_4 42
#define in_b_coil_4 43
#define in_a_coil_5 50
#define in_b_coil_5 51
#define in_a_coil_6 48
#define in_b_coil_6 49
#define in_a_coil_7 34
#define in_b_coil_7 35
#define in_a_coil_8 32
#define in_b_coil_8 33
#define in_a_coil_9 39
#define in_b_coil_9 38
#define in_a_coil_10 37
#define in_b_coil_10 36

#define acoustic_driver 12


const byte numChars = 7; // Form "X10010\n"
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
char coil_name;
char coil_num;
char coil_dir;
String pwm_read;
bool enable_state = 0;

class Coil{
  public:
    Coil(int pwm_pin, int d1_pin, int d2_pin);
    void set_pwm(int pwm_value);
    void set_dir(int dir);
    int pwm_pin;
    int d1;
    int d2;

  private:
};

Coil::Coil(int pwm_pin, int d1_pin, int d2_pin){
  pinMode(d1_pin, OUTPUT);
  pinMode(d2_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  digitalWrite(d1_pin, HIGH);
  digitalWrite(d2_pin, LOW);
  analogWrite(pwm_pin, 0);
  this->pwm_pin = pwm_pin;
  this->d1 = d1_pin;
  this->d2 = d2_pin;

}

void Coil::set_pwm(int pwm_value){
  analogWrite(pwm_pin, pwm_value);
}
void Coil::set_dir(int dir){
  if (dir == 1){
    digitalWrite(d1, HIGH);
    digitalWrite(d2, LOW);
  }else{
    digitalWrite(d1, LOW);
    digitalWrite(d2, HIGH);
  }

}

// Setup Coils Objects for Control
Coil coil_1(en_coil_1, in_a_coil_1, in_b_coil_1);
Coil coil_2(en_coil_2, in_a_coil_2, in_b_coil_2);
Coil coil_3(en_coil_3, in_a_coil_3, in_b_coil_3);
Coil coil_4(en_coil_4, in_a_coil_4, in_b_coil_4);
Coil coil_5(en_coil_5, in_a_coil_5, in_b_coil_5);
Coil coil_6(en_coil_6, in_a_coil_6, in_b_coil_6);
Coil coil_7(en_coil_7, in_a_coil_7, in_b_coil_7);
Coil coil_8(en_coil_8, in_a_coil_8, in_b_coil_8);
Coil coil_9(en_coil_9, in_a_coil_9, in_b_coil_9);
Coil coil_10(en_coil_10, in_a_coil_10, in_b_coil_10);


void setup()
{
  // Setup Acoustic Controls
  pinMode(acoustic_driver,OUTPUT);
  // pinMode(phase_reset,OUTPUT);  // Button 3
  // pinMode(phase_up,OUTPUT);     // Button 2
  // pinMode(phase_down,OUTPUT);   // Button 1

  Serial.begin(9600, SERIAL_8N1);

}

void loop()
{
  recvWithEndMarker();
  if (newData == true){
    pwm_read = String(receivedChars).substring(2,5);
    coil_name = receivedChars[0];
    coil_num = receivedChars[1];
    coil_dir = receivedChars[5];
    Serial.println(coil_dir);
    
    if(receivedChars[0] == 'E'){
      // Enable Motor Driver
      if (enable_state){
        enable_state = 0;
      }else{
        enable_state = 1;
      }
      driver_enable(enable_state);
      Serial.println(enable_state);
      Serial.println("enable_state");
    }else if (coil_name == 'X'){
      if (coil_num == '1'){
        if  (coil_dir == '1'){
          coil_1.set_dir(1);
        }else{
          coil_1.set_dir(0);
        }
        coil_1.set_pwm(pwm_read.toInt());

        Serial.print("X Coil 1: ");
        Serial.println(pwm_read);
      }else{
        if  (coil_dir == '1'){
          coil_2.set_dir(1);
        }else{
          coil_2.set_dir(0);
        }
        coil_2.set_pwm(pwm_read.toInt());
        Serial.print("X Coil 2: ");
        Serial.println(pwm_read);
      }
      
    }else if(coil_name == 'Y'){
      if (coil_num == '1'){
        if  (coil_dir == '1'){
          coil_3.set_dir(1);
        }else{
          coil_3.set_dir(0);
        }
        coil_3.set_pwm(pwm_read.toInt());
        Serial.print("Y Coil 1: ");
        Serial.println(pwm_read);
      }else{
        if  (coil_dir == '1'){
          coil_4.set_dir(1);
        }else{
          coil_4.set_dir(0);
        }
        coil_4.set_pwm(pwm_read.toInt());
        Serial.print("Y Coil 2: ");
        Serial.println(pwm_read);
      }

    }else if(coil_name == 'Z'){
      if (coil_num == '1'){
        if  (coil_dir == '1'){
          coil_5.set_dir(1);
        }else{
          coil_5.set_dir(0);
        }
        coil_5.set_pwm(pwm_read.toInt());
        Serial.print("Z Coil 1: ");
        Serial.println(pwm_read);
      }else{
        if  (coil_dir == '1'){
          coil_6.set_dir(1);
        }else{
          coil_6.set_dir(0);
        }
        coil_6.set_pwm(pwm_read.toInt());
        Serial.print("Z Coil 2: ");
        Serial.println(pwm_read);
      }
    }else if(coil_name == 'M'){
      if (coil_num == '1'){
        if  (coil_dir == '1'){
          coil_7.set_dir(1);
        }else{
          coil_7.set_dir(0);
        }
        coil_7.set_pwm(pwm_read.toInt());
        Serial.print("M Coil 1: ");
        Serial.println(pwm_read);
      }else if(coil_num == '2'){
        if  (coil_dir == '1'){
          coil_8.set_dir(1);
        }else{
          coil_8.set_dir(0);
        }
        coil_8.set_pwm(pwm_read.toInt());
        Serial.print("M Coil 2: ");
        Serial.println(pwm_read);
      }else if(coil_num == '3'){
        if  (coil_dir == '1'){
          coil_9.set_dir(1);
        }else{
          coil_9.set_dir(0);
        }
        coil_9.set_pwm(pwm_read.toInt());
        Serial.print("M Coil 3: ");
        Serial.println(pwm_read);
      }else{
        if  (coil_dir == '1'){
          coil_10.set_dir(1);
        }else{
          coil_10.set_dir(0);
        }
        coil_10.set_pwm(pwm_read.toInt());
        Serial.print("M Coil 4: ");
        Serial.println(pwm_read);
      }
    }
    newData = false;
  }
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        Serial.println(rc);
        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void button_press(int pin){
  digitalWrite(pin, HIGH);
  delay(50);
  digitalWrite(pin, LOW);
}

void driver_enable(bool enable_state){
  if (enable_state){
    digitalWrite(acoustic_driver, HIGH);
  }else{
    digitalWrite(acoustic_driver, LOW);
  }
}

