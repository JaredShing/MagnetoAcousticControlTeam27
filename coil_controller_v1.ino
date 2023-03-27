#define en_coil_1 2
#define en_coil_2 3
#define en_coil_3 4
#define en_coil_4 5
#define en_coil_5 6
#define en_coil_6 7

#define in_a_coil_1 52
#define in_b_coil_1 53
#define in_a_coil_2 51
#define in_b_coil_2 50
#define in_a_coil_3 49
#define in_b_coil_3 48
#define in_a_coil_4 47
#define in_b_coil_4 46
#define in_a_coil_5 45
#define in_b_coil_5 44
#define in_a_coil_6 43
#define in_b_coil_6 42

#define acoustic_driver 10
#define phase_reset 11
#define phase_up 12
#define phase_down 13


const byte numChars = 6; // Form "X1001\n"
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
char coil_name;
char coil_num;
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

void setup()
{
  // Setup Acoustic Controls
  pinMode(acoustic_driver,OUTPUT);
  pinMode(phase_reset,OUTPUT);  // Button 3
  pinMode(phase_up,OUTPUT);     // Button 2
  pinMode(phase_down,OUTPUT);   // Button 1

  Serial.begin(9600);

}

void loop()
{
  recvWithEndMarker();
  if (newData == true){
    pwm_read = String(receivedChars).substring(2,5);
    coil_name = receivedChars[0];
    coil_num = receivedChars[1];
    
    if (receivedChars[0] == 'U'){
      // Increase Phase
      button_press(phase_up);
      Serial.println("Phase up");
    }else if(receivedChars[0] == 'D'){
      // Decrease Phase
      button_press(phase_down);
      Serial.println("Phase down");

    }else if(receivedChars[0] == 'R'){
      // Reset Phase
      button_press(phase_reset);
      Serial.println("Phase reset");

    }
    else if(receivedChars[0] == 'E'){
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
        coil_1.set_pwm(pwm_read.toInt());
        Serial.print("X Coil 1: ");
        Serial.println(pwm_read);
      }else{
        coil_2.set_pwm(pwm_read.toInt());
        Serial.print("X Coil 2: ");
        Serial.println(pwm_read);
      }
      
    }else if(coil_name == 'Y'){
      if (coil_num == '1'){
        coil_3.set_pwm(pwm_read.toInt());
        Serial.print("Y Coil 1: ");
        Serial.println(pwm_read);
      }else{
        coil_4.set_pwm(pwm_read.toInt());
        Serial.print("Y Coil 2: ");
        Serial.println(pwm_read);
      }

    }else if(coil_name == 'Z'){
      if (coil_num == '1'){
        coil_5.set_pwm(pwm_read.toInt());
        Serial.print("Z Coil 1: ");
        Serial.println(pwm_read);
      }else{
        coil_6.set_pwm(pwm_read.toInt());
        Serial.print("Z Coil 2: ");
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

