#include <PID_v1.h>

#define PRESSURE_SENSOR_PIN A0
#define DEBUG_LED_PIN 17
#define MOTOR_CONTROL_PIN 3
#define VCC 5.0f
#define MOTOR_MINIMUM_PWM 50

// TODO: Call the PID method in an OCR interrupt
// TODO: Tune the pid
// https://controls.engin.umich.edu/wiki/index.php/PIDTuningClassical#Trial_and_Error
// http://www.ni.com/white-paper/3782/en/

void SerialReceive();
void SerialSend();
unsigned long serialTime; // This will help us know when to talk with processing

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 2, 1, 0, DIRECT);

float readPressureSensor() {
    int analogValue = analogRead(PRESSURE_SENSOR_PIN);
    float voltage = (analogValue / 1024.0f) * VCC;
    // Pressure sensor returns between 0.5V – 4.5V, corresponding to 0-100psi
    float pressure = ((voltage - 0.5) / 4) * 100.0;
    return pressure;
}

ISR(TIMER1_COMPA_vect) {
    // (1 / 100s) * (16000000Hz / 256) = 625
    OCR1A = TCNT1 + 625;
    float pressure = readPressureSensor();
    Input = pressure;
    myPID.Compute();
    uint8_t motorOutput = Output;
    if (motorOutput <= MOTOR_MINIMUM_PWM) {
        motorOutput = 0;
    }
    analogWrite(MOTOR_CONTROL_PIN, motorOutput);
}

void setup() {
    // Disable interrupts
    cli();

    TCCR1A = 0;
    TCCR1B = 1 << CS12; // Clock / 256 (31250Hz or .000032)
    TCCR1C = 0; // not forcing output compare
    TCNT1 = 0; // set timer counter initial value (16 bit value)
    TIMSK1 = 1 << OCIE1A; // enable timer compare match 1A interrupt    

    Serial.begin(115200);
    pinMode(DEBUG_LED_PIN, OUTPUT);
    pinMode(PRESSURE_SENSOR_PIN, INPUT);
    pinMode(MOTOR_CONTROL_PIN, OUTPUT);
    digitalWrite(MOTOR_CONTROL_PIN, LOW);

    // Initialize the variables we're linked to
    Input = readPressureSensor();
    Setpoint = 35;

    // Turn the PID on
    myPID.SetOutputLimits(MOTOR_MINIMUM_PWM, 255);
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(10);

    // Enable interrupts
    sei();
}

void loop() {
    // Send-receive with processing if it's time
    if (millis() > serialTime) {
        SerialReceive();
        SerialSend();
        serialTime += 300;
    }
}


/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{
  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else foo.asBytes[index-2] = Serial.read();
    index++;
  }

  // if the information we got was in the correct format,
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    Setpoint=double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the
                                          //   value of "Input"  in most cases (as
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in
    {                                     //   manual mode.  otherwise we'll get an
      Output=double(foo.asFloat[2]);      //   output blip, then the controller will
    }                                     //   overwrite.

    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    myPID.SetTunings(p, i, d);            //

    if(Auto_Man==0) myPID.SetMode(MANUAL);// * set the controller mode
    else myPID.SetMode(AUTOMATIC);             //

    if(Direct_Reverse==0) myPID.SetControllerDirection(DIRECT);// * set the controller Direction
    else myPID.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend() {
    Serial.print("PID ");
    Serial.print(Setpoint);
    Serial.print(" ");
    Serial.print(Input);
    Serial.print(" ");
    Serial.print(Output);
    Serial.print(" ");
    Serial.print(myPID.GetKp());
    Serial.print(" ");
    Serial.print(myPID.GetKi());
    Serial.print(" ");
    Serial.print(myPID.GetKd());
    Serial.print(" ");
    if(myPID.GetMode()==AUTOMATIC) Serial.print("Automatic");
    else Serial.print("Manual");
    Serial.print(" ");
    if(myPID.GetDirection()==DIRECT) Serial.println("Direct");
    else Serial.println("Reverse");
}
