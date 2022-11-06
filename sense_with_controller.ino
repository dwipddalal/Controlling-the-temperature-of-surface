// With Controller
// For the Analog and digital temperature sensor module (NTC Thermistor Temperature Sensor)
// Temprature Control using Arduino
// ES311 - Heat and Mass Transfer 
// Group-A3


// variables and definitions

      //Important parameter, set to match environment
          const int dt = 1000; // [ms] time constant in milliseconds (controller clock rate = 1/(dt/1000) [Hz])
          #define SetTemp 20 // [degC] set temperature in DegC
          #define MinTemp 17 // [degC] minimum expected temperature (needed for rescaling inputs)
          #define MaxTemp 23 // [degC] maximum allowed temperature, over which heater is turned off (needed for rescaling inputs)
          int SetTime = 1800; // [s] timer in seconds, if reached, running stops [Default: 1800]

      //I/O pins - don't edit unless replaced
          #define thermistorPin A0
          #define relay 10
//        #define LEDPin //number of LED pin (optional)

// code hasn't been tuned yet. A PID control tuning is the method of adjusting the value of Kp, Kd, Ki so s to follow 


      //control parameters - editing not recommended     
          double K_P_ctrl = 200; //proportional gain 10
          double K_I_ctrl = 0.01; //integral gain (set to lower values i.e. 10^-3) 0.2
          double K_D_ctrl = 10; //derivative gain 5

// including headers and definitions
  #include <math.h>

//Inititalization
    //target temperature reached?
       bool bInRange = 0; // 0 means not 

    //ticks per ms
       int TicksPerMS = floor(1000/dt);

    //Initialize PID variables:
       float previous_error = 0;
       float s_integral = 0;

// //Thermistor code
//     double Thermistor(int RawADC) {
//       double Temp;
//       Temp = log(10000.0*((1024.0/RawADC-1))); 
//       Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
//       Temp = Temp - 273.15;            // Convert Kelvin to Celcius
//       return Temp;
//     }

//PID controller code
    void Control_PID(double iTemp){
        
      //Overheat protection
          if(iTemp>MaxTemp){
            analogWrite(relay, 255);
            Serial.println("Temperature outside the maximum bounds.");
            Serial.println("Turn on the cooler at full speed.");
            return;
          }
        
      //In range? If in range, maybe turn on LED?
          if((iTemp) >= SetTemp){
            if(bInRange==0){
              //digitalWrite(LEDPin, HIGH); 
              bInRange=1;
            }
          }else{
            if(bInRange==1){
              //digitalWrite(LEDPin, LOW); 
              bInRange=0;
            }
          }
          
        
        //PID subroutine
          Serial.print("Error: ");
          float err = iTemp - SetTemp;
          Serial.println(err);
          s_integral += err*dt;
          //Serial.println(s_integral);
          float s_derivative = (err - previous_error)/dt;
          //Serial.println(s_derivative);
          int U_in_ctrl = (K_P_ctrl*err + K_I_ctrl*s_integral + K_D_ctrl*s_derivative);
          previous_error = err;
                 
          
        // put voltage to output and write value to serial monitor
            // Serial.print("Output PWM frequency: ");
            // Serial.print("Total error due to contributions from PID:");
            // Serial.println(U_in_ctrl);
            // Serial.print("kp:");
            // Serial.println(K_P_ctrl*err);
            // Serial.print("kI:");
            // Serial.println(K_I_ctrl*s_integral);
            // Serial.print("kD:");
            // Serial.println(K_D_ctrl*s_derivative);
            Serial.println(SetTemp);
            if (U_in_ctrl<=255){
               if (U_in_ctrl > 0){
                  analogWrite(relay, U_in_ctrl);
                  // Serial.println(U_in_ctrl);                
               }           
               else
               {
                  analogWrite(relay, 0);
                  // Serial.println("1 - cca. 0 V"); 
               }
            }
            else{
               analogWrite(relay,255); // relay is turned on  digtalwrite
              //  Serial.println("255 - cca. 5 V");             
            }                   
    }

void setup() {
    Serial.begin(9600);
    pinMode(relay, OUTPUT);
  //pinMode(LEDPin, OUTPUT);
  
    //rescale timer according to dt
    SetTime = SetTime * TicksPerMS;
}

void loop() {
  //Take a temperature reading and display it 
    // double Temp = double(Thermistor(analogRead(thermistorPin)));
    // double Temp = double(map(((analogRead(thermistorPin) - 20) * 3.04), 0, 1023, -40, 125));
    double Temp = double ((1-analogRead(thermistorPin)*3.75/1024.0)*100);
    Serial.print("Temperature:");
    Serial.println(Temp);  // display temperature
  
  // //Timer serial out - displays time on serial monitor
  //   Serial.print(SetTime/60*dt/1000);
  //   Serial.print(" [mins] - SetTime: ");
  //   Serial.print(SetTime);
  //   Serial.println("");
        
  //Call controller algorithm 
    Control_PID(Temp); // call controller algorithm
  
  //End line in serial monitor...
    Serial.println("");
    Serial.println("");
   
  //Timer ticking (countdown)
    if (SetTime>0){
        SetTime--;
        // if zero reached
        if (SetTime==0){

          while(1) {
            //loop until disconnected
            Serial.println("Time ran out, controller stopped. Please disconnect or reset the controller.");
            digitalWrite(relay, LOW);
            delay(dt);
          } 
        }
    }
    
   //wait dt before next cycle
      delay(dt); 
}
