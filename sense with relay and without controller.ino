// Group-A3

// Imp note:
// If sensor temp > base_temp then turn on the peltier and vice versa
// If relay on peltier on, if relay off peltier off.
// We shall control the relay using 0,1 from the aurdino.


// variables and definitions

//Important parameter, set to match environment
int baselineTemp = 0;
int celsius = 0;
int fahrenheit = 0;
#define relay 8



void setup()
{
  pinMode(A0, INPUT);
  Serial.begin(9600);
  pinMode(relay, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
}

void loop()
{
  // set threshold temperature to activate LEDs
  baselineTemp = 5;
  // measure temperature in Celsius
  celsius = map(((analogRead(A0) - 20) * 3.04), 0, 1023, -40, 125);
  // convert to Fahrenheit
  fahrenheit = ((celsius * 9) / 5 + 32);
  Serial.print(celsius);
  Serial.print(" C, ");
  Serial.print(fahrenheit);
  Serial.println(" F");
  if (celsius < baselineTemp) {
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      digitalWrite(4, LOW);
      digitalWrite(relay, LOW);
    }
  
  if (celsius >= baselineTemp) {
      digitalWrite(2, HIGH);
      digitalWrite(3, LOW);
      digitalWrite(4, LOW);
      digitalWrite(relay, HIGH);
    }

  delay(1000); // Wait for 1000 millisecond(s)
}
