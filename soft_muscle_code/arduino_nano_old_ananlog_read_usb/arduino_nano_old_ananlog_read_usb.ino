String to_print = "";
int sensor_value = 0;
const int numSensors = 8;
int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

void setup() {
  // Start the serial communication at 115200 baud rate
  Serial.begin(115200);

  // Setup each analog pin as input
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

void loop() {

  for (int i = 0; i < numSensors; i++) {
    to_print = ""; 
    sensor_value = analogRead(sensorPins[i]);
    to_print += String(i) + ":" + String(sensor_value) + " ";
    Serial.println(to_print);
    delay(3);  // 2ms delay after each print
  }
}


