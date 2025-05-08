char buffer[256];

void setup() {
    Serial.begin(9600);  // Make sure this matches your C++ code
}

void loop() {
    // For testing: send a simple message every second
    const char* msg = "Hello from Arduino!";
    Serial.println(msg);
    delay(1000);
}