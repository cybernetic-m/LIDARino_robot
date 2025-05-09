char buffer[256];

void setup() {
    Serial.begin(9600);	// opens serial port, sets data rate to 9600 bps
    // set up the LCD's number of columns and rows:
}

void loop() {

	// send data only when you receive data:
	if (Serial.available() > 0) {

	// read the data
        Serial.readBytes(buffer, 256);

        Serial.print(buffer);

	}

}