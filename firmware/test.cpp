#include <iostream>
#include <fstream>

int main(int argc, char* argv[])
{
	//open arduino device file (linux)
        std::ofstream arduino;
	arduino.open( "/dev/ttyACM1");

	//write to it
        arduino << "Hello from C++!";
	arduino.close();

	return 0;
}