#include <iostream>
#include <fstream>

int main(int argc, char* argv[])
{
	//open arduino device file (linux)
        std::ifstream arduino;
	arduino.open( "/dev/ttyACM0");

    // Read from it
    std::string response;
    arduino >> response;

    std::cout << "Received: " << response << std::endl;

	return 0;
}
