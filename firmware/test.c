#include <iostream>
#include <string>
#include <iomanip>

int main()
{
	char lower = 'h'; 
	char upper = lower ^ 0x20;
	std::cout << std::hex << (int)lower << "; " << std::hex << (int)upper << std::endl;
}
