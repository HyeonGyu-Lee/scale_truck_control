#include "crc.hpp"
#include <stdio.h>
#include <sys/time.h>

int main(int argc, char *argv[]){
	CenterResiliencyCoordinator::CenterRC CRC;
	
	while(1){
		CRC.Communicate();
	}

	return 0;
}
