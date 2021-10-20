#include "crc.hpp"

void main(int argc, char *argv[]){
	CenterResiliencyCoordinator::CenterRC CRC;
	while(1){
		CRC.Communicate();
	}
}
