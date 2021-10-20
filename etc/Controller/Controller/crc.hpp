#include "sock_udp.hpp"

namespace CenterResiliencyCoordinator{

class CenterRC{
	public:
		CenterRC();
		~CenterRC();

	private:
		bool Alpha_;
		bool Beta_;
		bool Gamma_;
		float CurVel_;
		float TarVel_;
		float PredVel_;
		float CurDist_;
		float TarDist_;
		uint8_t LrcMode_;
		uint8_t CrcMode_;
};

}

