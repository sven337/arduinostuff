#include <stdint.h>

#define NROWS 11
#define NCOLS  4

#define C(CODE, FUNC) { 0x##CODE, FUNC }

// These are the codes from the IR remote control that was sold with an RGB LED controller 
struct {
   	uint32_t code;
	const char *func;
} remote_codes[NROWS][NCOLS] = {
		{ C(B3D4B87F, "light up"),	C(44490A7B, "light down"),	C(3195A31F, "play"),	C(D7E84B1B, "off"), },
		{ C(A8E05FBB, "red"),       C(3954B1B7, "green"),       C(E318261B, "blue"),    C(52A3D41F, "white"), },
		{ C(5B83B61B, "orange"),	C(B5310E1F, "green2"),		C(73CEC633, "blue2"),	C(E35A7437, "beige"), },
		{ C(B08CB7DF, "orange2"), 	C(410109DB, "turquoise"),   C(7EC31EF7, "violet"),	C(EE4ECCFB, "beige2"), },
		{ C(488F3CBB, "orange3"),	C(A23C94BF, "turquoise2"), 	C(F63C8657, "brown"),	C(E721C0DB, "cyan"), },
		{ C(3D9AE3F7, "yellow"), 	C(97483BFB, "turquoise3"), 	C(DC0197DB, "pink"),	C(86B0E697, "cyan2"), },
		{ C(13549BDF, "redup"), 	C(A3C8EDDB, "greenup"),		C(C101E57B, "blueup"),	C(5BE75E7F, "quick"), },
		{ C(45473C1B, "reddowm"), 	C(9EF4941F, "greendown"),	C(F377C5B7, "bluedown"),C(8E5D3EBB, "slow"), },
		{ C(9716BE3F, "diy1"), 		C(F0C41643, "diy2"), 		C(44C407DB, "diy3"),	C(35A9425F, "auto"), },
		{ C(8C22657B, "diy4"), 		C(E5CFBD7F, "diy5"), 		C(2A89195F, "diy6"),	C(D538681B, "flash"), },
		{ C(51E43D1B, "jump3"), 	C(AB91951F, "jump7"), 		C(FF9186B7, "fade3"),	C(F076C13B, "fade7"), },
};
