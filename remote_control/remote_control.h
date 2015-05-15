#include <stdint.h>

#define NROWS 11
#define NCOLS  4

#define CHEAP_IR_CONTROLLER_CHIP 0
#if CHEAP_IR_CONTROLLER_CHIP
#define C(CODE, FUNC) { 0x##CODE, FUNC }
#else
#define C(CODE, FUNC) { 0xFF##CODE, FUNC }
#endif

// These are the codes from the IR remote control that was sold with an RGB LED controller 
struct remote_key {
   	uint32_t code;
	const char *name;
	void (*cb)(void *);
	void *cbdata;
} remote_codes[NROWS][NCOLS] = {
#if CHEAP_IR_CONTROLLER_CHIP
/* codes seen by the IR chip from the cheap controller  */
	    { C(B3D4B87F, "light up"),	C(44490A7B, "light down"),	C(3195A31F, "play"),	C(D7E84B1B, "off"), },
		{ C(A8E05FBB, "red"),       C(3954B1B7, "green"),       C(E318261B, "blue"),    C(52A3D41F, "white"), },
		{ C(5B83B61B, "orange"),	C(B5310E1F, "green2"),		C(73CEC633, "blue2"),	C(E35A7437, "beige"), },
		{ C(B08CB7DF, "orange2"), 	C(410109DB, "turquoise"),   C(7EC31EF7, "violet"),	C(EE4ECCFB, "beige2"), },
		{ C(488F3CBB, "orange3"),	C(A23C94BF, "turquoise2"), 	C(F63C8657, "brown"),	C(E721C0DB, "cyan"), },
		{ C(3D9AE3F7, "yellow"), 	C(97483BFB, "turquoise3"), 	C(DC0197DB, "pink"),	C(86B0E697, "cyan2"), },
		{ C(13549BDF, "redup"), 	C(A3C8EDDB, "greenup"),		C(C101E57B, "blueup"),	C(5BE75E7F, "quick"), },
		{ C(45473C1B, "reddown"), 	C(9EF4941F, "greendown"),	C(F377C5B7, "bluedown"),C(8E5D3EBB, "slow"), },
		{ C(9716BE3F, "diy1"), 		C(F0C41643, "diy2"), 		C(44C407DB, "diy3"),	C(35A9425F, "auto"), },
		{ C(8C22657B, "diy4"), 		C(E5CFBD7F, "diy5"), 		C(2A89195F, "diy6"),	C(D538681B, "flash"), },
		{ C(51E43D1B, "jump3"), 	C(AB91951F, "jump7"), 		C(FF9186B7, "fade3"),	C(F076C13B, "fade7"), },
#else
/* codes seen by the TSOP4838 sensor */
	    { C(3AC5, "light up"),	C(BA45, "light down"),	C(827D, "play"),	    C(02FD, "off"), },
		{ C(1AE5, "red"),       C(9A65, "green"),       C(A25D, "blue"),    C(22DD, "white"), },
		{ C(2AD5, "orange"),	C(AA55, "green2"),		C(926D, "blue2"),	C(12ED, "beige"), },
		{ C(0AF5, "orange2"), 	C(8A75, "turquoise"),   C(B24D, "violet"),	C(32CD, "beige2"), },
		{ C(38C7, "orange3"),	C(B847, "turquoise2"), 	C(7887, "brown"),	C(F807, "cyan"), },
		{ C(18E7, "yellow"), 	C(9867, "turquoise3"), 	C(58A7, "pink"),	C(D827, "cyan2"), },
		{ C(28D7, "redup"), 	C(A857, "greenup"),		C(6897, "blueup"),	C(E817, "quick"), },
		{ C(08F7, "reddown"), 	C(8877, "greendown"),	C(48B7, "bluedown"),C(C837, "slow"), },
		{ C(30CF, "diy1"), 		C(B04F, "diy2"), 		C(708F, "diy3"),	C(F00F, "auto"), },
		{ C(10EF, "diy4"), 		C(906F, "diy5"), 		C(50AF, "diy6"),	C(D02F, "flash"), },
		{ C(20DF, "jump3"), 	C(A05F, "jump7"), 		C(609F, "fade3"),	C(E01F, "fade7"), },
#endif

};

const struct remote_key unknown_key = { 0, "UNKNOWN", NULL, NULL };


