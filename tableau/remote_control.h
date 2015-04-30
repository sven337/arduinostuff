#include <stdint.h>

#define NROWS 11
#define NCOLS  4

// These are the codes from the IR remote control that was sold with an RGB LED controller 
struct remote_key {
   	uint32_t code;
	const char *name;
	void (*cb)(void *);
	void *cbdata;
};

struct color {
	uint8_t r, g, b;
};

#define COLOR(NAME, R, G, B) const struct color constant_color_##NAME = { R, G, B }


COLOR(red,	  255,	 0,	 0);
COLOR(green,	0,	255,	 0);
COLOR(blue,	 	0,	 0,	 255);
COLOR(white,   255,255,	 255);
COLOR(orange,  255,0xA5,    0);
COLOR(green2,	 0,	 0,	 0);
COLOR(blue2,	 0,	 0,	 0);
COLOR(beige,	 0,	 0,	 0);
COLOR(orange2,	 0,	 0,	 0);
COLOR(turquoise,	 0,	 0,	 0);
COLOR(violet,	 0,	 0,	 0);
COLOR(beige2,	 0,	 0,	 0);
COLOR(orange3,	 0,	 0,	 0);
COLOR(turquoise2 ,	 0,	 0,	 0);
COLOR(brown,	 0,	 0,	 0);
COLOR(cyan,	 0,	 255,	 255);
COLOR(yellow,	 255,	 255,	 0);
COLOR(turquoise3,	 0,	 0,	 0);
COLOR(pink,	 0,	 0,	 0);
COLOR(cyan2,	 0,	 0,	 0);

void remote_cb_light(void *bool_increase);
void remote_cb_play(void *bool_play);
void remote_cb_colortweak(void *char_color);
void remote_cb_color(void *struct_color);
void remote_cb_NULL(void *arg);
void remote_cb_diy(void *int_number);
void remote_cb_strobe(void *none);
void remote_cb_strobeperiod(void *bool_decrease);
void remote_cb_jump(void *int_number);
void remote_cb_fade(void *int_number);

#define C(CODE, FUNC, CB, DATA) { 0xFF##CODE, FUNC, &remote_cb_##CB, (void *)DATA }
#define CC(CODE, FUNC) { 0xFF##CODE, #FUNC, &remote_cb_color, (void *)&constant_color_##FUNC }
const struct remote_key remote_codes[NROWS][NCOLS] = {
/* codes seen by the TSOP4838 sensor */
	    { C(3AC5, "light up", light, 1),	C(BA45, "light down", light, 0),	C(827D, "play", play, 1),	    C(02FD, "off", play, 0), },
		{ CC(1AE5, red),       	CC(9A65, green),       CC(A25D, blue),    CC(22DD, white), },
		{ CC(2AD5, orange),		CC(AA55, green2),		CC(926D, blue2),	CC(12ED, beige), },
		{ CC(0AF5, orange2), 	CC(8A75, turquoise),   CC(B24D, violet),	CC(32CD, beige2), },
		{ CC(38C7, orange3),	CC(B847, turquoise2), 	CC(7887, brown),	CC(F807, cyan), },
		{ CC(18E7, yellow), 	CC(9867, turquoise3), 	CC(58A7, pink),	CC(D827, cyan2), },
		{ C(28D7, "redup", colortweak, 'R'), 	C(A857, "greenup", colortweak, 'G'),		C(6897, "blueup", colortweak, 'B'),	C(E817, "quick", strobeperiod, 1 ), },
		{ C(08F7, "reddown", colortweak, 'r'), 	C(8877, "greendown", colortweak, 'g'),	C(48B7, "bluedown", colortweak, 'b'), C(C837, "slow", strobeperiod, 0 ), },
		{ C(30CF, "diy1", diy, 1), 		C(B04F, "diy2", diy, 2), 		C(708F, "diy3", diy, 3),	C(F00F, "auto", NULL, NULL), },
		{ C(10EF, "diy4", diy, 4), 		C(906F, "diy5", diy, 5), 		C(50AF, "diy6", diy, 6),	C(D02F, "flash", strobe, NULL), },
		{ C(20DF, "jump3", jump, 3), 	C(A05F, "jump7", jump, 7), 		C(609F, "fade3", fade, 3),	C(E01F, "fade7", fade, 7), },

};

const struct remote_key unknown_key = { 0, "UNKNOWN", NULL, NULL };


