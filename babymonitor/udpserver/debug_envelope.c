#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define SILENCE_EMA_WEIGHT 8
#define ENVELOPE_EMA_WEIGHT 1
int silence_value = 0;
int envelope_value = 0;

typedef int32_t fixedpoint;
#define I2F(I) ((I) << 10)
#define F2I(F) ((F) >> 10)
#define D2F(D) ((fixedpoint)((D) * (1 << 10)))
#define F2D(F) ((float)(F) / (1 << 10))

class filter
{
	public:
		filter()
		{
			for(int i=0; i <= 5; i++) {
				v[i]=0;
				u[i]=0;
			}
		}
	private:
		long v[6];
		float u[6];
	public:

#if 0
		3rd order
		short step(short x)
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = v[3];
			long tmp = ((((x *  30021L) >>  2)	//= (   9.1617837037e-1 * x)
				+ ((v[0] * 27462L) >> 2)	//+(  0.8380745366*v[0])
				+ (v[1] * -21831L)	//+( -2.6648818213*v[1])
				+ (v[2] * 23154L)	//+(  2.8264706052*v[2])
				)+4096) >> 13; // round and downshift fixed point /8192

			v[3]= tmp;
			return ((/* xpart */
				 (((v[3] - v[0]))<<13) /* 16384L */
				 + (24576L * (v[1] - v[2]))
				)
+4096) >> 13; // round and downshift fixed point

		}
#endif

/*		int16_t step(int16_t xs) //class II 
		{
			fixedpoint x = xs;
			v[0] = v[1];
			v[1] = v[2];
			v[2] = v[3];
			v[3] = v[4];
			v[4] = v[5];
			v[5] = ((F2I(D2F(8.851194913973643441e-1) * x)
				   + F2I(D2F(0.78343651405152470169) * v[0])
				   + F2I(D2F(-4.10832301567163593603) * v[1])
				   + F2I(D2F(8.62245120994687574978) * v[2])
				   + F2I(D2F(-9.05358992763851411212) * v[3])
				   + F2I(D2F(4.75602305740711095439) * v[4])));

			printf("x %d v0 %ld v1 %ld v2 %ld v3 %ld v4 %ld v5 %ld\n", x, v[0], v[1], v[2], v[3], v[4], v[5]);
			return 
				 ((v[5] - v[0])
				+5 * (v[1] - v[4])
				+10 * (v[3] - v[2]));
		}
*/
				
		float step(float x) //class II 
		{
			u[0] = u[1];
			u[1] = u[2];
			u[2] = u[3];
			u[3] = u[4];
			u[4] = u[5];
			u[5] = (8.851194913973643441e-1 * x)
				 + (0.78343651405152470169 * u[0])
				 + (-4.10832301567163593603 * u[1])
				 + (8.62245120994687574978 * u[2])
				 + (-9.05358992763851411212 * u[3])
				 + (4.75602305740711095439 * u[4]);
			return 
				 (u[5] - u[0])
				+5 * (u[1] - u[4])
				+10 * (u[3] - u[2]);
		}

};

void write_wav_header(FILE *wav_file, unsigned long num_samples, int s_rate)
{
    unsigned int sample_rate;
    uint16_t num_channels;
    unsigned int bytes_per_sample;
    unsigned int byte_rate;
 
    num_channels = 1;   /* monoaural */
    bytes_per_sample = 2;
 
    if (s_rate<=0) sample_rate = 44100;
    else sample_rate = (unsigned int) s_rate;
 
    byte_rate = sample_rate*num_channels*bytes_per_sample;
 
    /* write RIFF header */
    fwrite("RIFF", 1, 4, wav_file);
	uint32_t tmp = 36 + 2 * num_samples;
	fwrite(&tmp, 4, 1, wav_file); 
    fwrite("WAVE", 1, 4, wav_file);
 
    /* write fmt  subchunk */
    fwrite("fmt ", 1, 4, wav_file);
	tmp = 16; // SubChunk1Size is 16
	fwrite(&tmp, 4, 1, wav_file);
	tmp = 1; // PCM is format 1
	fwrite(&tmp, 2, 1, wav_file);
	fwrite(&num_channels, 2, 1, wav_file);
	fwrite(&sample_rate, 4, 1, wav_file);
	fwrite(&byte_rate, 4, 1, wav_file);
	tmp = num_channels * bytes_per_sample; // block align
	fwrite(&tmp, 2, 1, wav_file);
	tmp = 8*bytes_per_sample; //bits/sample
	fwrite(&tmp, 2, 1, wav_file);

    /* write data subchunk */
    fwrite("data", 1, 4, wav_file);
	tmp = bytes_per_sample * num_samples * num_channels;
	fwrite(&tmp, 1, 4, wav_file);
}

void append_sample(FILE *f, uint16_t sample)
{
	fwrite(&sample, 2, 1, f);
}


int main()
{
	FILE *in = fopen("out", "r");
	FILE *out = fopen("debug.wav", "w");
	write_wav_header(out,  -1, 12500);
	int16_t *buf = (int16_t *)malloc((1<<21) + 2);
	int number_of_samples = fread(buf, 1, 1<<20, in);
	fclose(in);

	filter flt;
	for (int i = 0; i < number_of_samples / 2; i++) {
		int16_t val = buf[i];
		int16_t oval = flt.step(val);
		float fval = flt.step((float)val);
		printf("oval is %d, float %f\n", oval, fval);
		append_sample(out, oval);
	}
/*	for (unsigned int j = 0; j < (number_of_samples - 1) / 512; j++) {
		int accum_silence = 0;
		int accum_envelope = 0;
		for (unsigned int i = 0; i < 512; i+=2) {
			int16_t val = ((buf[j * 512 + i] & 0xFF) << 8) | (buf[j * 512 + i + 1] & 0xFF);
//			printf("value %d\n", (int)val);
			int32_t rectified;

			rectified = abs((int32_t)val - (int32_t)silence_value);

			accum_silence += val;
			accum_envelope += rectified;

			fprintf(out, "%d\t%d\t%d\t%d\t%d\t%d\n", val, rectified, accum_silence, accum_envelope, silence_value, envelope_value);
		}
		accum_silence /= 512;
		accum_envelope /= 512;
		silence_value = (SILENCE_EMA_WEIGHT * silence_value + accum_silence) / (SILENCE_EMA_WEIGHT + 1);
		envelope_value = accum_envelope;

		printf("Silence val %d, envelope %d, sil accum %d, env accum %d\n", silence_value, envelope_value, accum_silence, accum_envelope);
	}*/
	fclose(out);
}



