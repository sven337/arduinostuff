/* 
 * udpserver.c - A simple UDP echo server 
 * usage: udpserver <port>
 */

/* 
 *  Using WAV header from:
 *   make_wav.c
 * Creates a WAV file from an array of ints.
 * Output is monophonic, signed 16-bit samples
 * copyright
 * Fri Jun 18 16:36:23 PDT 2010 Kevin Karplus
 * Creative Commons license Attribution-NonCommercial
 *  http://creativecommons.org/licenses/by-nc/3.0/
 */
 
/* information about the WAV file format from
    http://ccrma.stanford.edu/courses/422/projects/WaveFormat/
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define BUFSIZE 2048
const int portno = 45990;

 
void write_wav_header(FILE *wav_file, unsigned long num_samples, int s_rate)
{
    unsigned int sample_rate;
    uint16_t num_channels;
    unsigned int bytes_per_sample;
    unsigned int byte_rate;
    unsigned long i;    /* counter for samples */
 
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

/*
 * error - wrapper for perror
 */
void error(char *msg) {
  perror(msg);
  exit(1);
}

int main(int argc, char **argv) {
  int sockfd; /* socket */
  int clientlen; /* byte size of client's address */
  struct sockaddr_in serveraddr; /* server's addr */
  struct sockaddr_in clientaddr; /* client addr */
  struct hostent *hostp; /* client host info */
  char buf[BUFSIZE]; /* message buf */
  char *hostaddrp; /* dotted decimal host addr string */
  int optval; /* flag value for setsockopt */
  int n; /* message byte size */

  /* 
   * socket: create the parent socket 
   */
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) 
    error("ERROR opening socket");

  /* setsockopt: Handy debugging trick that lets 
   * us rerun the server immediately after we kill it; 
   * otherwise we have to wait about 20 secs. 
   * Eliminates "ERROR on binding: Address already in use" error. 
   */
  optval = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, 
	     (const void *)&optval , sizeof(int));

  /*
   * build the server's Internet address
   */
  bzero((char *) &serveraddr, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
  serveraddr.sin_port = htons((unsigned short)portno);

  /* 
   * bind: associate the parent socket with a port 
   */
  if (bind(sockfd, (struct sockaddr *) &serveraddr, 
	   sizeof(serveraddr)) < 0) 
    error("ERROR on binding");

  /* 
   * main loop: wait for a datagram, then echo it
   */
  clientlen = sizeof(clientaddr);

  FILE *out = fopen("./out.wav", "w");
  write_wav_header(out,  -1, 20000);
  while (1) {

    /*
     * recvfrom: receive a UDP datagram from a client
     */
    bzero(buf, BUFSIZE);
    n = recvfrom(sockfd, buf, BUFSIZE, 0,
		 (struct sockaddr *) &clientaddr, &clientlen);
    if (n < 0)
      error("ERROR in recvfrom");

    /* 
     * gethostbyaddr: determine who sent the datagram
     */
    hostp = NULL; /*gethostbyaddr((const char *)&clientaddr.sin_addr.s_addr, 
			  sizeof(clientaddr.sin_addr.s_addr), AF_INET);*/
    hostaddrp = inet_ntoa(clientaddr.sin_addr);
    if (hostaddrp == NULL)
      error("ERROR on inet_ntoa\n");
    printf("server received datagram from %s (%s)\n", 
	   hostp ? hostp->h_name : "unknown", hostaddrp);

    printf("server received %d/%d bytes: ", strlen(buf), n);
	int i;
	for (i = 0; i < n; i+=2) {
		uint16_t value = ((buf[i+1] & 0x0F) <<8 | (buf[i] &0xFF)) & 0x0FFF;
		printf("%u ", value);
		int16_t v = (value - 0x1000/2) << 4;
		printf("(%d)\n", v);
		append_sample(out, v);
	}
	printf("\n");
	fflush(out);
    
  }
  fclose(out);
}
