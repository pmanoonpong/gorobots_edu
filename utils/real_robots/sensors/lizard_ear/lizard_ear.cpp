/*
 * lizard_ear.cpp
 *
 *  Created on: Jun 23, 2015
 *      Author: danny
 */

//#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
//#include <linux/i2c-dev.h>
//#include <pigpio.h>
//#include <ncurses.h>
#include <signal.h>
#include <math.h>

#include "lizard_ear.h"


using namespace std;

class adc
{
public:
	void getSamples(unsigned int *);
};

void adc::getSamples(unsigned int *)
{
	;
}

/*
class lizard_ear
{

private:
	// Samplerate = 50 KHz
	//double bz_C[4] = {1.12149800000000e-008,-3.13962940619566e-008,3.00757214285979e-008,-9.87041934581002e-009};
	//double bz_I[4] = {3.25260651745651e-24,2.31950732009048e-10,-1.31371090623241e-11,-2.18814137699633e-10};
	//double az_C[4] = {-3.67744663463145,5.21122916122862,-3.36870319684106,0.839136745082569};
	//double az_I[4] = {-3.67744663463145,5.21122916122862,-3.36870319684106,0.839136745082569};

	// Samplerate = 25 KHz
	struct parameters
	{
		double bz_C[4] = {	2.24299600000000e-08,
					-5.38772873816963e-08,
					4.81592835718768e-08,
					-1.60257404375767e-08};

		double bz_I[4] = {	-2.16840434497101e-24,
					1.67969865952935e-09,
					-1.83529255409265e-10,
					-1.49628743224228e-09};

		double az_C[4] = {	-3.10115542810494,
					4.05873039241399,
					-2.60229347594167,
					0.704150476947769};

		double az_I[4] = {	-3.10115542810494,
					4.05873039241399,
					-2.60229347594167,
					0.704150476947769};
	} params_13mm;

	double xL[4] = {0}, xR[4] = {0};
	double yLC[4] = {0}, yLI[4] = {0}, yRC[4] = {0}, yRI[4] = {0};

public:
	double out_LC[N_SAMPLES_USED], out_RC[N_SAMPLES_USED];
	double out_LI[N_SAMPLES_USED], out_RI[N_SAMPLES_USED];
	double sumL, sumR;

public:
	lizard_ear();

	~lizard_ear();

	void filter(double *, double *);
};
*/

lizard_ear::lizard_ear()
{

}

lizard_ear::~lizard_ear()
{

}

void lizard_ear::filter(std::vector<double> signalLeft, std::vector<double> signalRight)
{
	unsigned int i;

        std::vector<double>::iterator left = signalLeft.begin();
        std::vector<double>::iterator right = signalRight.begin();
	sumL = sumR = 0;
	xL[0] = xR[0] = 0;
	xL[1] = xR[1] = 0;
	xL[2] = xR[2] = 0;
	xL[3] = xR[3] = 0;
	yLC[0] = yLI[0] = yRC[0] = yRI[0] = 0;
	yLC[1] = yLI[1] = yRC[1] = yRI[1] = 0;
	yLC[2] = yLI[2] = yRC[2] = yRI[2] = 0;
	yLC[3] = yLI[3] = yRC[3] = yRI[3] = 0;

	// Filter the signals through the C and I filters
	for (i = 0; i < N_SAMPLES_USED; i++)
	{
		// Load the current input sample in the shift register "x"
		xL[0] = *left;
		xR[0] = *right;

		// Do convolution
		out_LC[i] =	xL[0]*params_13mm.bz_C[0] + xL[1]*params_13mm.bz_C[1] +
				xL[2]*params_13mm.bz_C[2] + xL[3]*params_13mm.bz_C[3] -
				yLC[0]*params_13mm.az_C[0] - yLC[1]*params_13mm.az_C[1] -
				yLC[2]*params_13mm.az_C[2] - yLC[3]*params_13mm.az_C[3];

		out_LI[i] =	xL[0]*params_13mm.bz_I[0] + xL[1]*params_13mm.bz_I[1] +
				xL[2]*params_13mm.bz_I[2] + xL[3]*params_13mm.bz_I[3] -
				yLI[0]*params_13mm.az_I[0] - yLI[1]*params_13mm.az_I[1] -
				yLI[2]*params_13mm.az_I[2] - yLI[3]*params_13mm.az_I[3];

		out_RC[i] =	xR[0]*params_13mm.bz_C[0] + xR[1]*params_13mm.bz_C[1] +
				xR[2]*params_13mm.bz_C[2] + xR[3]*params_13mm.bz_C[3] -
				yRC[0]*params_13mm.az_C[0] - yRC[1]*params_13mm.az_C[1] -
				yRC[2]*params_13mm.az_C[2] - yRC[3]*params_13mm.az_C[3];

		out_RI[i] =	xR[0]*params_13mm.bz_I[0] + xR[1]*params_13mm.bz_I[1] +
				xR[2]*params_13mm.bz_I[2] + xR[3]*params_13mm.bz_I[3] -
				yRI[0]*params_13mm.az_I[0] - yRI[1]*params_13mm.az_I[1] -
				yRI[2]*params_13mm.az_I[2] - yRI[3]*params_13mm.az_I[3];

		// Shift contents of shift registers to the right by one element
		xL[3] = xL[2];
		xL[2] = xL[1];
		xL[1] = xL[0];

		xR[3] = xR[2];
		xR[2] = xR[1];
		xR[1] = xR[0];

		yLC[3] = yLC[2];
		yLC[2] = yLC[1];
		yLC[1] = yLC[0];

		yLI[3] = yLI[2];
		yLI[2] = yLI[1];
		yLI[1] = yLI[0];

		yRC[3] = yRC[2];
		yRC[2] = yRC[1];
		yRC[1] = yRC[0];

		yRI[3] = yRI[2];
		yRI[2] = yRI[1];
		yRI[1] = yRI[0];

		// Load the current output sample in the shift register "y"
		yLC[0] = out_LC[i];
		yLI[0] = out_LI[i];
		yRC[0] = out_RC[i];
		yRI[0] = out_RI[i];

		sumL = sumL + fabs(out_LI[i] + out_RC[i]);
		sumR = sumR + fabs(out_RI[i] + out_LC[i]);

		// Increment pointer
		left++;
		right++;
	}
}

/*
int main (int argc, char * argv[])
{
	int i2c_handle;

	unsigned char channel_no, run_filter = 0;
	char rxbuf[BUF_SIZE] = {0}, cfg;

	unsigned int raw_data_ch1[N_SAMPLES_PER_CHANNEL], raw_data_ch2[N_SAMPLES_PER_CHANNEL], raw_data_ch3[N_SAMPLES_PER_CHANNEL], raw_data_ch4[N_SAMPLES_PER_CHANNEL];
	unsigned int i,j,k,l,bytecnt;

	double sine_L[N_SAMPLES_USED], sine_R[N_SAMPLES_USED];

	lizard_ear ear;

	// Initialise and set up ncurses window
	initscr();
	raw();
	noecho();
	nodelay(stdscr, TRUE);

	// Initialise pigpio library
	if (gpioInitialise() < 0)
	{
		mvprintw(0,0,"GPIO init failed.");
		exit(1);
	}

	// Open I2C device
	i2c_handle = i2cOpen(1, PMODAD2_I2C_ADDRESS, 0);
	if (i2c_handle < 0)
	{
		mvprintw(0,0,"Cannot open I2C device (handle = %u)",i2c_handle);
		exit(1);
	}

	// Set ADC configuration byte to read all 4 analogue channels
	cfg = ALL_CHANNELS | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;

	//cfg = CHANNEL_1 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;
	//cfg = CHANNEL_2 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;
	//cfg = CHANNEL_3 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;
	//cfg = CHANNEL_4 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;

	//cfg = CHANNEL_1 | CHANNEL_2 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;
	//cfg = CHANNEL_1 | CHANNEL_3 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;
	//cfg = CHANNEL_1 | CHANNEL_4 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;

	//cfg = CHANNEL_2 | CHANNEL_3 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;
	//cfg = CHANNEL_2 | CHANNEL_4 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;

	//cfg = CHANNEL_3 | CHANNEL_4 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;

	//cfg = CHANNEL_1 | CHANNEL_2 | CHANNEL_3 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;
	//cfg = CHANNEL_1 | CHANNEL_2 | CHANNEL_4 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;
	//cfg = CHANNEL_2 | CHANNEL_3 | CHANNEL_4 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;
	//cfg = CHANNEL_3 | CHANNEL_4 | CHANNEL_1 | AD7991_VREF_VCC | AD7991_FLT_ON | AD7991_BTD_ON | AD7991_SD_OFF;


	// Write the configuration byte into the ADC
	i2cWriteDevice(i2c_handle, &cfg, 1);

	// Sleep for a while...
	usleep(100000);


	while (1) {

	// Ctrl+C handler
	if (getch() == 3)
	{
		endwin();
		i2cClose(i2c_handle);
		gpioTerminate();
		system("stty sane");
		exit(0);
	}

	// Read data samples from all 4 channels
	bytecnt = i2cReadDevice(i2c_handle, rxbuf, BUF_SIZE);

	if (bytecnt == BUF_SIZE)
	{
		mvprintw(0,0,"bytecnt = %d",bytecnt);
			for (i = 0, j = 0; i < BUF_SIZE; i+=2)
		{
			channel_no = (rxbuf[i] & 0x30) >> 4;
			if ((channel_no == 0))
			{
				raw_data_ch1[j++] = (unsigned int)((rxbuf[i] & 0x0F)<<8) + (unsigned int)rxbuf[i+1];
			}
		}
			for (i = 0, j = 0; i < BUF_SIZE; i+=2)
		{
			channel_no = (rxbuf[i] & 0x30) >> 4;
			if ((channel_no == 1))
			{
				raw_data_ch2[j++] = (unsigned int)((rxbuf[i] & 0x0F)<<8) + (unsigned int)rxbuf[i+1];
			}
		}

		run_filter = 1;
	}
	else
	{
		mvprintw(0,0,"bytecnt = %d",bytecnt);
		run_filter = 0;
	}


	if (run_filter)
	{
		int ch = getch();
		if (ch == 3)
		{
			endwin();
			i2cClose(i2c_handle);
			gpioTerminate();
			system("stty sane");
			exit(0);
		}

		for (i = 0, j = 0; i < N_SAMPLES_USED; i++, j++)
		{
			sine_L[j] = (double)(raw_data_ch1[i]-800)/1900;
			sine_R[j] = (double)(raw_data_ch2[i]-800)/1900;
			//mvprintw(j+1,0,"%e\t%e",sine_L[j],sine_R[j]);
		}

		ear.filter(sine_L, sine_R);

		mvprintw(1,0,"sumL = %f\nsumR = %f\ndiff= %f\n",20*log10(ear.sumL),20*log10(ear.sumR),20*(log10(ear.sumL)-log10(ear.sumR)));
		refresh();

	}
	} // end of while (1)...

	endwin();
	i2cClose(i2c_handle);
	gpioTerminate();
	system("stty sane");

	return 0;
}*/
