#define PMODAD2_I2C_ADDRESS 0x28

#define N_CHANNELS 4
#define N_SAMPLES_PER_CHANNEL 100
#define N_BYTES_PER_SAMPLE 2
#define N_SAMPLES_USED 100
#define BUF_SIZE (N_BYTES_PER_SAMPLE * N_SAMPLES_PER_CHANNEL * N_CHANNELS)


#define AD7991_CH3_ON	(1 << 7)
#define AD7991_CH3_OFF	!(1 << 7)

#define AD7991_CH2_ON	(1 << 6)
#define AD7991_CH2_OFF	!(1 << 6)

#define AD7991_CH1_ON	(1 << 5)
#define AD7991_CH1_OFF	!(1 << 5)

#define AD7991_CH0_ON	(1 << 4)
#define AD7991_CH0_OFF	!(1 << 4)

#define AD7991_VREF_VCC	!(1 << 3)
#define AD7991_VREF_EXT	(1 << 3)

#define AD7991_FLT_ON	!(1 << 2)
#define AD7991_FLT_OFF	(1 << 2)

#define AD7991_BTD_ON	!(1 << 1)
#define AD7991_BTD_OFF	(1 << 1)

#define AD7991_SD_ON 	!(1 << 0)
#define AD7991_SD_OFF 	(1 << 0)

#define ALL_CHANNELS (AD7991_CH3_ON | AD7991_CH2_ON | AD7991_CH1_ON | AD7991_CH0_ON)
#define CHANNEL_1 (AD7991_CH3_OFF | AD7991_CH2_OFF | AD7991_CH1_OFF | AD7991_CH0_ON)
#define CHANNEL_2 (AD7991_CH3_OFF | AD7991_CH2_OFF | AD7991_CH1_ON | AD7991_CH0_OFF)
#define CHANNEL_3 (AD7991_CH3_OFF | AD7991_CH2_ON | AD7991_CH1_OFF | AD7991_CH0_OFF)
#define CHANNEL_4 (AD7991_CH3_ON | AD7991_CH2_OFF | AD7991_CH1_OFF | AD7991_CH0_OFF)

class lizard_ear
{

private:
        // Samplerate = 50 KHz
/*      double bz_C[4] = {1.12149800000000e-008,-3.13962940619566e-008,3.00757214285979e-008,-9.87041934581002e-009};
        double bz_I[4] = {3.25260651745651e-24,2.31950732009048e-10,-1.31371090623241e-11,-2.18814137699633e-10};
        double az_C[4] = {-3.67744663463145,5.21122916122862,-3.36870319684106,0.839136745082569};
        double az_I[4] = {-3.67744663463145,5.21122916122862,-3.36870319684106,0.839136745082569};
*/
        // Samplerate = 25 KHz
        struct parameters
        {
                double bz_C[4] = {      2.24299600000000e-08,
                                        -5.38772873816963e-08,
                                        4.81592835718768e-08,
                                        -1.60257404375767e-08};

                double bz_I[4] = {      -2.16840434497101e-24,
                                        1.67969865952935e-09,
                                        -1.83529255409265e-10,
                                        -1.49628743224228e-09};

                double az_C[4] = {      -3.10115542810494,
                                        4.05873039241399,
                                        -2.60229347594167,
                                        0.704150476947769};

                double az_I[4] = {      -3.10115542810494,
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
