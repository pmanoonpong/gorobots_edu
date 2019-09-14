/*
 * amosiisensormotordefinition.h
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 *
 */

#ifndef AMOSIISENSORMOTORDEFINITION_H_
#define AMOSIISENSORMOTORDEFINITION_H_

enum AmosIISensorNames{

        // Angle sensors (for actoric-sensor board (new board))
        TR0_as=0, //Thoracic joint of right front leg
        TR1_as=1, //Thoracic joint of right middle leg
        TR2_as=2, //Thoracic joint of right hind leg

        TL0_as=3, //Thoracic joint of left front leg
        TL1_as=4, //Thoracic joint of left middle leg
        TL2_as=5, //Thoracic joint of left hind leg

        CR0_as=6, //Coxa joint of right front leg
        CR1_as=7, //Coxa joint of right middle leg
        CR2_as=8, //Coxa joint of right hind leg

        CL0_as=9,  //Coxa joint of left hind leg
        CL1_as=10, //Coxa joint of left hind leg
        CL2_as=11, //Coxa joint of left hind leg

        FR0_as=12, //Fibula joint of right front leg
        FR1_as=13, //Fibula joint of right middle leg
        FR2_as=14, //Fibula joint of right hind leg

        FL0_as=15, //Fibula joint of left front leg
        FL1_as=16, //Fibula joint of left middle leg
        FL2_as=17, //Fibula joint of left hind leg

        BJ_as= 18, //Backbone joint angle

        //Foot contact sensors (AMOSII v1 and v2)
        R0_fs= 19, //Right front foot
        R1_fs= 20, //Right middle foot
        R2_fs= 21, //Right hind foot
        L0_fs= 22, //Left front foot
        L1_fs= 23, //Left middle foot
        L2_fs= 24, //Left hind foot

        // US sensors (AMOSII v1 and v2)
        FR_us=25, //Front Ultrasonic sensor (right)
        FL_us=26, //Front Ultrasonic sensor (left)
	
	F_lid_0=122, //lidar
	F_lid_1=123,
	F_lid_2=124,
	F_lid_3=125,
	F_lid_4=126,
	F_lid_5=127,
	F_lid_6=128,
	F_lid_7=129,
	F_lid_8=130,
	F_lid_9=131,
	F_lid_10=132,
	F_lid_11=133,
	F_lid_12=134,
	F_lid_13=135,
	F_lid_14=136,
	F_lid_15=137,
	F_lid_16=138,
	F_lid_17=139,
	F_lid_18=140,
	F_lid_19=141,  
	F_lid_20=142, //lidar
	F_lid_21=143,
	F_lid_22=144,
	F_lid_23=145,
	F_lid_24=146,
	F_lid_25=147,
	F_lid_26=148,
	F_lid_27=149,
	F_lid_28=150,
	F_lid_29=151,
	F_lid_30=152,
	F_lid_31=153,
	F_lid_32=154,
	F_lid_33=155,
	F_lid_34=156,
	F_lid_35=157,
	F_lid_36=158,
	F_lid_37=159,
	F_lid_38=160,
	F_lid_39=161,
	F_lid_40=162, //lidar
	F_lid_41=163,
	F_lid_42=164,
	F_lid_43=165,
	F_lid_44=166,
	F_lid_45=167,
	F_lid_46=168,
	F_lid_47=169,
	F_lid_48=170,
	F_lid_49=171,
	F_lid_50=172,
	F_lid_51=173,
	F_lid_52=174,
	F_lid_53=175,
	F_lid_54=176,
	F_lid_55=177,
	F_lid_56=178,
	F_lid_57=179,
	F_lid_58=180,
	F_lid_59=181,  
	F_lid_60=182, //lidar
	F_lid_61=183,
	F_lid_62=184,
	F_lid_63=185,
	F_lid_64=186,
	F_lid_65=187,
	F_lid_66=188,
	F_lid_67=189,
	F_lid_68=190,
	F_lid_69=191,
	F_lid_70=192,
	F_lid_71=193,
	F_lid_72=194,
	F_lid_73=195,
	F_lid_74=196,
	F_lid_75=197,
	F_lid_76=198,
	F_lid_77=199,
	F_lid_78=200,
	F_lid_79=201,
	F_lid_80=202, //lidar
	F_lid_81=203,
	F_lid_82=204,
	F_lid_83=205,
	F_lid_84=206,
	F_lid_85=207,
	F_lid_86=208,
	F_lid_87=209,
	F_lid_88=210,
	F_lid_89=211,
	F_lid_90=212,
	F_lid_91=213,
	F_lid_92=214,
	F_lid_93=215,
	F_lid_94=216,
	F_lid_95=217,
	F_lid_96=218,
	F_lid_97=219,
	F_lid_98=220,
	F_lid_99=221,
	F_lid_100=222, //lidar
	F_lid_101=223,
	F_lid_102=224,
	F_lid_103=225,
	F_lid_104=226,
	F_lid_105=227,
	F_lid_106=228,
	F_lid_107=229,
	F_lid_108=230,
	F_lid_109=231,
	F_lid_110=232,
	F_lid_111=233,
	F_lid_112=234,
	F_lid_113=235,
	F_lid_114=236,
	F_lid_115=237,
	F_lid_116=238,
	F_lid_117=239,
	F_lid_118=240,
	F_lid_119=241,
	F_lid_120=242, //lidar
	F_lid_121=243,
	F_lid_122=244,
	F_lid_123=245,
	F_lid_124=246,
	F_lid_125=247,
	F_lid_126=248,
	F_lid_127=249,
	F_lid_128=250,
	F_lid_129=251,
	F_lid_130=252,
	F_lid_131=253,
	F_lid_132=254,
	F_lid_133=255,
	F_lid_134=256,
	F_lid_135=257,
	F_lid_136=258,
	F_lid_137=259,
	F_lid_138=260,
	F_lid_139=261,  
	F_lid_140=262, //lidar
	F_lid_141=263,
	F_lid_142=264,
	F_lid_143=265,
	F_lid_144=266,
	F_lid_145=267,
	F_lid_146=268,
	F_lid_147=269,
	F_lid_148=270,
	F_lid_149=271,
	F_lid_150=272,
	F_lid_151=273,
	F_lid_152=274,
	F_lid_153=275,
	F_lid_154=276,
	F_lid_155=277,
	F_lid_156=278,
	F_lid_157=279,
	F_lid_158=280,
	F_lid_159=281,
	F_lid_160=282, //lidar
	F_lid_161=283,
	F_lid_162=284,
	F_lid_163=285,
	F_lid_164=286,
	F_lid_165=287,
	F_lid_166=288,
	F_lid_167=289,
	F_lid_168=290,
	F_lid_169=291,
	F_lid_170=292,
	F_lid_171=293,
	F_lid_172=294,
	F_lid_173=295,
	F_lid_174=296,
	F_lid_175=297,
	F_lid_176=298,
	F_lid_177=299,
	F_lid_178=300,
	F_lid_179=301,
	// IR reflex sensors at legs (AMOSIIv2)
        R0_irs=31,
        R1_irs=29,
        R2_irs=27,
        L0_irs=32,
        L1_irs=30,
        L2_irs=28,

        // Ultrasonic reflex sensors at front, middle and rear legs (AMOSIIv1)
        R0_us= 33,
        R1_us= 34,
        L0_us= 35,
        L1_us= 36,

        // Torque sensors, used as Current sensors at each motor (for actoric-sensor board (new board))
        TR0_ts=37,
        TR1_ts=38,
        TR2_ts=39,

        TL0_ts=40,
        TL1_ts=41,
        TL2_ts=42,

        CR0_ts=43,
        CR1_ts=44,
        CR2_ts=45,

        CL0_ts=46,
        CL1_ts=47,
        CL2_ts=48,

        FR0_ts=49,
        FR1_ts=50,
        FR2_ts=51,

        FL0_ts=52,
        FL1_ts=53,
        FL2_ts=54,

        BJ_ts= 55,

        // 3D Accelerometer (x,y,z) at body (for actoric-sensor board (new board))
        BX_acs= 56,
        BY_acs= 57,
        BZ_acs= 58,

        //photo (light) sensors Left, Middle and Right (AMOSIIv1 and v2)
        L_ps = 59,
        M_ps = 60,
        R_ps = 61,

        // goal orientation sensors (relative position to reference object 1, e.g. camera)
        G0x_s=62,
        G0y_s=63,
        G0z_s=64,

        //average current sensor of motors (ZAP 25)
        A_cs = 65,  //average motor current measurement (amosiiv1 & amosiiv2)
        B_cs = 121, //direct from Battery of only for amosiiv1

        //Body speed sensors (only simulation)
        BX_spd= 66,
        BY_spd= 67,
        BZ_spd= 68,

        //---Adding more sensors**

        // goal orientation sensors (relative angle to reference object 1, e.g. camera)
        G0angleroll_s=69,
        G0anglepitch_s=70,
        G0angleyaw_s=71,

        // goal orientation sensors (relative position to reference object 2, e.g. camera)
        G1x_s=72,
        G1y_s=73,
        G1z_s=74,

        // goal orientation sensors (relative angle to reference object 2, e.g. camera)
        G1angleroll_s=75,
        G1anglepitch_s=76,
        G1angleyaw_s=77,

        // goal orientation sensors (relative position to reference object 3, e.g. camera)
        G2x_s=78,
        G2y_s=79,
        G2z_s=80,

        // goal orientation sensors (relative angle to reference object 3, e.g. camera)
        G2angleroll_s=81,
        G2anglepitch_s=82,
        G2angleyaw_s=83,

        //laser scanner (number of edges found, average height of data points, roughness criterion and minimum/maximum height
        // AMOSIIv2
        LaserNmbEdge_s = 84,
        LaserHeight_s = 85,
        LaserRough_s = 86,
        LaserMaxHeight_s = 87,
        LaserMinHeight_s = 88,

        // AMOSIIv1
        Poti_s = 89,

        //Compass sensors at body (for actoric-sensor board (new board))
        Compassx_s = 90,
        Compassy_s = 91,

        // 3D Accelerometer (x,y,z) at RO
        // on Foot sensor board, AMOSIIv1 and v2
        R0X_acs= 92,
        R0Y_acs= 93,
        R0Z_acs= 94,

        // 3D Accelerometer (x,y,z) at R1
        // on Foot sensor board, AMOSIIv1 and v2
        R1X_acs= 95,
        R1Y_acs= 96,
        R1Z_acs= 97,

        // 3D Accelerometer (x,y,z) at R2
        // on Foot sensor board, AMOSIIv1 and v2
        R2X_acs= 98,
        R2Y_acs= 99,
        R2Z_acs= 100,

        // 3D Accelerometer (x,y,z) at L0
        // on Foot sensor board, AMOSIIv1 and v2
        L0X_acs= 101,
        L0Y_acs= 102,
        L0Z_acs= 103,

        // 3D Accelerometer (x,y,z) at L1
        // on Foot sensor board, AMOSIIv1 and v2
        L1X_acs= 104,
        L1Y_acs= 105,
        L1Z_acs= 106,

        // 3D Accelerometer (x,y,z) at L2
        // on Foot sensor board, AMOSIIv1 and v2
        L2X_acs= 107,
        L2Y_acs= 108,
        L2Z_acs= 109,

        //Microphone sensors (for actoric-sensor board (new board))
        Microphone0_s= 110, // Microphone 0
        Microphone1_s= 111, // Microphone 1
        Microphone2_s= 112, // Microphone 2

        // Inclinometer sensors of the AMOSIIv1 and v2 body
        In_x = 113, //around x axis (forward walking direction)
        In_y = 114, //around y axis (sideward walking direction)

        //Body position sensors (only simulation)
        BX_pos = 115, //(forward walking direction)
        BY_pos = 116, //(sideward walking direction)
        BZ_pos = 117, //(vertical direction)

        //Body orientation sensors (only simulation), comparable to compass of the real robot
        BX_ori = 118, // around x axis
        BY_ori = 119, // around y axis
        BZ_ori = 120, // around z axis

        //Changing according to the maximum sensor number
        AMOSII_SENSOR_MAX = 302,
        dungbeetle_SENSOR_MAX = 302,

};



enum AmosIIMotorNames{
        TR0_m = 0,
        TR1_m = 1,
        TR2_m = 2,// Upward (+), Downward (-)
        TL0_m = 3,
        TL1_m = 4,
        TL2_m = 5,

        CR0_m = 6,
        CR1_m = 7,
        CR2_m = 8,// Upward (+), Downward (-)
        CL0_m = 9,
        CL1_m = 10,
        CL2_m = 11,

        FR0_m = 12,
        FR1_m = 13,
        FR2_m = 14, // Upward (+), Downward (-)
        FL0_m = 15,
        FL1_m = 16,
        FL2_m = 17,

        BJ_m = 18,  // Upward (+), Downward (-)

        //Changing according to the maximum motor number
        AMOSII_MOTOR_MAX = 19,
        dungbeetle_MOTOR_MAX = 19,



};

#endif /* AMOSIISENSORMOTORDEFINITION_H_ */
