/************************************************************************
* \brief: genetic programming, solving the problem of prime factori-    *
*         zation. This is a minimum environment to allow genetic        *
*         programming based on machine language including crossover.    *
*																		                                    *
* (c) copyright by Joern Fischer											                    *
*                                                                       *
*  This source may be distributed freely without any warranty.          *
*  If it is used to produce data for publication of any kind            *
*  please cite any of my genetic programming papers related to this.    *
*																		                                    *
* @autor: Prof.Dr.Joern Fischer											                    *
* @email: j.fischer@hs-mannheim.de										                  *
*                                                                       *
* @file : TinyVM.h                                                      *
*************************************************************************/

#ifndef TINY_VM                                                                                               
#define TINY_VM   
#include <math.h>

#define CMD_MASK  0x000F    // 0000000000001111 CMD_MASK                                                                                    
#define IDX0_MASK 0x00F0    // 0000000011110000 IDX0_MASK [R0..R15]                                                                                   
#define IDX1_MASK 0x0F00    // 0000111100000000 IDX1_MASK [R0..R15]  
#define IDX2_MASK 0xF000    // 1111000000000000 IDX2_MASK [R0..R15]
#define NUMBER_MASK  0xFFF0 // 1111111111110000 JMP_MASK  

#define SUBROUTINE_SIZE 64
#define PROG_LENGTH 1024
#define STACKSIZE 10000

char COMMAND[16][15]={"LOAD    ","MOVE    ","MUL     ","DIV     ","SUB     ","ADD     ","EQU     ","RET     ","JIH     ","JSR     ","SQRT    ","IS_PRIME","MOD     ","PUSH     ","POP     ","NOP     "};  

enum commands{                                                                                                
	LOAD,
    MOVE,					// MOVE Rx,Ry     -> moves the contents of Register Ry to the register Rx         
    MUL,                    // MUL Rx,Ry -> Rx = Ry * Rz                                                      
    DIV,                    // DIV Rx,Ry -> Rx = Ry / Rz                                                      
    SUB,                    // SUB Rx,Ry -> Rx = Ry - Rz                                                      
    ADD,                    // ADD Rx,Ry -> Rx = Ry + Rz                                                      
    EQU,                    // EQU Rx,Ry -> Rx = Ry == Rz                                                     
    RET,                    // RET       -> returns to the (adress on the stack)+1                            
    JIH,                    // JIH jmpAdress -> if (R0>0) then jumps to the jmpAdress                        
    JSR,                    // JSR jmpAdress -> puts the returnadress on the stack and jumps to the jmpAdress 
    SQRT,                   // Rx = SQRT(Ry)
	IS_PRIME,               // if (Ry==prime) -> Rx=1 else Rx=0
	MOD,					// Rx=Ry % Rz
	PUSH,					// Rx->stack 
	POP                     // stack->Rx
};                                                                                                            

class TinyVM                                                                                                  
{                                                                                                             
public:              
	bool debug;
    unsigned short memory[PROG_LENGTH];                                                                              
    unsigned short programCounter;                                                                            
    long int reg[32];                                                                                   
    short stack[STACKSIZE];    
	long int regStack[STACKSIZE];
	unsigned short regStackPtr;
    unsigned short stackptr;                                                                                  
    bool endOfProg;      
    // --- OpcodeVars                                                                                         
    short cmd; // command                                                                            
    unsigned char idx0,idx1,idx2; // number of register [R0..R15]                                                 
    unsigned short number; // adress to jump into   
	long int primeA,primeB;
	int cycle;
	double sqrtHelp;
	long int R0,R1;
	double fitness;
	unsigned short opcode;
public:       
	/**********************************************************************
	* \brief: constructor with initialization
	**********************************************************************/
    TinyVM()
	{
		fitness = 0;
		debug = false;
	}
	/**********************************************************************
	* \brief: This function simulates the program generated by the GP 
	*         algorithm
	**********************************************************************/
    int simulate(){  
		cycle = 0;
        endOfProg = false;                                                                                    
        stackptr  = 0;                                                                                        
        programCounter = 0; // starts at adress 0  
		regStackPtr = 0;

		for (int t=0;t<8;t++){
			reg[t] = primeA*primeB;
			reg[t+8] = t-4; //-4,-3,-2,-1,0,1,2,3
		}

        do{                                                                                                                                                                                
			opcode = memory[programCounter];
			cmd       = (opcode & CMD_MASK);                                                      
			idx0      = (opcode & IDX0_MASK) >> 4;                                               
			idx1      = (opcode & IDX1_MASK) >> 8; 
			idx2      = (opcode & IDX2_MASK) >> 12;
			number    = (opcode & NUMBER_MASK) >> 4;   

		   if (debug){
			   if (cmd==0 || (cmd>6 && cmd<10)){
				   printf("%5d %s %5d\n",programCounter,COMMAND[cmd],number);
			   }
			   else{
					printf("%5d %s R[%d],R[%d],R[%d]      %d, %d, %d\n",programCounter,COMMAND[cmd], idx0,idx1,idx2, reg[idx0],reg[idx1],reg[idx2]);
			   }
			   if (cycle%100==99){
				   int var;
				   scanf("%d\n",&var);
			   }
		   }			
			
			switch(cmd){   
				case LOAD: {R0 = reg[0]    = number;                     programCounter++;  break;}  
				case MOVE: {R0 = reg[idx0] = reg[idx1];                  programCounter++;  break;}                        
				case MUL:  {R0 = reg[idx0] = reg[idx1] * reg[idx2];      programCounter++; break;}                        
				case DIV:  {if (reg[idx2]>0) R0 = reg[idx0] = reg[idx1] / reg[idx2];      programCounter++; break;}                        
				case SUB:  {R0 = reg[idx0] = reg[idx1] - reg[idx2];      programCounter++; break;}                        
				case ADD:  {R0 = reg[idx0] = reg[idx1] + reg[idx2];      programCounter++; break;}                        
				case EQU:  {R0 = reg[idx0] = reg[idx1] == reg[idx2];     programCounter++; break;}                        
				case RET:  {if (stackptr<1){                                                      
								endOfProg=true;
								return false; // program ends...no prime found
							}
							programCounter = stack[--stackptr]+1;                                                                      
							break; }                                                               
				case JIH:   {if (reg[0]>0) {programCounter = (number & 63) + (programCounter & 0x3c0); }break; }   
				case JSR:   {stack[stackptr++] = programCounter;   programCounter = (number & 0x3c0); break;}//   64er subroutines 0011 1100 0000
				case SQRT:  {R0 = reg[idx0] = (long int)sqrt((double)reg[idx1]);  programCounter++; break;}                        
				case IS_PRIME: {R0 = reg[idx0] = 1; //is a prime
								sqrtHelp = sqrt((double)reg[idx1]);
								R1 = reg[idx1];
								for (int t=2; t<sqrtHelp; t++){
									if ( R1 % t == 0){
										R0 = reg[idx0] = 0;//is not a prime
										break;
									}
								}                              
								programCounter++; 
								break; 
							   }     
				case MOD:  {if (reg[idx2]>0)
							   R0 = reg[idx0] = reg[idx1] % reg[idx2];     programCounter++; break;}                        
				case PUSH: {regStack[(regStackPtr++)%STACKSIZE] = reg[idx0]; programCounter++; break;}
			    case POP:  {reg[idx0] = regStack[(STACKSIZE+(--regStackPtr))%STACKSIZE]; programCounter++; break;}
				default: programCounter++;
			}  

			programCounter%=PROG_LENGTH;
			cycle++;
			if (R0 == primeA || R0 == primeB){
				// factorization sucessfull
				if (debug) printf("Found Number = %d\n",reg[idx0]);
				fitness++;

				return cycle; // prime found
			}
		}while(endOfProg==false && cycle<10000);  
		return 0; // prime not found
    }                                                                                                                                                                                                                    
};                                                                                                            
                                                                                                              
#endif // TINY_VM     
