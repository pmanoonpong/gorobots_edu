/*
 * ngnet.h
 *
 *  Created on: Jun 15, 2011
 *      Author: poramate
 *      Updated by Bassel Zeidan DEC 10 2013
 */

#ifndef NGNET_H_
#define NGNET_H_

//#include <iostream>
//#include <fstream>
//#include <cstdlib>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <fstream>


using namespace std;

#define NONE  0
#define WRAP  1


//#define IN    4 //= XDIM  //.e.g, 6 pre_initial BUT here use only 2 arrays = 2 inputs
//#define OUT   2 //= UDIM //.e.g, 2 pre_initial BUT here use only 1 array  = 1 output
//#define UNITNUM 1000000 //pre_initial



/*class Unit
{
public:
	int ni,no;
	double* cx;
	double ac;
	double softac;
	double* ivar;
	double* gx;
	double* w;
	double* dw;

	Unit(int in_num, int out_num) {
		cx = new double [in_num];
		ivar = new double [in_num];
		gx = new double [in_num];
		w = new double [out_num];
		dw = new double [out_num];
	}

	Unit(){}
};

class Cell
{
public:
	int fstf;
	Unit* cell;

	Cell(int unit_num, int in_num, int out_num) {
		cell = new Unit[unit_num];
		for (int i=0; i<unit_num; i++)
			cell[i](in_num, out_num);
	}

	Cell() {}
};*/

struct Unit{
	int ni,no;
	double* cx; /* centers of basis*/
	double ac;
	double softac;
	double* ivar;
	double* gx;
	double* w;
	double* dw;

	Unit(){

	}

	void set_Unit(int IN, int OUT){
		cx = new double [IN];
		ivar = new double [IN];
		gx = new double [IN];

		w = new double [OUT];
		dw = new double [OUT];
	}

	char* get_bytes_buffer(int IN, int OUT, size_t &size) {
		//char buffer [(IN*3) + (OUT*2)];
		static vector<double> buffer_double;
		buffer_double.clear();

		for (int i=0; i<IN; i++)
			buffer_double.push_back(cx[i]);
		for (int i=0; i<IN; i++)
			buffer_double.push_back(ivar[i]);
		for (int i=0; i<IN; i++)
			buffer_double.push_back(gx[i]);


		//std::ofstream myfile;
		//myfile.open ("Ws.txt", std::fstream::app);
		for (int i=0; i<OUT; i++) {
		  //  myfile << w[i]<<", ";

			buffer_double.push_back(w[i]);
		}
		//myfile <<"\n";
		//myfile.close();
		std::cout<<std::endl;

		for (int i=0; i<OUT; i++)
			buffer_double.push_back(dw[i]);

		buffer_double.push_back((double)ni);
		buffer_double.push_back((double)no);

		size = buffer_double.size()*sizeof(double);

		//char res [size];
		//memcpy(&res, buffer.data(), size);

		return (char*) &buffer_double.data()[0];
	}

	void read_unit(char* byte_buffer, int IN, int OUT){
		double* pr = (double*) &byte_buffer[0];

		for (int i=0; i<IN; i++) {
			this->cx[i] = pr[0];
			pr++;
		}

		for (int i=0; i<IN; i++) {
			this->ivar[i] = pr[0];
			pr++;
		}

		for (int i=0; i<IN; i++) {
			this->gx[i] = pr[0];
			pr++;
		}


		for (int i=0; i<OUT; i++) {
			this->w[i] = pr[0];
			pr++;
		}

		for (int i=0; i<OUT; i++) {
			this->dw[i] = pr[0];
			pr++;
		}

		this->ni = (int)pr[0];
		this->no = (int)pr[1];
	}
};

struct Cell{
	int fstf;
	Unit* cell;

	Cell(int UNITNUM, int IN, int OUT) {
		cell = new Unit[UNITNUM];
		for (int i=0; i<UNITNUM; i++)
			cell[i].set_Unit(IN, OUT);
	}

	char* get_bytes_buffer(int UNITNUM, int IN, int OUT, size_t &size) {
		static vector<char> buffer;
		buffer.clear();
		int nsize = 0;
		for (int i=0; i<UNITNUM; i++) {
			size_t size_temp;
			char* buffer_temp = cell[i].get_bytes_buffer(IN, OUT, size_temp);
			std::cout<<"cell["<<i<<"] is written and size ="<<size_temp<<std::endl;
			for (int j=0; j<size_temp; j++)
				buffer.push_back(buffer_temp[j]);
			nsize += size_temp;
		}
		size = nsize;
		return buffer.data();
	}

	void write_to_file(int UNITNUM, int IN, int OUT) {
		size_t size ;
		char* final_buffer = this->get_bytes_buffer(UNITNUM, IN, OUT, size);
		std::cout<<"size la la la = "<<size<<std::endl;
		FILE * pFile;
		pFile = fopen ("RBF_Network_actor_critic.bin", "wb");
		std::cout<<"file size ="<<size<<std::endl;
		fwrite(final_buffer, sizeof(char), size, pFile); //(buffer, sizeof(char) , pFile);

		fclose (pFile);
	}

	void read_file(int UNITNUM, int IN, int OUT) {
		FILE * pFile;
		long lSize;
		char * buffer;
		size_t result;

		pFile = fopen ( "RBF_Network_actor_critic.bin" , "rb" );
		if (pFile==NULL) {fputs ("File error",stderr); exit (1);}

		fseek (pFile , 0 , SEEK_END);
		lSize = ftell (pFile);
		std::cout<<"size ="<<lSize<<std::endl;
		rewind (pFile);

		buffer = (char*) malloc (sizeof(char)*lSize);
		if (buffer == NULL) {fputs ("Memory error",stderr); exit (2);}

		result = fread (buffer,1,lSize,pFile);
		if (result != lSize) {fputs ("Reading error",stderr); exit (3);}

		std::cout<<"size ="<<lSize<<std::endl;
		fclose (pFile);




		//Cell c(UNITNUM, IN, OUT);

		char* pr = buffer;

		for (int i=0; i<UNITNUM; i++) {
			//std::cout<<"U["<<i<<"] is loaded"<<std::endl;
			this->cell[i].read_unit(pr, IN, OUT);
			pr += ((3*IN)+(2*OUT)+2)*sizeof(double);
		}

	}

	void print_weights(string file_name, int UNITNUM, int OUT) {
		ofstream critic_weights_file;
		ofstream actor_weights_file;
		critic_weights_file.open("critic_" + file_name, std::fstream::app);
		actor_weights_file.open("actor_" + file_name, std::fstream::app);
		for (int i=0; i<UNITNUM; i++){
			critic_weights_file<<cell[i].w[0];
			actor_weights_file<<cell[i].w[1];
			if (i != (UNITNUM - 1)) {
				critic_weights_file<<" ";
				actor_weights_file<<" ";
			} else {
				critic_weights_file<<"\r\n";
				actor_weights_file<<"\r\n";
			}
		}
		critic_weights_file.close();
		actor_weights_file.close();

	}
};

/*
typedef struct defunit{
	int ni,no;
	double cx[IN];
	double ac;
	double softac;
	double ivar[IN];
	double gx[IN];
	double w[OUT];
	double dw[OUT];
}Unit,*unit;

typedef struct defcell{
	int fstf;
	Unit cell[UNITNUM];
}Cell;
*/

class NGNet{
public:

	//NGNet(int i, int j);

	int IN;
	int OUT;
	//Initial RBF
	double active_thresh;
	void put_incsbox(Cell *isb, int ni, int no, double *xp, double *Ivr, int *nc); ///????
	int init_incsbox(Cell *isb,int ni, int no);
	int reset_incsbox(Cell *isb);

	//Calculate value function
	void incsbox_output(Cell *isb, double *x, double *y,int *nc);
	//Calculate neural activation with softmax
	double incsbox_activate( Cell *isb, double *x, int *nc);
	//Calculate neural activation without softmax
	void incsbox_unit(Unit *cunit, double *x);
	//Calculate update value function trace
	void incsbox_trace(Cell *isb, double *x, double lambda, int *nc);
	//Calculate update weight of value function--> Learning mechanism of critic
	double incsbox_update( Cell *isb, double *x, double *error, double rate, int *nc,
			double *Ivr, double Thr ,double near);
	double incsbox_update_actor( Cell *isb, double *x, double* exp, double *error, double rate, int *nc,
				double *Ivr, double Thr ,double near);
	double incsbox_update_v_action_pairs(
					Cell *isb,
					double *x,
					double exp,
					double error/*-TDerror*/,
					double rate_v,
					double rate_actor,
					int *nc,
					double *Ivr,
					double Thr /*e.g. 0.1*/ ,
					double near /*e.g. 0.6*/,
					bool update_actor
					);

	NGNet(int ni, int no);

	void write_RBF_to_file(int Units_number, int IN, int out, Cell* C);

	//void write_to_file(Cell* C);
	//void get_RBF_network(Cell *c);

	////// Used function
//	extern double active_thresh;
//	extern int init_incsbox(Cell *isb,int ni, int no);
//	extern int reset_incsbox(Cell *isb);
//	extern void incsbox_output( Cell *isb, double *x, double *y,int *nc);
//	extern double incsbox_activate( Cell *isb, double *x,int *nc);
//	extern void incsbox_unit(Unit *cunit, double *x);
//	extern void incsbox_trace(Cell *isb, double *x, double lambda, int *nc);
//	extern double incsbox_update( Cell *isb, double *x, double *error, double rate, int *nc,
//			       double *Ivr, double Thr, double near);
	//////






//_________////// UNUsed function___________________________________________________________________//

//	extern void incsbox_predict( Cell *isb,double *y,double suma,int *nc); NOT exit!
//	extern int reset_incsbox_trace(Cell *isb, int *nc);
//	extern void incsbox_jacobian( Cell *isb, double *x,double **jac,int *nc);
//	extern double **realloc_matrix( double **ptr, int row, int column);
//	extern void incsbox_trace_elig(Cell *isb, double *x, double *coeff, double lambda, int *nc);

//	extern int save_center(FILE* fp, Cell* isb, int *unum);
//	extern int save_incsbox(FILE* fp, Cell* isb, int *unum);
//	extern int load_incsbox(FILE* fp, Cell* isb, int *unum);
//	extern double incsbox_target( Cell *isb, double *x, double *yt, double rate, int *nc,
//				      double *Ivr, double Thr, double near);
//	extern void plot_func(Cell *isb, int *nc, char *filename);
//	extern int save_incsbox(FILE* fp, Cell* isb,int *unum);
//	extern int load_incsbox(FILE* fp, Cell* isb, int *unum);



};

#endif /* NGNET_H_ */
