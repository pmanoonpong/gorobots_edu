#ifndef DELIMITED_FILE_READER_H
#define DELIMITED_FILE_READER_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <sstream>

using namespace std;

class DelimitedFileReader {

public:

    DelimitedFileReader();

    static void split( vector<string>&, string, char );
    static void split( vector<double>&, string, char );
    static void read( string, char, vector<vector<string>>&, unsigned int = 0 );
    static void read( string, char, vector<vector<double>>&, unsigned int = 0 );

protected:


};

#endif
