#include "DelimitedFileReader.h"

/**
 * Splits delimiter separated data and packages it into a vector.
 *
 * @param separated [out] contains the separated input
 * @param data [in]
 * @param delimiter the character separating the data elements
 */
void DelimitedFileReader::split( vector<string>& separated, string data, char delimiter ) {

    stringstream ss;

    for( unsigned int i = 0; i < data.size(); i++ ) {

        if( data[i] != delimiter ) {

            ss << data[i];

        } else {

            separated.push_back( ss.str() );
            ss.str( std::string() );

        }

    }

    separated.push_back( ss.str() );

}

/**
 * Splits delimiter separated data and packages it into a vector.
 *
 * @param separated [out] contains the separated input
 * @param data [in]
 * @param delimiter the character separating the data elements
 */
void DelimitedFileReader::split( vector<double>& separated, string data, char delimiter ) {

    stringstream ss;

    for( unsigned int i = 0; i < data.size(); i++ ) {

        if( data[i] != delimiter ) {

            ss << data[i];

        } else {

            separated.push_back( atof( ss.str().c_str() ) );
            ss.str( std::string() );

        }

    }

    separated.push_back( atof( ss.str().c_str() ) );

}

/**
 * Reads in delimited data as string from a file.
 */
void DelimitedFileReader::read (
    string path,
    char delimiter,
    vector<vector<string>> &out,
    unsigned int numberOfHeaderLines ) {

    ifstream file;
    file.open( path.c_str(), ifstream::in );

    string line;
    if ( file.is_open() ) {

        unsigned int i = 1;
        while ( getline ( file, line ) ) {

            if ( i > numberOfHeaderLines ) {

                vector<string> stemp;
                if( line.size() > 0 ) DelimitedFileReader::split( stemp, line, delimiter );

                out.push_back( stemp );

            }

            i++;

        }

        file.close();

    } else {

        cout << "Unable to open file.";

    }

}

/**
 * Reads in delimited data as double precision number from a file.
 */
void DelimitedFileReader::read (
    string path,
    char delimiter,
    vector<vector<double>> &out,
    unsigned int numberOfHeaderLines ) {

    ifstream file;
    file.open( path.c_str(), ifstream::in );

    string line;
    if ( file.is_open() ) {

        unsigned int i = 1;
        while ( getline ( file, line ) ) {

            if ( i > numberOfHeaderLines ) {

                vector<double> dtemp;
                if( line.size() > 0 ) DelimitedFileReader::split( dtemp, line, delimiter );

                out.push_back( dtemp );

            }

            i++;

        }

        file.close();

    } else {

        cout << "Unable to open file.";

    }

}
