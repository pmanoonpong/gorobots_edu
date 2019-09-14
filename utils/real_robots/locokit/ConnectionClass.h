/*
 * ConnectionClass.h
 *
 *  Created on: Mar 28, 2014
 *      Author: bassel Zeidan
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>




class ConnectionClass {
private:
	struct sockaddr_in dest; /* socket info about the machine connecting to us */
	struct sockaddr_in serv; /* socket info about our side */
	int mysocket;            /* socket used to listen for incoming connections */
	int connected_socket_descripter;
	socklen_t socksize;
	bool is_server;

public:
	bool connected;
	bool close_connection;

	ConnectionClass();
	~ConnectionClass();
	//void define_server(short int port_number);
	void define_client(const char* server_address, short int port_number);
	int send_command(int length, void* buffer);
	int receive_command(int length, char** buffer);
};


