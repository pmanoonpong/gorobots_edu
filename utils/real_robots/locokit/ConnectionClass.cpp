/*
 * ConnectionClass.cpp
 *
 *  Created on: Mar 28, 2014
 *      Author: Bassel Zeidan
 */


#include "ConnectionClass.h"

ConnectionClass::ConnectionClass(){//1211
	socksize = sizeof(struct sockaddr_in);
	is_server = false;
	close_connection = false;
}

ConnectionClass::~ConnectionClass(){
	close(connected_socket_descripter);
	close(mysocket);
	printf("Clean socket...\n");
}

/*
void ConnectionClass::define_server(short int port_number) {
	is_server = true;
	memset(&serv, 0, sizeof(serv));
	serv.sin_family = AF_INET;
	serv.sin_addr.s_addr = htonl(INADDR_ANY);
	serv.sin_port = htons(port_number);
	mysocket = socket(AF_INET, SOCK_STREAM, 0);
	bind(mysocket, (struct sockaddr *)&serv, sizeof(struct sockaddr));
	connected = true;
	if (listen(mysocket, 1) == -1) {
		connected = false;
		printf("Error: listen(), can't listen on the socket...\n");
	}
	connected_socket_descripter = accept(mysocket, (struct sockaddr *)&dest, &socksize);
	if (connected_socket_descripter == -1) {
		connected = false;
		printf("Error: accept(), can't connect to the socket and establish the connection...\n");
	}

}
*/

void ConnectionClass::define_client(const char* server_address, short int port_number) {//"192.168.2.4"
	mysocket = socket(AF_INET, SOCK_STREAM, 0); //use tcp protocol
	memset(&dest, 0, sizeof(dest));                /* zero the struct */
	dest.sin_family = AF_INET;
	dest.sin_addr.s_addr = inet_addr(server_address); //htonl(INADDR_LOOPBACK);/* set destination IP number - localhost, 127.0.0.1*/
	dest.sin_port = htons(port_number);                /* set destination port number */
	connected = true;
	int client_connection = connect(mysocket, (struct sockaddr *)&dest, sizeof(struct sockaddr));
	if (client_connection == -1) {
		connected = false;
		printf("Error: client - connect(), can't connect to the socket and establish the connection...\n");
	}
}

int ConnectionClass::send_command(int length, void* buffer) {
	int rec_res;

	/*if (is_server)
		rec_res = send(connected_socket_descripter, buffer, length, 0);
	else*/
	rec_res = send(mysocket, buffer, length, 0);

	if (rec_res == -1) {
		printf("Warning: send(), a message hasn't been sent...\n");
		return -1;
	}
	return 0;
}

int ConnectionClass::receive_command(int length, char** buffer) {
	//char buffer[sizeof(int)];
	int rec_res;
	rec_res = recv(mysocket, *buffer, length, 0);
	if (rec_res == -1) {
		printf("Warning: receive(), an error while receiving a message...\n");
		return -1;
	} else if (rec_res == 0) {
		printf("Warning: connection is closed...\n");
		return 0;
	}
	return rec_res;
}
