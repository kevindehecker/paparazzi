#ifndef SOCKET_H
#define SOCKET_H

#include <unistd.h>             /*  for ssize_t data type  */

#define LISTENQ        (1024)   /*  Backlog for listen()   */


/*  Function declarations  */
	int initSocket(unsigned int port) ;
	ssize_t Readline_socket(void *vptr, size_t maxlen);
	ssize_t Writeline_socket(char * text, size_t n);
	int closeSocket(void);

#endif  /*  SOCKET_H  */

