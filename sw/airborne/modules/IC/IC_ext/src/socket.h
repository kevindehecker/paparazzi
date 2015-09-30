#ifndef SOCKET_H
#define SOCKET_H

#include <unistd.h>             /*  for ssize_t data type  */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <mutex>
#include <thread>

/*  Global constants  */
#define MAX_LINE           (1000)
#define LISTENQ        (1024)   /*  Backlog for listen()   */
#define TCPPORT 6969
struct ICDataPackage {
    int avgdisp_gt;
    int frameID;
    int avgdisp_est;
    int avgdisp_est_thresh;
    int ROCchoice; // 0 for stereo, 1 for estimator. ROC choice
    float fps;
    char endl;             // endl fix, makes it worker nicer in terminal for debugging :)
};
extern struct ICDataPackage video_impl;

/*
 * This class will interface with to the IC pprz module over tcp/ip
 *
 */
class Socket{
private:
    /*  Global variables  */
    int       list_s;                /*  listening socket          */
    int       conn_s;                /*  connection socket         */
    struct    sockaddr_in servaddr;  /*  socket address structure  */
    char      buffer[MAX_LINE];      /*  character buffer          */
    char     *endptr;                /*  for strtol()              */
    std::thread thread_comm;
    std::mutex g_lockComm;
    bool connectionAccepted;
    unsigned char *key;
    bool *cams_are_running;
    bool closeThreads;


    /*  Function declarations  */
    int initSocket(unsigned int port) ;
    bool Read_socket(char * c, size_t maxlen);
    bool Write_socket(char * c, size_t n) ;
    int closeSocket(void);
    int shutdownSocket(void);
    void commInThread();
    void commOutThread();

public:
    /*  Global variables  */
    int commdata_frameID;
    int commdata_gt;
    int commdata_est;
    int commdata_est_thresh;    
    int commdata_Choice;
    float commdata_fps;

    /*  Function declarations  */
    void Close();
    void Init(unsigned char *key, bool *cams_are_running);
    void Unlock();


};
#endif  /*  SOCKET_H  */

