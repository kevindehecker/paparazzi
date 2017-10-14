/**
    C++ client example using sockets
*/
#include<iostream>    //cout
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent

#include <fstream>      // std::ifstream
#include <sstream>

using namespace std;

std::ofstream logger;

#define MAXBUFFERSIZE 65536
unsigned char data1[MAXBUFFERSIZE];
unsigned char data2[MAXBUFFERSIZE];
#define TOTALBUFFERSIZE (MAXBUFFERSIZE + MAXBUFFERSIZE)
#define ESPBUFFERSIZE 2048
struct RAM_log_data {
    int32_t accx;
    int32_t accy;
    int32_t accz;
    int32_t gyrop;
    int32_t gyroq;
    int32_t gyror;
} __attribute__((__packed__));

/**
    TCP Client class
*/
class tcp_client
{
private:
    int sock;
    std::string address;
    int port;
    struct sockaddr_in server;

public:
    tcp_client();
    bool conn(string, int);
    bool send_data(string data);
    ssize_t receive(unsigned char*,int);
};

tcp_client::tcp_client()
{
    sock = -1;
    port = 0;
    address = "";
}

/**
    Connect to a host on a certain port number
*/
bool tcp_client::conn(string address , int port)
{
    //create socket if it is not already created
    if(sock == -1)
    {
        //Create socket
        sock = socket(AF_INET , SOCK_STREAM , 0);
        if (sock == -1)
        {
            perror("Could not create socket");
        }

        cout<<"Socket created\n";
    }
    else    {   /* OK , nothing */  }

    //setup address structure
    if(inet_addr(address.c_str()) == -1)
    {
        struct hostent *he;
        struct in_addr **addr_list;

        //resolve the hostname, its not an ip address
        if ( (he = gethostbyname( address.c_str() ) ) == NULL)
        {
            //gethostbyname failed
            herror("gethostbyname");
            cout<<"Failed to resolve hostname\n";

            return false;
        }

        //Cast the h_addr_list to in_addr , since h_addr_list also has the ip address in long format only
        addr_list = (struct in_addr **) he->h_addr_list;

        for(int i = 0; addr_list[i] != NULL; i++)
        {
            //strcpy(ip , inet_ntoa(*addr_list[i]) );
            server.sin_addr = *addr_list[i];

            cout<<address<<" resolved to "<<inet_ntoa(*addr_list[i])<<endl;

            break;
        }
    }

    //plain ip address
    else
    {
        server.sin_addr.s_addr = inet_addr( address.c_str() );
    }

    server.sin_family = AF_INET;
    server.sin_port = htons( port );

    //Connect to remote server
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
    {
        perror("connect failed. Error");
        return 1;
    }

    cout<<"Connected\n";
    return true;
}

/**
    Send data to the connected host
*/
bool tcp_client::send_data(string data)
{
    //Send some data
    if( send(sock , data.c_str() , strlen( data.c_str() ) , 0) < 0)
    {
        perror("Send failed : ");
        return false;
    }
    cout<<"Data send\n";

    return true;
}

/**
    Receive data from the connected host
*/
ssize_t tcp_client::receive(unsigned char * buffer, int size)
{
    //wait for data
    while (recv(sock, buffer, size, MSG_PEEK) != size);


    //Receive a reply from the server
    return recv(sock , buffer , size , 0) ;

}

int main(int argc , char *argv[])
{    

    tcp_client c;

    //connect to host
    //c.conn("127.0.0.1", 9988);
    c.conn("192.168.4.1", 666);

    logger.open("log.csv",std::ofstream::out);

    struct RAM_log_data * tmp;

    int entry_id1 = 0;
    int entry_id2 = 0;

    int junk_start = (int)MAXBUFFERSIZE / (int)sizeof(struct RAM_log_data);
    int junk_size = MAXBUFFERSIZE - junk_start*sizeof(struct RAM_log_data);

    int totcnt = 0;
    cout << "|--------------------------------------------------|" << std::endl;
    cout << " ";
    //std::cout.setf( std::ios_base::unitbuf );
    int progress_stepsize = TOTALBUFFERSIZE / 50;
    while (entry_id2 < junk_start) {

        struct RAM_log_data * rtmp ;
        int id;
        if ((entry_id1 +1 ) * sizeof(struct RAM_log_data) < MAXBUFFERSIZE) {
            tmp = (struct RAM_log_data * ) data1;
            rtmp = &tmp[entry_id1];
            id = entry_id1;
            entry_id1++;
        } else {
            tmp = (struct RAM_log_data * ) data2;
            rtmp = &tmp[entry_id2];
            id = entry_id2;
            entry_id2++;
        }

        int k = c.receive((unsigned char * )rtmp,sizeof(struct RAM_log_data));
        if (k > 0) {
            totcnt+=k;
            //std::cout << totcnt << " | " << id << ": " << rtmp->accx << ", " << rtmp->accy << ", " << rtmp->accz << ", " << rtmp->gyrop << ", " << rtmp->gyroq << ", " << rtmp->gyror << std::endl;
            logger << rtmp->accx << ", " << rtmp->accy << ", " << rtmp->accz << ", " << rtmp->gyrop << ", " << rtmp->gyroq << ", " << rtmp->gyror << std::endl;
        }

        //there are some bytes in between the two buffers that are not filled:
        if (entry_id1 == junk_start) {
            entry_id1 = 99999999;
            unsigned char junk[junk_size+1];
            //std::cout << "Skipping junk: " << junk_size << std::endl;
            c.receive(junk,junk_size);
            totcnt+=junk_size;
        }

        static int totcnt_prev=0;
        if ( totcnt - totcnt_prev > progress_stepsize ) {
            totcnt_prev = totcnt;
            cout << "*" << std::flush;
        }
    }
    cout << "*" << std::endl;
    logger.close();
    return 0;
}
