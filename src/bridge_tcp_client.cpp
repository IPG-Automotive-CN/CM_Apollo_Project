#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <thread>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <iostream>

#include <fcntl.h>

extern "C"
{
#include "bridge_tcp_client_c_interface.h"
#include "cyberbridge_C_interface.h"

#include <DataDict.h>
#include <InfoUtils.h>
#include <Log.h>
#include <SimCore.h>
}

#include "bridge_tcp_client.h"

#define PRINT_ERROR  std::cout<<"ERROR:"<<__FILE__<<":"<<__LINE__<<":"<<__func__<<":"
#define PRINT_INFO   std::cout<<"INFO:"<<__FILE__<<":"<<__LINE__<<":"<<__func__<<":"


uint32_t get_32b_little_endian(std::string &buffer, size_t offset)
{
    return buffer[offset + 0] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16) | (buffer[offset + 3] << 24);
}

void append32bLittleEndian(std::string *buffer, uint32_t val)
{
    buffer->push_back( (uint8_t) (val >> 0)  );
    buffer->push_back( (uint8_t) (val >> 8)  );
    buffer->push_back( (uint8_t) (val >> 16) );
    buffer->push_back( (uint8_t) (val >> 24) );
}


static void error(const char *msg)
{
    perror(msg);
    exit(0);
}

typedef int SOCKET_HANDLE;
static volatile SOCKET_HANDLE sockfd = -1;

static volatile char must_run_tcp_client = 0;

#define TCP_BUFFER_IN_SIZE 4096
static unsigned char tcp_buffer_in[TCP_BUFFER_IN_SIZE];


/* receive functions to read the ctrl msgs from the socket */
static void* tcp_client_rcv_thread(void*)
{
    std::cout << "[tcp_client_rcv_thread] TCP read started" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    while (must_run_tcp_client)
    {
        
        if (sockfd >= 0)
        {

    		int tcp_buf_size=0;
//TODO:ipg: code to change the blocking socket to NON Block
//as NON Block socket, when apollo sends no msg, the err msg "would block" comes very often.
//    		const int flags = fcntl(sockfd, F_GETFL, 0);
//    		 fcntl(sockfd, F_SETFL,flags | O_NONBLOCK);
        	    tcp_buf_size = read(sockfd, tcp_buffer_in, TCP_BUFFER_IN_SIZE - 1);

            // std::cout << "[tcp_client_rcv_thread] reading ..."<<tcp_buf_size<<"bytes" << std::endl;
            if (tcp_buf_size <= 0)
            {
/*
        		struct timespec current_time;
        		timespec_get(&current_time, TIME_UTC);
        		u_int64_t current_time_ms = (current_time.tv_sec) * 1000 +
        									(current_time.tv_nsec) / (1000000);*/
                //it is defaultly blocking socket with timeout_rcv
            	 int err=  errno;
            	if (err == EWOULDBLOCK) {
            		// std::cout<<"Error "<<err<<" TCP read would block at "<<current_time_ms/1000<<std::endl;
            	} else {
            	//	std::cout<<"Error "<<err<<" TCP read timed-out at "<<current_time_ms/1000<<std::endl;
            	}
            }
            else
            {
//                PRINT_INFO << "TCP read got data at the time" << "with size " <<tcp_buf_size<<" bytes \n";
                receive_control_data(tcp_buffer_in, tcp_buf_size);
//                PRINT_INFO << "TCP processed at "<< current_time_ms <<" received data "<<tcp_buf_size<<" bytes \n";
            }
        } else {

        	std::cout << "[tcp_client_rcv_thread] the c variable socket < 0" << std::endl;
        }
        // No-sleep, check the socket every cycle, because the ctrl msgs comes in high framerate 100hz
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }   // end of while loop
    
    std::cout << "[tcp_client_rcv_thread] Ending TCP read" << std::endl;
    return nullptr;
}

// TODO: ipg: Do we need a static thread ?
static std::thread tcp_client_rcv_thread_task;

int tcp_client_init()
{
    struct sockaddr_in serv_addr;
    struct hostent *server;
    const int PORT_NUM = 9090;
    must_run_tcp_client = 1;
    
    printf("Opening socket ...\n");

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");

<<<<<<< HEAD

    char* tcp_server_ip = 
        iGetStrOpt(SimCore.TestRig.SimParam.Inf, "Apollo_Server_Ip", "192.168.0.130");
=======
    //You can set the server ip by edit the CarMaker Infofile 
    //which named "SimParameter" and located in "./Data/Config", 
    //usage:  "Apollo_Server_Ip = 192.168.0.130"
    char* tcp_server_ip = 
        iGetStrOpt(SimCore.TestRig.SimParam.Inf, "Apollo_Server_Ip", "localhost");
>>>>>>> 4e3a4bcbe3e88a047ac019ffd778cda1310c3aaf


    printf("Apollo Server Ip = %s\n", tcp_server_ip);
    server = //gethostbyname("localhost");
            gethostbyname(tcp_server_ip); //NOTE: gethostbyname is obsolete ...
    // ... replace by with  getnameinfo or set IP address directly
    // ... (https://stackoverflow.com/questions/6652384/how-to-set-the-ip-address-from-c-in-linux)
    if (server == NULL)
    {
        fprintf(stderr, "ERROR, no such host\n");
        exit(0);
    }
    bzero((char *)&serv_addr, sizeof(serv_addr)); //set zeros in string
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(PORT_NUM);

    struct timeval timeout_rcv;
    timeout_rcv.tv_sec = 1;
    timeout_rcv.tv_usec = 2000;    //TODO: ipg: result timeout_rece is about 1020 ms, different as set timeout 1002ms

    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout_rcv,
                   sizeof(timeout_rcv)) < 0)
        error("setsockopt timeoutrcv failed\n");

    struct timeval timeout_snd;
    timeout_snd.tv_sec = 1;
    timeout_snd.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout_snd,
                   sizeof(timeout_rcv)) < 0)
        error("setsockopt timeoutsnd failed\n");

    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){
        close(sockfd);
        sockfd = -1;
        printf("ERROR connecting to Apollo \n");
        fflush(stdout);
        return -1; //error("ERROR connecting");
    }

    // Start receiving thread
    tcp_client_rcv_thread_task = std::thread(tcp_client_rcv_thread, nullptr);
    return 1; //Success
}


void tcp_send(std::string & data){
    if(sockfd < 0){
        // Make sure that tcp_client_init has been run first
        //PRINT_ERROR<<"Socket is not connected to the apollo server"<<std::endl;
        set_bridge_connection_state(false);
        return;
    }
    int n = write(sockfd, data.data(), data.size());
    if (n < 0){
        //PRINT_ERROR<<"Error sending data"<<std::endl;
        set_bridge_connection_state(false);
    }
}


void tcp_client_stop()
{
    PRINT_INFO<<"Stopping TCP connection"<<std::endl;
    must_run_tcp_client = 0;
    if(sockfd >= 0){
        PRINT_INFO<<"[tcp_client_stop] sockfd is valid"<<std::endl;
        tcp_client_rcv_thread_task.join();
        close(sockfd);
    }
    sockfd = -1;
    PRINT_INFO"<<Ended function"<<std::endl;
<<<<<<< HEAD
}
=======
}
>>>>>>> 4e3a4bcbe3e88a047ac019ffd778cda1310c3aaf
