#ifndef SERVER_H
#define SERVER_H

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <vector>
#include <iostream>
#include "client.h"
#include "Frame.h"
#include "System.h"
#define PORT 8080

class Client; 
class Server{
public:
    Server(const string &strSettingsFile, ORB_SLAM2::System *sys);
    Server(); 
    void StartListening(); 
    void Listening(); 
    void Close(); 
    bool CheckNewFrame();
    ORB_SLAM2::Frame* GetNewFrame();
    void InsertFrame(ORB_SLAM2::Frame *pF);

public:
	struct sockaddr_in address;
	int addrlen = sizeof(address);

    int client_num, max_client_num;
	int opt;
    bool listenFlag; 
	char* hello;
    
    int server_fd, new_socket, valread;

    std::thread listen_thread_; 

    std::vector<Client*> clients; 

    std::mutex mMutexServer;

    std::list<ORB_SLAM2::Frame*> mlNewFrames;

    string settingFile; 

    ORB_SLAM2::System *system; 
}; 

#endif