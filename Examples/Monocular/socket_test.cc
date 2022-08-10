#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<server.h>
#include<client.h>

int main(int argc, char **argv)
{
    Server* server = new Server(); 

    server->StartListening(); 
    while(server->listenFlag){
        usleep(1e5); 
    }

    // request close of the server
    server->Close(); 
}