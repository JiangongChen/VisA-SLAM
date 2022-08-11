// server, responsible for listening to clients (smartphones)
// support multiple clients, generate a new thread for each client

#include "server.h"

Server::Server():
    client_num(0), max_client_num(1), opt(1), listenFlag(true){
    // initialize the server
    //hello = "Hello from server";

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0))
        == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET,
                SO_REUSEADDR | SO_REUSEPORT, &opt,
                sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr*)&address,
            sizeof(address))
        < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

}

Server::Server(const string &strSettingsFile, ORB_SLAM2::System *sys):
    client_num(0), max_client_num(2), opt(1), listenFlag(true){
    // initialize the server
    //hello = "Hello from server";
    settingFile = strSettingsFile; 

    system = sys; 

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0))
        == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET,
                SO_REUSEADDR | SO_REUSEPORT, &opt,
                sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr*)&address,
            sizeof(address))
        < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

}

void Server::StartListening(){
    std::cout << "server starts listening to new clients." << std::endl; 

    // start listening  
    listen_thread_ = std::thread(&Server::Listening, this);
}

void Server::Listening(){
    while(listenFlag){
        if (listen(server_fd, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }
        new_socket
            = accept(server_fd, (struct sockaddr*)&address,
                    (socklen_t*)&addrlen);
        if(client_num>=max_client_num) {
            // closing the connected socket
            close(new_socket);
            break;
        }
        // client id starts from 0 
        // add new tracking thread in SLAM system
        system->AddClient(client_num); 
        Client* client = new Client(client_num,new_socket,settingFile,this);
        clients.push_back(client);
        std::cout << "client " << client_num << "has been accepted" << std::endl; 
        client_num++;
    }
    // closing the listening socket
    shutdown(server_fd, SHUT_RDWR);
    std::cout << "server listening thread has been stopped." << std::endl; 
    listenFlag = false; 
}

void Server::Close(){
    std::cout << "request for stopping the server." << std::endl; 
    listenFlag = false; 
    listen_thread_.join();

    //wait for all clients to be closed
    for (Client* client : clients)
        client->Close(); 
    std::cout << "server has been successfully closed." << std::endl; 
}

void Server::InsertFrame(ORB_SLAM2::Frame *pF)
{
    unique_lock<mutex> lock(mMutexServer);
    mlNewFrames.push_back(pF);
}

bool Server::CheckNewFrame()
{
    unique_lock<mutex> lock(mMutexServer);
    return(!mlNewFrames.empty());
}

ORB_SLAM2::Frame* Server::GetNewFrame()
{
    unique_lock<mutex> lock(mMutexServer);
    ORB_SLAM2::Frame* mpCurrentFrame = mlNewFrames.front();
    mlNewFrames.pop_front();
    return mpCurrentFrame; 
}
