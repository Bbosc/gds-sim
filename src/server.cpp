// server.cpp
#include <zmq.hpp>
#include <string>
#include <iostream>
#include <unistd.h>

int main() {
    // Prepare our context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::rep);
    socket.bind("tcp://*:5511");

    while (true) {
        zmq::message_t request;

        // Wait for next request from client
        socket.recv(request, zmq::recv_flags::none);
        std::string rcv_msg = request.to_string();
        std::cout << "Received: " << rcv_msg << std::endl;

        // Do some 'work'
        sleep(1);

        // Send reply back to client
        std::string reply_msg = "World";
        zmq::message_t reply(reply_msg.size());
        memcpy(reply.data(), reply_msg.data(), reply_msg.size());
        socket.send(reply, zmq::send_flags::none);
    }
    return 0;
}
