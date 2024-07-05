// client.cpp
#include <zmq.hpp>
#include <string>
#include <iostream>

int main() {
    // Prepare our context and socket
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::req);
    socket.connect("tcp://localhost:5511");

    for (int request_nbr = 0; request_nbr != 10; request_nbr++) {
        // Create a message to send
        std::string msg = "Hello";
        zmq::message_t request(msg.size());
        memcpy(request.data(), msg.data(), msg.size());
        std::cout << "Sending Hello " << request_nbr << "..." << std::endl;
        socket.send(request, zmq::send_flags::none);

        // Get the reply
        zmq::message_t reply;
        socket.recv(reply, zmq::recv_flags::none);
        std::string reply_msg = reply.to_string();
        std::cout << "Received " << reply_msg << " " << request_nbr << std::endl;
    }
    return 0;
}
