#include <iostream>
#include <chrono>
#include <beautiful_bullet/Simulator.hpp>
#include <beautiful_bullet/graphics/MagnumGraphics.hpp>
#include <zmq_stream/Subscriber.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

using namespace std::chrono_literals;


struct FrankaModel : public beautiful_bullet::bodies::MultiBody {
public:
    FrankaModel() : beautiful_bullet::bodies::MultiBody("models/franka/panda.urdf"), _frame("panda_joint_8"), _reference(pinocchio::LOCAL_WORLD_ALIGNED) {}

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q)
    {
        return static_cast<beautiful_bullet::bodies::MultiBody*>(this)->jacobian(q, _frame, _reference);
    }

    Eigen::MatrixXd jacobianDerivative(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
    {
        return static_cast<beautiful_bullet::bodies::MultiBody*>(this)->jacobianDerivative(q, dq, _frame, _reference);
    }

    Eigen::Matrix<double, 6, 1> framePose(const Eigen::VectorXd& q)
    {
        return static_cast<beautiful_bullet::bodies::MultiBody*>(this)->framePose(q, _frame);
    }

    Eigen::Matrix<double, 6, 1> frameVelocity(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
    {
        return static_cast<beautiful_bullet::bodies::MultiBody*>(this)->frameVelocity(q, dq, _frame, _reference);
    }

    std::string _frame;
    pinocchio::ReferenceFrame _reference;
};

Eigen::VectorXd jsonToEigenVector(const nlohmann::json& jsonArray) {
    // Check if the JSON is an array
    if (!jsonArray.is_array()) {
        throw std::invalid_argument("JSON data is not an array");
    }

    // Get the size of the array
    size_t size = jsonArray.size();

    // Create an Eigen vector of appropriate size
    Eigen::VectorXd eigenVector(size);

    // Assign values from JSON array to Eigen vector
    for (size_t i = 0; i < size; ++i) {
        if (!jsonArray[i].is_number()) {
            throw std::invalid_argument("JSON array contains non-numeric values");
        }
        eigenVector[i] = jsonArray[i].get<double>();
    }

    return eigenVector;
}

Eigen::VectorXd getJointConfiguration(zmq::socket_t& subscriber){
    zmq::message_t message;
    auto start = std::chrono::steady_clock::now();
    subscriber.recv(&message);
    std::string received_message(static_cast<char*>(message.data()), message.size());

    nlohmann::json data = nlohmann::json::parse(received_message);
    Eigen::VectorXd vector = jsonToEigenVector(data);
    return vector;
}



int main(int argc, char const* argv[])
{
    std::ifstream f("config/environment1.json");
    nlohmann::json data = nlohmann::json::parse(f);

    zmq::context_t context(1);
    zmq::socket_t subscriber(context , ZMQ_SUB);

    subscriber.connect("tcp://localhost:5511");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    beautiful_bullet::Simulator simulator;
    simulator.setGraphics(std::make_unique<beautiful_bullet::graphics::MagnumGraphics>());
    simulator.addGround();
    
    // Adding obstacles
    beautiful_bullet::bodies::SphereParams paramsSphere;
    paramsSphere.setRadius(0.02).setMass(0.).setColor("red");
    std::cout << data["obstacles"].size() << std::endl;
    size_t nb_obstacles = data["obstacles"].size();
    for (size_t i = 0; i < nb_obstacles; ++i){
        Eigen::VectorXd obstacle = jsonToEigenVector(data["obstacles"][i]); 
        beautiful_bullet::bodies::RigidBodyPtr sphere = std::make_shared<beautiful_bullet::bodies::RigidBody>("sphere", paramsSphere);
        sphere->setPosition(obstacle[0], obstacle[1], obstacle[2]);
        simulator.add(sphere);

    }

    auto franka = std::make_shared<FrankaModel>();
    (*franka).activateGravity();
    simulator.add(static_cast<beautiful_bullet::bodies::MultiBodyPtr>(franka));
    // set robot initial configuration
    Eigen::VectorXd initial_state = jsonToEigenVector(data["initial_configuration_rad"]); 
    (*franka).setState(initial_state);

    simulator.initGraphics();
    size_t desiredFPS = data["fps"];
    simulator.graphics().setDesiredFPS(desiredFPS);

    double t = 0.0, dt = 1e-3, T= 10.0;
    auto next = std::chrono::steady_clock::now();
    auto prev = next - 1ms;


    while (t <= T) {
        simulator.step(t);

        Eigen::VectorXd state = getJointConfiguration(subscriber);
        (*franka).setState(state);

        t += dt;
    }

    return 0;
}