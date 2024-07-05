/*
    This file is part of beautiful-bullet.

    Copyright (c) 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

// Simulator
#include <beautiful_bullet/Simulator.hpp>

// Graphics
#include <beautiful_bullet/graphics/MagnumGraphics.hpp>

// Spaces
#include <control_lib/spatial/SE.hpp>
#include <control_lib/spatial/SO.hpp>

// Controllers
#include <control_lib/controllers/Feedback.hpp>
#include <control_lib/controllers/QuadraticControl.hpp>

// CPP Utils
#include <utils_lib/FileManager.hpp>
#include <utils_lib/Timer.hpp>

// Stream
#include <zmq_stream/Requester.hpp>

// parse yaml
#include <yaml-cpp/yaml.h>

#include <nlohmann/json.hpp>

#include <chrono>
#include <iostream>
#include <thread>

using namespace beautiful_bullet;
using namespace control_lib;
using namespace utils_lib;
using namespace std::chrono;
using namespace zmq_stream;

using R3 = spatial::R<3>;
using R7 = spatial::R<7>;
using SE3 = spatial::SE<3>;
using SO3 = spatial::SO<3, true>;

struct ParamsConfig {
    struct controller : public defaults::controller {
        PARAM_SCALAR(double, dt, 1.0e-2);
    };

    struct feedback : public defaults::feedback {
        PARAM_SCALAR(size_t, d, 7);
    };

    struct quadratic_control : public defaults::quadratic_control {
        // State dimension
        PARAM_SCALAR(size_t, nP, 7);

        // Control/Input dimension (optimization torques)
        PARAM_SCALAR(size_t, nC, 7);

        // Slack variable dimension (optimization slack)
        PARAM_SCALAR(size_t, nS, 6);

        // derivative order (optimization joint acceleration)
        PARAM_SCALAR(size_t, oD, 2);
    };
};

Eigen::VectorXd jsonToEigenVector(const nlohmann::json& jsonArray){
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


struct FrankaModel : public bodies::MultiBody {
public:
    FrankaModel() : bodies::MultiBody("models/franka/panda.urdf"), _frame("panda_joint_8"), _reference(pinocchio::LOCAL_WORLD_ALIGNED) {}

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q)
    {
        return static_cast<bodies::MultiBody*>(this)->jacobian(q, _frame, _reference);
    }

    Eigen::MatrixXd jacobianDerivative(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
    {
        return static_cast<bodies::MultiBody*>(this)->jacobianDerivative(q, dq, _frame, _reference);
    }

    Eigen::Matrix<double, 6, 1> framePose(const Eigen::VectorXd& q)
    {
        return static_cast<bodies::MultiBody*>(this)->framePose(q, _frame);
    }

    Eigen::Matrix<double, 6, 1> frameVelocity(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
    {
        return static_cast<bodies::MultiBody*>(this)->frameVelocity(q, dq, _frame, _reference);
    }

    std::string _frame;
    pinocchio::ReferenceFrame _reference;
};

struct ConfigDynamics : public controllers::AbstractController<ParamsConfig, R7> {
    ConfigDynamics()
    {
        _d = R7::dimension();
        _u.setZero(_d);

        // position ds weights
        Eigen::MatrixXd K = Eigen::MatrixXd::Zero(7, 7), D = Eigen::MatrixXd::Zero(7, 7);
        K.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        D.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

        double k = 3.0, d = 2.0 * std::sqrt(k);
        _pos
            .setStiffness(k*K)
            .setDamping(d*D);

        // external ds stream
        _external = false;

        _context = std::make_unique<zmq::context_t>(1);
        _socket = std::make_unique<zmq::socket_t>(*_context, zmq::socket_type::rep);

        _socket->bind("tcp://*:5511");

    }

    ConfigDynamics& setReference(const R7& q)
    {
        _pos.setReference(q);
        return *this;
    }

    ConfigDynamics& setExternal(const bool& value)
    {
        _external = value;
        return *this;
    }

    Eigen::VectorXd getJointAccleration(){
        zmq::message_t request;

        _socket->recv(request);

        std::string rcv_msg = request.to_string();
        

        std::string recevied_message(static_cast<char*>(request.data()), request.size()); 
        nlohmann::json data = nlohmann::json::parse(recevied_message);
        Eigen::VectorXd vector = jsonToEigenVector(data);
        return vector;
    }

    void sendPositionSpeed(Eigen::VectorXd pos_speed){
        std::vector<char> send_buffer;
        serialize(pos_speed, send_buffer);

        // Send reply back to client
        zmq::message_t reply(send_buffer.size());
        memcpy(reply.data(), send_buffer.data(), send_buffer.size());
        _socket->send(reply, zmq::send_flags::none);

    }
    void serialize(const Eigen::VectorXd& vec, std::vector<char>& buffer) {
        size_t vec_size = vec.size();
        buffer.resize(vec_size * sizeof(double));
        memcpy(buffer.data(), vec.data(), buffer.size());
    }

    const bool& external() { return _external; }

    void update(const R7& q) override
    {
        _u = _external ? getJointAccleration() : _pos(q);
    }

    Eigen::VectorXd getAccelerationTarget(){
        return _u;
    }

protected:
    using AbstractController<ParamsConfig, R7>::_d;
    using AbstractController<ParamsConfig, R7>::_u;

    controllers::Feedback<ParamsConfig, R7> _pos;

    bool _external;

    std::unique_ptr<zmq::context_t> _context;
    std::unique_ptr<zmq::socket_t> _socket;
};



struct QPController : public control::MultiBodyCtr {
    QPController(const std::shared_ptr<FrankaModel>& model, const R7& ref_config, const double q, const double r, const double s)
        : control::MultiBodyCtr(ControlMode::CONFIGURATIONSPACE), _ref_config(ref_config), _model(model), _q(q), _r(r), _s(s)
    {
        R7 curr_state(_model->state());
        // torque reference
        _ref_input = _model->gravityVector(curr_state._x);

        // task ds
        _config
            .setReference(_ref_config)
            .update(curr_state);

        // inverse kinematics
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(7, 7), R = Eigen::MatrixXd::Zero(7, 7), S = Eigen::MatrixXd::Zero(6, 6);
        Q.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        R.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        S.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

        Q*=_q;
        R*=_r;
        S*=_s;

        _qp
            .setModel(_model)
            .stateCost(Q)
            .inputCost(R)
            .inputReference(_ref_input)
            .stateReference(_config.output())
            .slackCost(S)
            .modelConstraint()
            .positionLimits()
            .velocityLimits()
            .accelerationLimits()
            .effortLimits()
            .init(curr_state);

        _output_writer.setFile("outputs/run1-accel-output.csv");
        _input_writer.setFile("outputs/run1-accel-input.csv");
        _torque_writer.setFile("outputs/run1-torque.csv");
        _angle_writer.setFile("outputs/run1-angle-tracking.csv");
        _velocity_writer.setFile("outputs/run1-velocity-tracking.csv");
    }


    Eigen::VectorXd action(bodies::MultiBody& body) override
    {
        // curr
        R7 curr_state(body.state());
        curr_state._v = body.velocity();

        if ((curr_state._x - _ref_config._x).norm() <= 0.05 && !_config.external()){
            _config.setExternal(true);
            std::cout<<"setting to external"<<std::endl;
        }

        _config.update(curr_state);
        _ref_input = _model->gravityVector(curr_state._x); // _ref_input = _model->nonLinearEffects(state._x, state._v);

        Eigen::Matrix<double, 7, 1>tau = _qp(curr_state).segment(7, 7);
        Eigen::Matrix<double, 7, 1>accel = _qp(curr_state).head(7);
        if (_config.external()){
            _output_writer.append(accel);
            _input_writer.append(_config.output());
            _angle_writer.append(curr_state._x);
            _torque_writer.append(tau);
            _velocity_writer.append(curr_state._v);
        }
        
        if (_config.external()){
            Eigen::VectorXd q_dq(curr_state._x.size() + curr_state._v.size());
            q_dq << curr_state._x, curr_state._v;
            _config.sendPositionSpeed(q_dq);
        }
        return tau;

    }

    // reference
    R7 _ref_config;
    double _q, _r, _s;
    // torque reference
    Eigen::Matrix<double, 7, 1> _ref_input;
    // qp controller
    controllers::QuadraticControl<ParamsConfig, FrankaModel> _qp;
    // model
    std::shared_ptr<FrankaModel> _model;
    ConfigDynamics _config;

    FileManager _output_writer;
    FileManager _input_writer;
    FileManager _torque_writer;
    FileManager _angle_writer;
    FileManager _velocity_writer;


    const bool& external() { return _config.external(); }

};

int main(int argc, char const* argv[])
{
    std::ifstream f("config/environment2.json");
    nlohmann::json data = nlohmann::json::parse(f);

    // Create simulator
    Simulator simulator;

    // Add graphics
    simulator.setGraphics(std::make_unique<graphics::MagnumGraphics>());

    // Add ground
    simulator.addGround();

    // Adding obstacles
    beautiful_bullet::bodies::SphereParams paramsSphere;
    paramsSphere.setRadius(0.05).setMass(0.).setColor("red");
    std::cout << data["obstacles"].size() << std::endl;
    size_t nb_obstacles = data["obstacles"].size();
    for (size_t i = 0; i < nb_obstacles; ++i){
        Eigen::VectorXd obstacle = jsonToEigenVector(data["obstacles"][i]); 
        beautiful_bullet::bodies::RigidBodyPtr sphere = std::make_shared<beautiful_bullet::bodies::RigidBody>("sphere", paramsSphere);
        sphere->setPosition(obstacle[0], obstacle[1], obstacle[2]);
        simulator.add(sphere);
    }

    // Multi Bodies
    auto franka = std::make_shared<FrankaModel>();
    Eigen::VectorXd state_ref = (franka->positionUpper() - franka->positionLower()) * 0.5 + franka->positionLower();
    franka->setState(state_ref);

    Eigen::VectorXd initial_config = jsonToEigenVector(data["initial_configuration_rad"]);
    R7 ref_config(initial_config);

    double q = data["Q"];
    double r = data["R"];
    double s = data["S"];

    auto controller = std::make_shared<QPController>(franka, ref_config, q, r, s);

    // Set controlled robot
    (*franka).addControllers(controller);

    // Add robots and run simulation
    simulator.add(static_cast<bodies::MultiBodyPtr>(franka));

    simulator.initGraphics();
    size_t desiredFPS = data["fps"];

    size_t index = 0;
    double t = 0.0, dt = 1e-3, T = 400.0;

    auto next = steady_clock::now();
    auto prev = next - 1ms;

    while (t <= T) {
        auto now = steady_clock::now();
        
        if (!simulator.step(size_t(t/dt)))
            break;

        if (controller->external()){
            simulator.graphics().setDesiredFPS(desiredFPS);
        }

        t += dt;

        prev = now;
        next += 1ms;
        std::this_thread::sleep_until(next);
    }

    return 0;
}