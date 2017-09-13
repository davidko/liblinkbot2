#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>

#include <linkbot/linkbot.hpp>


namespace barobo {

int _connect_n = 0;

std::string daemonHostName () {
    return "127.0.0.1";
}

uint16_t daemonPortNo () {
    return 42000;
}

Linkbot::Linkbot (const std::string& serialId) {
    m = rs::linkbotFromSerialId(serialId.c_str());
}

Linkbot::Linkbot() {
    // Get the serial id from the environment
    auto env_str = std::getenv("ROBOTMANAGER_IDS");
    if(!env_str) {
        throw Error("Environment variable ROBOTMANAGER_IDS not set.");
    }
    std::string env{env_str};
    std::istringstream ss{env};
    std::string token;
    _connect_n++;
    for(int i = 0; i < _connect_n; i++) {
        std::getline(ss, token, ',');
    }
    if ( token.length() != 4 ) {
        throw Error("Insufficient number of robots connected in robot manager.");
    }
    m = rs::linkbotFromSerialId(token.c_str());
}

Linkbot::~Linkbot() {
    rs::linkbotDelete(m);
}

void Linkbot::setLedColor(int r, int g, int b) {
    rs::linkbotSetLedColor(m, r, g, b);
}

/* GETTERS */
// Member functions take angles in degrees.
// All functions are non-blocking. Use moveWait() to wait for non-blocking
// movement functions.
void Linkbot::getAccelerometer (int& timestamp, double&, double&, double&){
    throw std::exception();
}
std::vector<int> Linkbot::getAdcRaw(){
    throw std::exception();
}
void Linkbot::getBatteryVoltage(double& voltage){
    throw std::exception();
}
void Linkbot::getFormFactor(LinkbotFormFactor& form){
    throw std::exception();
}
void Linkbot::getJointAngles (int& timestamp, double&, double&, double&){
    throw std::exception();
}
void Linkbot::getJointSpeeds(double&, double&, double&){
    throw std::exception();
}
void Linkbot::getJointStates(int& timestamp,
                    LinkbotJointState& s1,
                    LinkbotJointState& s2,
                    LinkbotJointState& s3){
    throw std::exception();
}
void Linkbot::getLedColor (int&, int&, int&){
    throw std::exception();
}
void Linkbot::getVersionString (std::string& v){
    throw std::exception();
}
void Linkbot::getSerialId(std::string& serialId){
    throw std::exception();
}
void Linkbot::getJointSafetyThresholds(int&, int&, int&){
    throw std::exception();
}
void Linkbot::getJointSafetyAngles(double&, double&, double&){
    throw std::exception();
}

/* SETTERS */
void Linkbot::resetEncoderRevs(){
    throw std::exception();
}
void Linkbot::setBuzzerFrequency (double){
    throw std::exception();
}
void Linkbot::setJointAccelI(int mask, double, double, double){
    throw std::exception();
}
void Linkbot::setJointAccelF(int mask, double, double, double){
    throw std::exception();
}
void Linkbot::setJointSpeeds (int mask, double, double, double){
    throw std::exception();
}
void Linkbot::setJointStates(
    int mask,
    LinkbotJointState s1, double d1,
    LinkbotJointState s2, double d2,
    LinkbotJointState s3, double d3){
    throw std::exception();
}
void Linkbot::setJointStates(
    int mask,
    LinkbotJointState s1, double d1, double timeout1, LinkbotJointState end1,
    LinkbotJointState s2, double d2, double timeout2, LinkbotJointState end2,
    LinkbotJointState s3, double d3, double timeout3, LinkbotJointState end3
    ){
    throw std::exception();
}

void Linkbot::setJointSafetyThresholds(int mask, int t1, int t2, int t3){
    throw std::exception();
}
void Linkbot::setJointSafetyAngles(int mask, double t1, double t2, double t3){
    throw std::exception();
}

/* MOVEMENT */
// Member functions take angles in degrees.
// All functions are non-blocking. Use moveWait() to wait for non-blocking
// movement functions.
void Linkbot::drive (int mask, double, double, double){
    throw std::exception();
}
void Linkbot::driveTo (int mask, double, double, double){
    throw std::exception();
}
void Linkbot::move (int mask, double, double, double){
    throw std::exception();
}
// moveContinuous takes three angular speed coefficients. Use -1 to move
// a motor backward, +1 to move it forward.
void Linkbot::moveAccel(int mask, int relativeMask,
    double omega0_i, double timeout0, LinkbotJointState endstate0,
    double omega1_i, double timeout1, LinkbotJointState endstate1,
    double omega2_i, double timeout2, LinkbotJointState endstate2){
    throw std::exception();
}
void Linkbot::moveContinuous (int mask, double, double, double){
    throw std::exception();
}
void Linkbot::moveTo (int mask, double, double, double){
    throw std::exception();
}
void Linkbot::moveSmooth(int mask, int relativeMask, double a0, double a1, double a2){
    throw std::exception();
}
void Linkbot::moveWait(int mask){
    throw std::exception();
}
void Linkbot::motorPower(int mask, int m1, int m2, int m3){
    throw std::exception();
}
void Linkbot::stop (int mask){
    throw std::exception();
}

// Passing a null pointer as the first parameter of those three functions
// will disable its respective events.
void Linkbot::setButtonEventCallback (LinkbotButtonEventCallback, void* userData){
    throw std::exception();
}
// cb function params: button, button state, timestamp(millis)
void Linkbot::setButtonEventCallback (std::function<void(LinkbotButton, LinkbotButtonState, int)>){
    throw std::exception();
}

void Linkbot::setEncoderEventCallback (LinkbotEncoderEventCallback, double granularity, void* userData){
    throw std::exception();
}
// cb function params: joint number, angle, timestamp
void Linkbot::setEncoderEventCallback (std::function<void(int, double, int)>, double granularity){
    throw std::exception();
}

void Linkbot::setAccelerometerEventCallback (LinkbotAccelerometerEventCallback, void* userData){
    throw std::exception();
}
// cb function params: x, y, z, timestamp
void Linkbot::setAccelerometerEventCallback (std::function<void(double, double, double, int)>){
    throw std::exception();
}

void Linkbot::setJointEventCallback(LinkbotJointEventCallback, void* userData){
    throw std::exception();
}
void Linkbot::setJointEventCallback(std::function<void(int, LinkbotJointState, int)>){
    throw std::exception();
}

void Linkbot::setConnectionTerminatedCallback (LinkbotConnectionTerminatedCallback, void* userData){
    throw std::exception();
}

/* MISC */
void Linkbot::writeEeprom(uint32_t address, const uint8_t* data, size_t size){
    throw std::exception();
}
void Linkbot::readEeprom(uint32_t address, size_t recvsize, uint8_t* buffer){
    throw std::exception();
}
void Linkbot::writeTwi(uint32_t address, const uint8_t* data, size_t size){
    throw std::exception();
}
void Linkbot::readTwi(uint32_t address, size_t recvsize, uint8_t* buffer){
    throw std::exception();
}
void Linkbot::writeReadTwi(
    uint32_t address,
    const uint8_t* sendbuf,
    size_t sendsize,
    uint8_t* recvbuf,
    size_t recvsize){
    throw std::exception();
}
void Linkbot::setPeripheralResetMask(int mask, int resetMask){
    throw std::exception();
}
} // namespace barobo
