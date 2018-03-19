#include <linkbot/linkbot.hpp>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#ifndef _WIN32
#include <unistd.h>
#endif

int main() {
    std::cout << "Enter test robot ID:\n";
    std::string id;
    std::getline(std::cin, id);
    barobo::Linkbot l{id};

    std::cout << "Testing accelerometer: \n";
    double x, y, z;
    l.getAccelerometer(x, y, z);
    std::cout << x << " " << y << " " << z << "\n";

    /* FIXME
    std::cout << "Testing battery voltage: \n";
    l.getBatteryVoltage(x);
    std::cout << x << "\n";
    */

    std::cout << "Testing getJointAngles(): \n";
    int timestamp;
    l.getJointAngles(timestamp, x, y, z);
    std::cout << timestamp << " " << x << " " << y << " " << z << "\n";

    std::cout << "Testing getJointSpeeds(): \n";
    l.getJointSpeeds(x, y, z);
    std::cout << x << " " << y << " " << z << "\n";

    std::cout << "Testing getJointStates(): \n";
    LinkbotJointState states[3];
    l.getJointStates(timestamp, states[0], states[1], states[2]);
    std::cout << timestamp << " " << states[0] << " " << states[1] << " " << states[2] << "\n";

    std::cout << "Testing getLedColor(): \n";
    int r, g, b;
    l.getLedColor(r, g, b);
    std::cout << r << " " << g << " " << b << "\n";

    std::cout << "Testing getVersionString():\n";
    std::string v;
    l.getVersionString(v);
    std::cout << v << "\n";

    // SETTERS

    std::cout << "Testing setBuzzerFrequency():\n";
    l.setBuzzerFrequency(440);
    std::this_thread::sleep_for( std::chrono::seconds(1));
    l.setBuzzerFrequency(880);
    std::this_thread::sleep_for( std::chrono::seconds(1));
    l.setBuzzerFrequency(0);

    std::cout << "Setting accelerations to 10 deg/s/s...\n";
    l.setJointSpeeds(0x07, 90, 90, 90);
    l.setJointAccelI(0x07, 10, 10, 10);
    l.setJointAccelF(0x07, 10, 10, 10);
    l.moveSmooth(0x07, 0x07, 360, 360, 360);
    l.moveWait(0x07);
    std::cout << "Setting accelerations to 90 deg/s/s...\n";
    l.setJointAccelI(0x07, 90, 90, 90);
    l.setJointAccelF(0x07, 90, 90, 90);
    l.moveSmooth(0x07, 0x07, 360, 360, 360);
    l.moveWait(0x07);

    std::cout << "Resseting to zero...\n";
    l.resetEncoderRevs();
    l.moveTo(0x07, 0, 0, 0);
    l.moveWait(0x07);

    std::cout << "Testing PID controller...\n";
    l.driveTo(0x07, 30, 30, 30);
    l.moveWait(0x07);
    l.driveTo(0x07, 60, 60, 60);
    l.moveWait(0x07);
    l.driveTo(0x07, 90, 90, 90);
    l.moveWait(0x07);
    l.drive(0x07, -30, -30, -30);
    l.moveWait(0x07);
    l.drive(0x07, -30, -30, -30);
    l.moveWait(0x07);
    l.drive(0x07, -30, -30, -30);
    l.moveWait(0x07);

    l.setLedColor(255, 255, 0);
    std::cout << "Setting button handler... Try presing some buttons, press 'Enter' to continue.\n";
    l.setButtonEventCallback( [] (LinkbotButton button, LinkbotButtonState state, int timestamp) {
            std::cout << "Button callback. Button: " << button << " State: " << state << "\n";
            });
    std::cin.ignore();
    l.setButtonEventCallback(nullptr);

    std::cout << "Setting encoder handler... Try moving the motors, press 'Enter' to continue.\n";
    l.setEncoderEventCallback( [] (int jointNo, double angle, int timestamp) {
            std::cout << "Encoder callback. Joint: " << jointNo << ", angle: " << angle << "timestamp: " << timestamp << "\n";
            }, 5.0);
    std::cin.ignore();
    l.setEncoderEventCallback(nullptr, 0);

    std::cout << "Setting accelerometer handler... Try moving the robot around. Press 'Enter' to continue.\n";
    l.setAccelerometerEventCallback( [] (double x, double y, double z, int timestamp) {
            std::cout << "Accel callback: " << x << " " << y << " " << z << " " << timestamp << "\n";
            });
    std::cin.ignore();
    l.setAccelerometerEventCallback(nullptr);
}
