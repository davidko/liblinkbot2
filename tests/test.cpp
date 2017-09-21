#include <linkbot/linkbot.hpp>
#include <iostream>
#include <string>

int main() {
    std::cout << "Enter test robot ID:\n";
    std::string id;
    std::getline(std::cin, id);
    barobo::Linkbot l{id};
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
}
