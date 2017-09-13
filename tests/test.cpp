#include <linkbot/linkbot.hpp>
#include <iostream>
#include <string>

int main() {
    std::cout << "Enter test robot ID:\n";
    std::string id;
    std::getline(std::cin, id);
    barobo::Linkbot l{id};
    l.setLedColor(255, 255, 0);
}
