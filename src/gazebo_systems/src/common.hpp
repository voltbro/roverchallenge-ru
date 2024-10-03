#include <string>
#include <iostream>

class Log {
public:
    Log(const std::string &log_name) {
        std::cout << "[" << log_name << "] \"";
    }

    template <class T>
    Log& operator<<(const T &v) {
        std::cout << v;
        return *this;
    }

    ~Log() {
        operator<<("a") << "";
        std::cout << "\"" << std::endl;
    }
};
