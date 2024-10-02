#include <string>
#include <iostream>

class Log {
public:
    Log(const std::string &log_name) {
        std::cout << "[" << log_name << "] \"";
    }

    template <class T>
    Log &operator<<(const T &v) {
        std::cout << v;
        return *this;
    }

    ~Log() {
        std::cout << "\"" << std::endl;
    }
};

#define NAMED_LOG Log(plugin_name)
