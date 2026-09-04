#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

struct CocotbEnvironment {
  CocotbEnvironment() {
    if (std::getenv("PYGPI_PYTHON_BIN")) {
      return;
    }

    FILE* command = popen("cocotb-config --python-bin", "r");
    if (!command) {
      return;
    }

    char buffer[4096];
    if (std::fgets(buffer, sizeof(buffer), command)) {
      std::string python_bin(buffer);
      python_bin.erase(python_bin.find_last_not_of("\r\n") + 1);
      setenv("PYGPI_PYTHON_BIN", python_bin.c_str(), 0);
    }
    pclose(command);
  }
};

CocotbEnvironment cocotb_environment;

}  // namespace
