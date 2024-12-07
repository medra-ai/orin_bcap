#include "DensoController.hpp"

int main() {
    auto driver = denso_controller::DensoReadWriteDriver();

    for (size_t i = 0; i < 1000; ++i)
    {
        auto arm_mutex = denso_controller::DensoArmMutex(driver);
        arm_mutex.Claim();
    }

    return 0;
}