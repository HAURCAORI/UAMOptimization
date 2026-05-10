#pragma once

#include <string>

namespace hexaarch::optimization {

class SooRunner {
public:
    [[nodiscard]] std::string describe() const;
};

}  // namespace hexaarch::optimization
