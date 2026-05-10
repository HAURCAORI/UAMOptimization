#pragma once

#include <string>

namespace hexaarch::optimization {

class MooRunner {
public:
    [[nodiscard]] std::string describe() const;
};

}  // namespace hexaarch::optimization
