#pragma once

#include <string>

namespace hexaarch::analysis {

class ParetoAnalyzer {
public:
    [[nodiscard]] std::string describe() const;
};

}  // namespace hexaarch::analysis
