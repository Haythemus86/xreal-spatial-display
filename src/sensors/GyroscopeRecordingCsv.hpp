#pragma once

#include "sensors/GyroscopeBiasCalibration.hpp"
#include "sensors/ImuSample.hpp"

#include <istream>
#include <optional>
#include <string>
#include <vector>

namespace xreal::sensors
{

struct GyroscopeRecordingCsvResult
{
    std::vector<ImuSample> samples;
    std::optional<GyroscopeBias> bias;
    std::string error;
    std::uint64_t lineNumber{};

    [[nodiscard]] bool valid() const noexcept
    {
        return error.empty();
    }
};

[[nodiscard]] GyroscopeRecordingCsvResult parseGyroscopeRecordingCsv(
    std::istream& input);

} // namespace xreal::sensors
