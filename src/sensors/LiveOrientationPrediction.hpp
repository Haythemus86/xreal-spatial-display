#pragma once

#include "sensors/OrientationPredictionEvaluation.hpp"

#include <optional>
#include <string>

namespace xreal::sensors
{

struct LiveOrientationPredictionConfig
{
    OrientationPredictorConfig predictor;
    DelayedOrientationEvaluatorConfig evaluator;
    bool evaluateDelayed{};
};

struct LiveOrientationPredictionInput
{
    bool fusedOrientationValid{};
    AbsoluteOrientation measuredAbsolute;
    RelativeOrientation measuredRelative;
    RecenterReference recenterReference;
    AngularVelocityRadians bodyAngularVelocity;
    std::uint64_t deviceTimestampNanoseconds{};
    std::uint64_t hostTimestampNanoseconds{};
    double elapsedSeconds{};
    std::uint8_t sequence{};
    std::uint64_t recenterGeneration{};
    std::string phase{"live"};
};

class LiveOrientationPredictionSession
{
public:
    explicit LiveOrientationPredictionSession(LiveOrientationPredictionConfig configuration);
    [[nodiscard]] std::optional<OrientationPredictionRecord> consume(
        const LiveOrientationPredictionInput& input);
    void finish() noexcept;
    [[nodiscard]] const OrientationPredictor& predictor() const noexcept;
    [[nodiscard]] const DelayedOrientationEvaluator& evaluator() const noexcept;

private:
    LiveOrientationPredictionConfig configuration_;
    OrientationPredictor predictor_;
    DelayedOrientationEvaluator evaluator_;
};

} // namespace xreal::sensors
