#include "sensors/LiveOrientationPrediction.hpp"

#include <utility>

namespace xreal::sensors
{

LiveOrientationPredictionSession::LiveOrientationPredictionSession(
    LiveOrientationPredictionConfig configuration)
    : configuration_(std::move(configuration)),
      predictor_(configuration_.predictor),
      evaluator_(configuration_.evaluator)
{
}

std::optional<OrientationPredictionRecord> LiveOrientationPredictionSession::consume(
    const LiveOrientationPredictionInput& input)
{
    if (!input.fusedOrientationValid)
    {
        return std::nullopt;
    }

    std::optional<OrientationPredictionEvaluation> evaluation;
    if (configuration_.evaluateDelayed)
    {
        evaluation = evaluator_.consumeMeasured(
            input.deviceTimestampNanoseconds,
            input.measuredAbsolute,
            input.recenterGeneration);
    }

    const OrientationPredictionResult prediction = predictor_.predict({
        input.measuredAbsolute,
        input.recenterReference,
        input.bodyAngularVelocity,
        input.deviceTimestampNanoseconds,
    });
    if (configuration_.evaluateDelayed && prediction.validity == PredictionValidity::valid)
    {
        const auto horizon = std::chrono::duration_cast<std::chrono::nanoseconds>(
            prediction.horizon.applied);
        evaluator_.enqueue(
            input.deviceTimestampNanoseconds,
            horizon,
            input.measuredAbsolute,
            prediction.absolute,
            input.recenterGeneration);
        if (horizon == std::chrono::nanoseconds::zero())
        {
            evaluation = evaluator_.consumeMeasured(
                input.deviceTimestampNanoseconds,
                input.measuredAbsolute,
                input.recenterGeneration);
        }
    }

    OrientationPredictionRecord record;
    record.deviceTimestampNanoseconds = input.deviceTimestampNanoseconds;
    record.hostTimestampNanoseconds = input.hostTimestampNanoseconds;
    record.elapsedSeconds = input.elapsedSeconds;
    record.sequence = input.sequence;
    record.recenterGeneration = input.recenterGeneration;
    record.phase = input.phase;
    record.mode = configuration_.predictor.mode;
    record.prediction = prediction;
    record.measuredAbsolute = input.measuredAbsolute;
    record.measuredRelative = input.measuredRelative;
    record.delayedEvaluation = evaluation;
    return record;
}

void LiveOrientationPredictionSession::finish() noexcept
{
    evaluator_.finish();
}

const OrientationPredictor& LiveOrientationPredictionSession::predictor() const noexcept
{
    return predictor_;
}

const DelayedOrientationEvaluator& LiveOrientationPredictionSession::evaluator() const noexcept
{
    return evaluator_;
}

} // namespace xreal::sensors
