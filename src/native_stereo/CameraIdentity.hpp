#pragma once

namespace xreal::stereo
{
// Reject configurations where the upstream DirectShow friendly-name selection
// could resolve to an unrelated camera or an arbitrary fallback device.
void requireUltraCamera();
}
