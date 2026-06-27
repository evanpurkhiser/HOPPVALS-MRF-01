#pragma once

#include <string>
#include <vector>

#include "hv-mrf-01/types.hpp"

// Automated motor self-test for bench bring-up of a freshly-wired board. Spins
// each motor, checks that the encoders and current sensors respond sanely,
// verifies direction, brake, and a linear duty→speed response, and (for both
// sides) that the two motors track each other.
//
// The test owns the motors for its whole duration: run() calls motion::stop()
// first (so the 100 Hz controller isn't fighting it) and only ever uses the
// motor::drive / motor::debug surfaces — it never re-arms the controller. Every
// drive phase is a fixed, bounded set of delays (no spin-until loops), so the
// total runtime is deterministic and the motors always end actively braked.
//
// Reach it from the console with `selftest [L|R|both]`; run() is the
// transport-agnostic core that the command delegates to.

namespace hvmrf01::self_test {

// Outcome of a single named check. `note` carries the machine-parseable
// measured detail (e.g. "side=L dcount=420 dir=+ avg_ma=85"); on failure it is
// prefixed with "reason=<token>" so the why is always front-and-centre.
struct Result
{
    std::string name;
    bool        pass;
    std::string note;
};

// The full battery of results, in the order they ran.
struct SelfTestResult
{
    std::vector<Result> results;

    bool passed() const;      // true iff every check passed
    int  pass_count() const;
    int  fail_count() const;
};

// Optional callback invoked as each check completes, so a caller can surface
// results incrementally instead of waiting for the return. On the interactive
// serial console these print live; the remote websocket transport batches them
// into one reply when run() returns, like the other long-running commands.
using ProgressFn = void (*)(const Result&);

// Run the self-test for one side (Left/Right) or the full both-sides battery
// (Both also adds the joint two-motor tracking stage). Blocks for the duration
// of the test (~5 s single side, ~14.5 s for both). Leaves both motors braked.
SelfTestResult run(hvmrf01::Side side, ProgressFn on_result = nullptr);

}  // namespace hvmrf01::self_test
