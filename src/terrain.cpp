#include "lander/terrain.hpp"

#include <algorithm>
#include <cmath>

namespace lander {

namespace {

constexpr std::uint64_t kMixIncrement = 0x9E3779B97F4A7C15ULL;
constexpr std::uint64_t kMixMulA = 0xBF58476D1CE4E5B9ULL;
constexpr std::uint64_t kMixMulB = 0x94D049BB133111EBULL;

// SplitMix64: the same deterministic integer-only PRNG family used by
// src/sim.cpp, so terrain generation stays reproducible for a given seed.
class Rng {
public:
    explicit Rng(std::uint64_t seed) : state_(seed) {}

    std::uint64_t next() {
        std::uint64_t z = (state_ += kMixIncrement);
        z = (z ^ (z >> 30)) * kMixMulA;
        z = (z ^ (z >> 27)) * kMixMulB;
        return z ^ (z >> 31);
    }

    // Uniform double in [0, 1).
    double uniform() {
        return static_cast<double>(next() >> 11) * 0x1.0p-53;
    }

private:
    std::uint64_t state_;
};

// Deterministic pseudo-random value in [0, 1) for an integer lattice cell,
// derived from the cell index and a per-octave salt.
double cell_value(std::int64_t cell, std::uint64_t salt) {
    std::uint64_t z =
        static_cast<std::uint64_t>(cell) * kMixIncrement ^ salt;
    z = (z ^ (z >> 30)) * kMixMulA;
    z = (z ^ (z >> 27)) * kMixMulB;
    z = (z ^ (z >> 31));
    return static_cast<double>(z >> 11) * 0x1.0p-53;
}

// 1-D value noise: linear interpolation between hashed lattice values.
// Linear (not smooth) interpolation keeps the surface visibly jagged.
double vnoise(double x, std::uint64_t salt) {
    const std::int64_t i0 = static_cast<std::int64_t>(std::floor(x));
    const double t = x - static_cast<double>(i0);
    const double v0 = cell_value(i0, salt);
    const double v1 = cell_value(i0 + 1, salt);
    return v0 * (1.0 - t) + v1 * t;
}

// Landing site geometry.
constexpr double kSiteHalfWidth = 6.0;  // 12 m wide: plenty for the lander.
constexpr double kSiteRange = 150.0;    // centers drawn from [-150, 150]
constexpr double kSiteMinSpacing = 100.0;
constexpr int kSiteCount = 3;
constexpr int kMaxPlacementsPerSite = 200;

} // namespace

Terrain::Terrain(std::uint64_t seed) : seed_(seed) {
    Rng rng(seed_);

    // Place a few sites, each far enough from the ones already placed that
    // sites never overlap. The rejection loop is deterministic: the RNG is
    // drawn in the same order for a given seed, so the result is identical.
    std::vector<double> centers;
    for (int i = 0; i < kSiteCount; ++i) {
        bool placed = false;
        for (int attempt = 0; attempt < kMaxPlacementsPerSite && !placed;
             ++attempt) {
            const double center =
                -kSiteRange + 2.0 * kSiteRange * rng.uniform();
            bool too_close = false;
            for (const double other : centers) {
                if (std::abs(other - center) < kSiteMinSpacing) {
                    too_close = true;
                    break;
                }
            }
            if (!too_close) {
                centers.push_back(center);
                placed = true;
            }
        }
    }

    // Each site is flattened to the base-surface height at its center, so
    // it reads as a deliberate plateau carved into the terrain.
    for (const double center : centers) {
        Pad pad;
        pad.x_min = center - kSiteHalfWidth;
        pad.x_max = center + kSiteHalfWidth;
        pad.y = base_height(center);
        pad.multiplier = 1;
        pads_.push_back(pad);
    }
    std::sort(pads_.begin(), pads_.end(),
              [](const Pad& a, const Pad& b) { return a.x_min < b.x_min; });
}

double Terrain::base_height(double x) const {
    // Three octaves of value noise: broad undulation, medium hills, fine
    // jaggedness. Scales and amplitudes are fixed; only the seed varies.
    const double u =
        2.0 * vnoise(x / 120.0, seed_ * kMixIncrement + 0x01) - 1.0;
    const double m =
        2.0 * vnoise(x / 28.0, seed_ * kMixIncrement + 0x02) - 1.0;
    const double f =
        2.0 * vnoise(x / 6.5, seed_ * kMixIncrement + 0x03) - 1.0;
    return 9.0 * u + 4.0 * m + 1.6 * f;
}

double Terrain::height_at(double x) const {
    for (const Pad& pad : pads_) {
        if (x >= pad.x_min && x <= pad.x_max) {
            return pad.y;
        }
    }
    return base_height(x);
}

const std::vector<Pad>& Terrain::pads() const noexcept {
    return pads_;
}

std::uint64_t Terrain::seed() const noexcept {
    return seed_;
}

} // namespace lander
