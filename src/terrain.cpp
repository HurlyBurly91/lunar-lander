#include "lander/terrain.hpp"

#include <algorithm>
#include <array>

namespace lander {

namespace {

class Rng {
public:
    explicit Rng(std::uint64_t seed)
        : state_(seed == 0 ? 0x9E3779B97F4A7C15ULL : seed) {}

    double uniform() {
        state_ =
            state_ ^ (state_ << 13) ^
            state_ >> 7 ^
            state_ ^ (state_ << 17);
        return (state_ >> 11) * 0x1p-53;
    }

private:
    std::uint64_t state_;
};

double cell_value(std::uint64_t salt, int cell) {
    std::uint64_t x =
        salt ^
        static_cast<std::uint64_t>(cell) * 0x9E3779B97F4A7C15ULL;
    x ^= x >> 33;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33;
    x *= 0xc4ceb9fe1a85ec53ULL;
    x ^= x >> 33;
    return (x >> 11) * 0x1p-53;
}

double smooth_step(double t) {
    t = t < 0.0 ? 0.0 : t > 1.0 ? 1.0 : t;
    return t * t * (3.0 - 2.0 * t);
}

double periodic_value_noise(double arc, double circumference, int cells,
                            std::uint64_t salt) {
    double x = arc / circumference * static_cast<double>(cells);
    double i0 = std::floor(x);
    double t = smooth_step(x - i0);
    int c0 = static_cast<int>(std::fmod(i0, static_cast<double>(cells)));
    int c1 = c0 + 1;
    if (c0 < 0) {
        c0 += cells;
    }
    if (c1 >= cells) {
        c1 -= cells;
    }
    return cell_value(salt, c0) * (1.0 - t) + cell_value(salt, c1) * t;
}

}  // namespace

Terrain::Terrain(std::uint64_t seed, double reference_radius,
                 double height_scale)
    : seed_(seed),
      reference_radius_(reference_radius),
      height_scale_(height_scale),
      circumference_(kTwoPi * reference_radius) {
    Rng rng(seed);
    const double C = circumference_;
    double base[3] = {
        0.0,
        0.38 * C,
        0.76 * C,
    };

    for (int i = 0; i < 3; ++i) {
        double center = base[i];
        if (i != 0) {
            center += (rng.uniform() - 0.5) * 0.08 * C;
        }
        center = normalize_arc(center);

        Pad pad;
        pad.center_arc = center;
        pad.half_width = 6.0;
        pad.radius = base_radius_at_arc(center);
        pad.multiplier = 1;
        pads_.push_back(pad);
    }

    std::sort(pads_.begin(), pads_.end(),
              [](const Pad& a, const Pad& b) {
                  return a.center_arc < b.center_arc;
              });
}

double Terrain::base_height_at_arc(double arc) const {
    arc = normalize_arc(arc);
    std::uint64_t salt = seed_ ^ 0xA5A489058670D2B2ULL;
    double h = 9.0 * height_scale_ *
               periodic_value_noise(arc, circumference_, 17, salt ^ 1);
    h += 4.0 * height_scale_ *
         periodic_value_noise(arc, circumference_, 75, salt ^ 7);
    h += 1.6 * height_scale_ *
         periodic_value_noise(arc, circumference_, 321, salt ^ 13);
    return h;
}

double Terrain::base_radius_at_arc(double arc) const {
    return reference_radius_ + base_height_at_arc(arc);
}

const Pad* Terrain::pad_at_arc(double arc) const {
    arc = normalize_arc(arc);
    const double C = circumference_;
    for (const Pad& pad : pads_) {
        double delta = normalize_arc(arc - pad.center_arc);
        double distance = std::min(delta, C - delta);
        if (distance <= pad.half_width) {
            return &pad;
        }
    }
    return nullptr;
}

double Terrain::surface_radius_at_arc(double arc) const {
    arc = normalize_arc(arc);
    if (const Pad* pad = pad_at_arc(arc)) {
        return pad->radius;
    }
    return base_radius_at_arc(arc);
}

double Terrain::surface_radius_at_angle(double theta) const {
    return surface_radius_at_arc(arc_at_angle(theta));
}

double Terrain::max_surface_radius() const {
    double best = -1.0e300;
    const int samples = 2048;
    for (int i = 0; i < samples; ++i) {
        double arc = i * circumference_ / samples;
        best = std::max(best, surface_radius_at_arc(arc));
    }
    return best;
}

}  // namespace lander
