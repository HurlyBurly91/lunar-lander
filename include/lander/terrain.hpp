#pragma once

#include <cstdint>
#include <vector>

namespace lander {

// A flat landing site carved into the terrain. [x_min, x_max] is the flat
// interval at constant height y; the lander only lands safely on these.
struct Pad {
    double x_min{};
    double x_max{};
    double y{};
    int multiplier{1};

    bool operator==(const Pad&) const = default;
};

// The lunar surface: a deterministic, single-valued height function of x.
//
// The base surface is three octaves of 1-D value noise (linearly
// interpolated hashed lattice values), which yields visibly jagged hills,
// valleys, and slopes. A small number of landing sites are then flattened
// into the surface itself, so the drawn terrain is the collision surface.
//
// The same seed always produces the identical surface (pads and heights).
class Terrain {
public:
    Terrain() = default;
    explicit Terrain(std::uint64_t seed);

    // Height of the surface at horizontal position x. For x inside a
    // landing site this is the site's flat height; otherwise it is the
    // jagged base surface.
    double height_at(double x) const;

    const std::vector<Pad>& pads() const noexcept;
    std::uint64_t seed() const noexcept;

private:
    // Jagged base surface without the flattened landing sites.
    double base_height(double x) const;

    std::uint64_t seed_{};
    std::vector<Pad> pads_;
};

} // namespace lander
