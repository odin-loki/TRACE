// SPDX-License-Identifier: AGPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Odin Loch <https://github.com/odin-loki>

// Appearance evidence exists for one situation: entities whose positions are
// genuinely indistinguishable. These tests construct that situation, because
// in anything less crowded position alone already settles it - which is itself
// a finding, recorded in docs/VALIDATION.md.
#include "trace/core/descriptor.hpp"

#include <cstdio>

#include "trace/core/engine.hpp"
#include "test_harness.hpp"

using namespace trace;

namespace {

/// A descriptor for entity `k`, well separated from its neighbours.
Descriptor tag(int k) {
    return SoftBinner::encode({{static_cast<Real>(k), 0.0, 5.0, 24, 0.7}});
}

void test_descriptor_algebra() {
    const Descriptor a = tag(0);
    const Descriptor b = tag(5);
    CHECK(a.valid());
    CHECK(b.valid());

    // Self-similarity is 1; distinct tags are far apart.
    CHECK_NEAR(a.similarity(a), 1.0, 1e-6);   // float storage: ~1e-7 precision
    CHECK(a.similarity(b) < 0.2);
    std::printf("  similarity: self %.3f, distinct %.3f\n", a.similarity(a),
                a.similarity(b));

    // An absent descriptor reads as "no evidence", never as "dissimilar".
    const Descriptor none;
    CHECK(!none.valid());
    CHECK_NEAR(a.similarity(none), 0.0, 1e-12);

    // Soft binning is what makes cosine meaningful: neighbouring values must
    // overlap, distant ones must not.
    CHECK(tag(2).similarity(tag(2)) > tag(2).similarity(tag(3)));
    CHECK(tag(2).similarity(tag(3)) > tag(2).similarity(tag(5)));
}

void test_blend_tracks_the_entity() {
    Descriptor model = tag(1);
    for (int i = 0; i < 20; ++i) model.blend(tag(1), 0.9);
    CHECK(model.similarity(tag(1)) > 0.99);

    // A long memory should resist a single contradictory observation.
    Descriptor stubborn = tag(1);
    stubborn.blend(tag(4), 0.9);
    CHECK(stubborn.similarity(tag(1)) > stubborn.similarity(tag(4)));
    std::printf("  after one contradictory frame at momentum 0.9: "
                "still %.2f like the original vs %.2f like the intruder\n",
                stubborn.similarity(tag(1)), stubborn.similarity(tag(4)));
}

void test_normalisation_and_degenerate_input() {
    Descriptor d;
    d.v.fill(3.0f);
    d.normalise();
    CHECK(d.valid());
    CHECK_NEAR(d.similarity(d), 1.0, 1e-6);

    Descriptor zero;   // all zeros: nothing to normalise
    zero.normalise();
    CHECK(!zero.valid());

    // Encoding no features at all must not produce a bogus unit vector.
    CHECK(!SoftBinner::encode({}).valid());
}

/// Six entities converge on one point, mill about within measurement noise of
/// each other, then disperse. Position carries no information during the
/// huddle; only the descriptor can say who came out where.
int identity_errors(Real appearance_weight, std::uint64_t seed) {
    constexpr int kN = 6;
    EngineConfig cfg;
    cfg.profile = CityCameraSurveillance();
    cfg.profile.scan_dt_s = 1.0;
    cfg.profile.pos_noise_m = 1.0;
    cfg.profile.meas_noise_var = 4.0;
    cfg.profile.appearance_weight = appearance_weight;
    cfg.profile.appearance_sigma = 0.30;
    cfg.area = Area{0, 400, 0, 400};
    cfg.seed = seed;
    Engine eng(cfg);

    Rng rng(seed * 7 + 5);
    std::vector<std::string> id_before(kN), id_after(kN);

    for (int i = 0; i < 80; ++i) {
        const Real t = i * 1.0;
        std::vector<Observation> obs;
        std::vector<Vec2> truth(kN);

        for (int k = 0; k < kN; ++k) {
            const Real angle = 2.0 * std::numbers::pi * k / kN;
            Vec2 p;
            if (i < 25) {                       // converge
                const Real r = 120.0 - i * 4.0;
                p = {200.0 + r * std::cos(angle), 200.0 + r * std::sin(angle)};
            } else if (i < 50) {                // huddle: all within ~2 m
                p = {200.0 + rng.normal(0.0, 1.0), 200.0 + rng.normal(0.0, 1.0)};
            } else {                            // disperse, each a new way
                const Real out = 2.0 * std::numbers::pi * ((k + 3) % kN) / kN;
                const Real r = (i - 50) * 4.0;
                p = {200.0 + r * std::cos(out), 200.0 + r * std::sin(out)};
            }
            truth[k] = p;
            obs.push_back(Observation("o" + std::to_string(i) + "_" + std::to_string(k),
                                      t, p, Modality::GEOINT, 0.95, "CAM")
                              .with_descriptor(tag(k)));
        }

        const ScanReport r = eng.ingest(obs, t);
        const auto nearest = [&](Vec2 p) {
            std::string best;
            Real bd = 1e9;
            for (const auto& tr : r.targets) {
                const Real d = distance(tr.position, p);
                if (d < bd) { bd = d; best = tr.track_id; }
            }
            return best;
        };
        if (i == 20) for (int k = 0; k < kN; ++k) id_before[k] = nearest(truth[k]);
        if (i == 75) for (int k = 0; k < kN; ++k) id_after[k] = nearest(truth[k]);
    }

    int lost = 0;
    for (int k = 0; k < kN; ++k) {
        if (id_before[k].empty() || id_before[k] != id_after[k]) ++lost;
    }
    return lost;
}

void test_appearance_holds_identity_through_a_huddle() {
    // Averaged over seeds: a single run of this is noisy, and asserting on one
    // draw would test the seed rather than the mechanism.
    int total_without = 0, total_with = 0;
    constexpr int kSeeds = 7;
    for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
        total_without += identity_errors(0.0, seed);
        total_with += identity_errors(4.0, seed);
    }
    const Real avg_without = static_cast<Real>(total_without) / kSeeds;
    const Real avg_with = static_cast<Real>(total_with) / kSeeds;
    std::printf("  six entities huddle then disperse, mean over %d seeds: "
                "identities lost without appearance %.1f/6, with appearance %.1f/6\n",
                kSeeds, avg_without, avg_with);
    CHECK(avg_with < avg_without);
    CHECK(avg_with <= 4.5);
}

}  // namespace

int main() {
    test_descriptor_algebra();
    test_blend_tracks_the_entity();
    test_normalisation_and_degenerate_input();
    test_appearance_holds_identity_through_a_huddle();
    return trace::test::summary("test_appearance");
}
