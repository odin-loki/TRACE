// Pattern of life drives reacquisition, the loiter threshold, cover stops and
// one of the three convergence predictors, so its failure modes are subtle.
#include "trace/core/pattern_of_life.hpp"

#include <cstdio>

#include "test_harness.hpp"

using namespace trace;

namespace {

constexpr Real kHour = 3600.0;

void test_unfitted_is_neutral() {
    // An entity with no history is unknown, not suspicious. Reporting 1.0 here
    // would make every newly-seen entity look anomalous.
    const DomainProfile p = UrbanHUMINT();
    PatternOfLife pol(p);
    CHECK(!pol.fitted());
    CHECK_NEAR(pol.anomaly_score(0.0, Vec2{0, 0}), 0.5, 1e-9);
    Rng rng(1);
    CHECK(pol.predict_location(0.0, rng).uncertainty_m > 900.0);
}

void test_learns_a_daily_routine() {
    // Home at night, work in the day, every day. The model should find both.
    const DomainProfile p = UrbanHUMINT();
    PatternOfLife pol(p);
    Rng rng(42);
    const Vec2 home{0, 0};
    const Vec2 work{2000, 1500};

    for (int day = 0; day < 12; ++day) {
        const Real base = day * 86400.0;
        for (int h = 0; h < 8; ++h) {
            pol.add(base + h * kHour,
                    Vec2{home.x + rng.normal(0, 30), home.y + rng.normal(0, 30)});
        }
        for (int h = 9; h < 17; ++h) {
            pol.add(base + h * kHour,
                    Vec2{work.x + rng.normal(0, 30), work.y + rng.normal(0, 30)});
        }
    }
    CHECK(pol.fitted());

    // Normal behaviour scores low; the same place at the wrong hour, or an
    // entirely new place, scores high.
    const Real normal_night = pol.anomaly_score(3.0 * kHour, home);
    const Real normal_day = pol.anomaly_score(12.0 * kHour, work);
    const Real wrong_place = pol.anomaly_score(12.0 * kHour, Vec2{-9000, 7000});

    std::printf("  anomaly: home@03:00=%.3f  work@12:00=%.3f  nowhere@12:00=%.3f\n",
                normal_night, normal_day, wrong_place);
    CHECK(normal_night < 0.55);
    CHECK(normal_day < 0.55);
    CHECK(wrong_place > normal_day);
    CHECK(wrong_place > 0.6);
}

void test_predicts_the_right_place_for_the_hour() {
    const DomainProfile p = UrbanHUMINT();
    PatternOfLife pol(p);
    Rng rng(7);
    const Vec2 home{0, 0};
    const Vec2 work{3000, 0};

    for (int day = 0; day < 15; ++day) {
        const Real base = day * 86400.0;
        for (int h = 0; h < 6; ++h) {
            pol.add(base + h * kHour, Vec2{rng.normal(0, 25), rng.normal(0, 25)});
        }
        for (int h = 10; h < 16; ++h) {
            pol.add(base + h * kHour,
                    Vec2{3000 + rng.normal(0, 25), rng.normal(0, 25)});
        }
    }
    CHECK(pol.fitted());

    const auto at_night = pol.predict_location(2.0 * kHour, rng, 200);
    const auto at_noon = pol.predict_location(13.0 * kHour, rng, 200);
    std::printf("  predicted 02:00 -> (%.0f, %.0f); 13:00 -> (%.0f, %.0f)\n",
                at_night.position.x, at_night.position.y, at_noon.position.x,
                at_noon.position.y);

    // The prediction must discriminate by hour; without that, reacquisition
    // would just guess the entity's overall centre of mass.
    CHECK(distance(at_night.position, home) < distance(at_night.position, work));
    CHECK(distance(at_noon.position, work) < distance(at_noon.position, home));
}

void test_clone_transfers_a_baseline() {
    const DomainProfile p = UrbanHUMINT();
    PatternOfLife parent(p);
    Rng rng(3);
    for (int i = 0; i < 40; ++i) {
        parent.add(i * kHour, Vec2{rng.normal(100, 20), rng.normal(50, 20)});
    }
    CHECK(parent.fitted());

    PatternOfLife child(p);
    child.clone_from(parent);
    CHECK(child.fitted());
    CHECK_NEAR(child.anomaly_score(5 * kHour, Vec2{100, 50}),
               parent.anomaly_score(5 * kHour, Vec2{100, 50}), 1e-9);
}

void test_active_windows_and_spread() {
    const DomainProfile p = UrbanHUMINT();
    PatternOfLife pol(p);
    Rng rng(21);
    for (int day = 0; day < 10; ++day) {
        for (int h = 8; h < 12; ++h) {
            pol.add(day * 86400.0 + h * kHour,
                    Vec2{rng.normal(0, 40), rng.normal(0, 40)});
        }
    }
    CHECK(pol.fitted());
    const auto windows = pol.active_windows();
    CHECK(!windows.empty());
    CHECK(pol.spatial_spread() > 0.0);
    std::printf("  active windows: %zu, spatial spread %.1f m\n", windows.size(),
                pol.spatial_spread());
}

}  // namespace

int main() {
    test_unfitted_is_neutral();
    test_learns_a_daily_routine();
    test_predicts_the_right_place_for_the_hour();
    test_clone_transfers_a_baseline();
    test_active_windows_and_spread();
    return trace::test::summary("test_pattern_of_life");
}
