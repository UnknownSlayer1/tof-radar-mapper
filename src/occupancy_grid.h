// occupancy_grid.h
// 2D occupancy grid with log odds updates and ray casting.
// Pure C++, no Arduino dependencies.
//
// Why log odds: a probability grid saturates at 0 and 1, so repeated hits
// can never be undone by later free observations. Log odds add, so a cell
// hit many times needs many free observations to clear, and vice versa.
// This is the same representation robot vacuums and defense lidar
// mapping stacks use, at a scale that fits in ESP32 RAM.
//
// Layout: square grid, robot at the centre cell. Cell size is fixed at
// construction. World coordinates are millimetres relative to the robot.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "polar.h"

namespace radar {

class OccupancyGrid {
public:
    // cell_size_mm: edge of one square cell, e.g. 50 mm.
    // cells: grid is cells x cells, must be odd so the robot sits on a
    //        centre cell.
    OccupancyGrid(float cell_size_mm, int cells)
        : cell_size_mm_(cell_size_mm),
          cells_(cells),
          half_extent_mm_(cell_size_mm * static_cast<float>(cells) / 2.0f),
          log_odds_(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells), 0.0f) {}

    float cellSizeMm() const { return cell_size_mm_; }
    int cells() const { return cells_; }

    // World (x, y) in mm -> cell indices. Returns false outside the grid.
    bool worldToCell(float x_mm, float y_mm, int& cx, int& cy) const {
        cx = static_cast<int>(std::floor((x_mm + half_extent_mm_) / cell_size_mm_));
        cy = static_cast<int>(std::floor((y_mm + half_extent_mm_) / cell_size_mm_));
        return cx >= 0 && cx < cells_ && cy >= 0 && cy < cells_;
    }

    // Cell indices -> world coordinates of the cell centre, in mm.
    void cellToWorld(int cx, int cy, float& x_mm, float& y_mm) const {
        x_mm = (static_cast<float>(cx) + 0.5f) * cell_size_mm_ - half_extent_mm_;
        y_mm = (static_cast<float>(cy) + 0.5f) * cell_size_mm_ - half_extent_mm_;
    }

    float logOddsAt(int cx, int cy) const {
        if (cx < 0 || cx >= cells_ || cy < 0 || cy >= cells_) return 0.0f;
        return log_odds_[index(cx, cy)];
    }

    // Occupancy probability in [0, 1], 0.5 = unknown.
    float probabilityAt(int cx, int cy) const {
        const float l = logOddsAt(cx, cy);
        return 1.0f - 1.0f / (1.0f + std::exp(l));
    }

    bool isOccupied(int cx, int cy) const { return logOddsAt(cx, cy) > kOccupiedThreshold; }
    bool isFree(int cx, int cy) const { return logOddsAt(cx, cy) < kFreeThreshold; }
    bool isUnknown(int cx, int cy) const { return !isOccupied(cx, cy) && !isFree(cx, cy); }

    // Mark one cell hit (sensor saw a surface here) or free.
    void markHit(int cx, int cy) { addLogOdds(cx, cy, kHitIncrement); }
    void markFree(int cx, int cy) { addLogOdds(cx, cy, -kFreeDecrement); }

    // Integrate one range reading taken from the robot at the origin.
    // Ray casts from the origin to the endpoint: every cell the beam
    // crossed is free, the endpoint cell is occupied.
    void integrateReading(const PolarSample& sample) {
        if (!sample.valid) return;
        const CartesianPoint p = polarToCartesian(sample);
        int ex = 0, ey = 0;
        if (!worldToCell(p.x_mm, p.y_mm, ex, ey)) {
            // Endpoint outside the map: still clear the ray up to the edge.
            const float scale = maxRangeInGrid(p.x_mm, p.y_mm);
            const CartesianPoint clipped{ p.x_mm * scale, p.y_mm * scale };
            int cx = 0, cy = 0;
            if (worldToCell(clipped.x_mm, clipped.y_mm, cx, cy)) {
                raycastFree(centreCell(), centreCell(), cx, cy);
                markFree(cx, cy);
            }
            return;
        }
        raycastFree(centreCell(), centreCell(), ex, ey);
        markHit(ex, ey);
    }

    // Integrate a reading whose endpoint is out of range: the beam saw
    // nothing, so clear the ray out to the sensor max range.
    void integrateMiss(float angle_deg, float max_range_mm) {
        const CartesianPoint p = polarToCartesian(angle_deg, max_range_mm);
        int ex = 0, ey = 0;
        if (!worldToCell(p.x_mm, p.y_mm, ex, ey)) {
            const float scale = maxRangeInGrid(p.x_mm, p.y_mm);
            const CartesianPoint clipped{ p.x_mm * scale, p.y_mm * scale };
            if (worldToCell(clipped.x_mm, clipped.y_mm, ex, ey)) {
                raycastFree(centreCell(), centreCell(), ex, ey);
                markFree(ex, ey);
            }
            return;
        }
        raycastFree(centreCell(), centreCell(), ex, ey);
        markFree(ex, ey);
    }

    void clear() { std::fill(log_odds_.begin(), log_odds_.end(), 0.0f); }

    int centreCell() const { return cells_ / 2; }

private:
    static constexpr float kHitIncrement = 0.85f;       // log odds per hit
    static constexpr float kFreeDecrement = 0.35f;      // log odds per free pass
    static constexpr float kOccupiedThreshold = 0.65f;  // log odds > this: occupied
    static constexpr float kFreeThreshold = -0.30f;     // log odds < this: free
    static constexpr float kLogOddsMin = -4.0f;         // clamp, avoids saturation
    static constexpr float kLogOddsMax = 4.0f;

    std::size_t index(int cx, int cy) const {
        return static_cast<std::size_t>(cy) * static_cast<std::size_t>(cells_) +
               static_cast<std::size_t>(cx);
    }

    void addLogOdds(int cx, int cy, float delta) {
        if (cx < 0 || cx >= cells_ || cy < 0 || cy >= cells_) return;
        float& l = log_odds_[index(cx, cy)];
        l = std::clamp(l + delta, kLogOddsMin, kLogOddsMax);
    }

    // Bresenham line from (x0, y0) to (x1, y1), marking every cell free
    // except the endpoint cell itself (the caller marks that as a hit).
    void raycastFree(int x0, int y0, int x1, int y1) {
        const int dx = std::abs(x1 - x0);
        const int dy = -std::abs(y1 - y0);
        const int sx = x0 < x1 ? 1 : -1;
        const int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        int x = x0, y = y0;
        while (true) {
            if (x == x1 && y == y1) break;
            markFree(x, y);
            const int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x += sx; }
            if (e2 <= dx) { err += dx; y += sy; }
        }
    }

    // Scale factor so a point beyond the grid lands just inside the edge.
    float maxRangeInGrid(float x_mm, float y_mm) const {
        const float ax = std::abs(x_mm);
        const float ay = std::abs(y_mm);
        const float m = std::max(ax, ay);
        if (m <= 0.0f) return 1.0f;
        return (half_extent_mm_ - cell_size_mm_ * 0.5f) / m;
    }

    float cell_size_mm_;
    int cells_;
    float half_extent_mm_;
    std::vector<float> log_odds_;
};

}  // namespace radar
