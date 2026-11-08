// map_serializer.h
// Serializes the occupancy grid to JSON (for plotting on a PC) and to a
// printable ASCII top-down map (for the Arduino Serial Monitor).
// Pure C++, no Arduino dependencies. Output goes through a small
// string builder so main.cpp can print it with Serial.print().
#pragma once

#include <string>

#include "occupancy_grid.h"
#include "scan_assembler.h"

namespace radar {

// JSON: {"cell_mm":50,"cells":81,"robot":[40,40],"grid":[[0.5,...]]}
// Grid rows are emitted top row first (highest Y), one row per line,
// probabilities rounded to two decimals so the payload stays small
// enough for a single Serial dump at 115200 baud.
inline std::string gridToJson(const OccupancyGrid& grid) {
    std::string out;
    out.reserve(8192);
    out += "{\"cell_mm\":";
    out += std::to_string(static_cast<int>(grid.cellSizeMm()));
    out += ",\"cells\":";
    out += std::to_string(grid.cells());
    out += ",\"robot\":[";
    out += std::to_string(grid.centreCell());
    out += ",";
    out += std::to_string(grid.centreCell());
    out += "],\"grid\":[";
    for (int cy = grid.cells() - 1; cy >= 0; --cy) {
        out += "[";
        for (int cx = 0; cx < grid.cells(); ++cx) {
            const float p = grid.probabilityAt(cx, cy);
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%.2f", static_cast<double>(p));
            out += buf;
            if (cx + 1 < grid.cells()) out += ",";
        }
        out += "]";
        if (cy > 0) out += ",";
    }
    out += "]}\n";
    return out;
}

// ASCII map, north (+Y) at the top, robot at the centre marked 'R':
//   '#' occupied   '.' known free   ' ' unknown / never observed
inline std::string gridToAscii(const OccupancyGrid& grid) {
    std::string out;
    out.reserve(static_cast<std::size_t>(grid.cells() + 1) *
                static_cast<std::size_t>(grid.cells()));
    const int centre = grid.centreCell();
    for (int cy = grid.cells() - 1; cy >= 0; --cy) {
        for (int cx = 0; cx < grid.cells(); ++cx) {
            if (cx == centre && cy == centre) {
                out += 'R';
            } else if (grid.isOccupied(cx, cy)) {
                out += '#';
            } else if (grid.isFree(cx, cy)) {
                out += '.';
            } else {
                out += ' ';
            }
        }
        out += '\n';
    }
    return out;
}

// One scan as JSON: {"points":[{"a":0.0,"r":1234.0,"hit":true}, ...]}
inline std::string scanToJson(const ScanAssembler& scan) {
    std::string out;
    out.reserve(4096);
    out += "{\"points\":[";
    bool first = true;
    for (const ScanPoint& p : scan.points()) {
        if (!first) out += ",";
        first = false;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "{\"a\":%.2f,\"r\":%.1f,\"hit\":%s}",
                      static_cast<double>(p.angle_deg),
                      static_cast<double>(p.range_mm),
                      p.hit ? "true" : "false");
        out += buf;
    }
    out += "]}\n";
    return out;
}

}  // namespace radar
