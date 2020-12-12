#pragma once

#include <cstdint>
#include <vector>
#include <set>
#include <atomic>

struct Graph;

struct ColorizationResult
{
    std::vector<std::set<uint32_t>> color_coverage;
};

namespace solver
{

ColorizationResult Colorize(const Graph& graph);

};

