#include "graph.h"
#include "solver.h"
#include "timer.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <thread>
#include <Windows.h>

struct Task
{
    std::string graph;
    size_t      answer = 0;
};

std::vector<uint32_t> ValidColoring(const Graph& g, const ColorizationResult& res)
{
    std::vector<std::set<uint32_t>> colors_per_vertex(g.GetSize());
    for (uint32_t i = 0; i < res.color_coverage.size(); ++i)
    {
        for (auto v : res.color_coverage[i])
        {
            colors_per_vertex[v].emplace(i);
        }
    }

    for (uint32_t a = 0; a < g.GetSize(); ++a)
    {
        for (auto b : g.GetNeighbors(a))
        {
            if (!(colors_per_vertex[a] * colors_per_vertex[b]).empty())
                return {};
        }
    }

    std::vector<uint32_t> colors;
    for (uint32_t a = 0; a < g.GetSize(); ++a)
    {
        if (colors_per_vertex[a].empty())
            return {};
        colors.emplace_back(*colors_per_vertex[a].begin());
    }

    return colors;
}

void Test(const std::string& file_path, const std::string& path, size_t answer)
{
    std::cout << "Test graph: " << path << std::endl;

    uint64_t bnp_elapsed = 0;
    std::string test_result = "CRASHED";
    std::vector<uint32_t> colors;
    ColorizationResult res;

    try
    {
        Graph g(path);

        Timer timer;
        TimeoutThread break_timer(std::chrono::seconds(7200));

        res = solver::Colorize(g);

        bnp_elapsed = timer.Stop();

        colors = ValidColoring(g, res);
        auto passed = answer == res.color_coverage.size() && !colors.empty();
        test_result = passed ? "PASSED" : "FAILED";
    }
    catch (std::runtime_error&)
    {
        std::cerr << "Something went wrong, please debug me :)" << std::endl;
    }

    {
        std::stringstream ss_far;
        ss_far << "Test graph:                      " << path << std::endl;
        ss_far << "Result:                          " << test_result << std::endl;
        ss_far << "Branch and price algorithm time: " << bnp_elapsed << " ms" << std::endl;
        //ss_far << "Branch and Bound branch count:   " << cf.branch_count << std::endl;
        //ss_far << "Average heurisrtic time:         " << cf.average_heuristic_time << std::endl;
        //ss_far << "Average solve time:              " << cf.average_solve_time << std::endl;
        //ss_far << "Average loop time:               " << cf.average_loop_time << std::endl;
        ss_far << "Colors count:                    " << res.color_coverage.size() << std::endl;
        ss_far << "Colors:                          ";
        for (auto c : colors)
            ss_far << c << " ";
        ss_far << std::endl;
        ss_far << std::endl;

        std::ofstream outfile(file_path, std::ios_base::app);
        outfile << ss_far.str();
    }
    {
        std::stringstream ss_short;
        ss_short << path << ";" << bnp_elapsed;
        ss_short << std::endl;

        std::ofstream outfile_short("short_" + file_path, std::ios_base::app);
        outfile_short << ss_short.str();
    }
}

std::vector<Task> ReadTasks(const std::string& path)
{
    std::ifstream myfile(path);
    std::string line;
    std::vector<Task> tasks;
    while (std::getline(myfile, line))
    {
        Task t;
        std::istringstream ss(line);
        ss >> t.graph >> t.answer;
        tasks.push_back(t);
    }
    return tasks;
}

void RunTests(const std::string& prefix)
{
    for (const auto& test : ReadTasks("tasks/" + prefix + ".txt"))
        Test(prefix + "_test_results.txt", "graphs/" + test.graph, test.answer);
}

int main()
{
    RunTests("custom");
    RunTests("simple");
    RunTests("medium");
    RunTests("hardcr");
    return 0;
}
