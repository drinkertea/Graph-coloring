#pragma once

#include <memory>
#include <vector>
#include <set>

struct Graph;

namespace models
{

using IndependetSet = std::set<uint32_t>;
using Variables     = std::vector<double>;

struct Solution
{
    double    objective{ 0.0 };
    Variables variables;
};

struct MainProblemModelHolder;

struct MainProblemModel
{
    MainProblemModel(const Graph& graph);
    ~MainProblemModel();

    Solution Solve() const;

    void AddVariable(const IndependetSet& ind_set);

private:
    const Graph&            m_graph;
    std::set<IndependetSet> m_sets;

    std::unique_ptr<MainProblemModelHolder> m_model;
};

};
