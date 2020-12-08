#pragma once

#include <memory>
#include <set>

struct Graph;

namespace models
{

using IndependetSet = std::set<uint32_t>;

struct MainProblemModelHolder;

struct MainProblemModel
{
    MainProblemModel(const Graph& graph);
    ~MainProblemModel();

    void AddVariable(const IndependetSet& ind_set);

private:
    const Graph&            m_graph;
    std::set<IndependetSet> m_sets;

    std::unique_ptr<MainProblemModelHolder> m_model;
};

};
