#pragma once

#include <memory>
#include <vector>
#include <set>

struct Graph;

namespace models
{

using IndependetSet = std::set<uint32_t>;
using Variables     = std::vector<double>;

struct MainSolution
{
    double    objective{ 0.0 };
    Variables variables;
};

struct MainProblemModelHolder;

struct MainProblemModel
{
    MainProblemModel(const Graph& graph);
    ~MainProblemModel();

    MainSolution Solve() const;

    void AddVariables(const std::set<models::IndependetSet>& ind_sets);

private:
    const Graph&            m_graph;
    std::set<IndependetSet> m_sets;

    std::unique_ptr<MainProblemModelHolder> m_model;
};

struct SupportSolution
{
    bool optimal = false;
    IndependetSet ind_set;
};

struct SupportProblemModelHolder;

struct SupportProblemModel
{
    SupportProblemModel(const Graph& inv_graph, const Variables& weights);
    ~SupportProblemModel();

    SupportSolution Solve(bool conditional = true) const;

private:
    const Graph&            m_graph;
    std::set<IndependetSet> m_sets;

    std::unique_ptr<SupportProblemModelHolder> m_model;
};

};
