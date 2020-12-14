#pragma once

#include <memory>
#include <vector>
#include <deque>
#include <set>
#include "common.h"

struct Graph;

namespace models
{

using IndependetSet    = std::set<uint32_t>;
using IndependetSetRef = std::reference_wrapper<const IndependetSet>;
using ColorCoverage    = std::vector<IndependetSetRef>;
using Variables        = std::vector<double>;

struct MainProblemModelHolder;

struct IScopedConstrain
{
    virtual void OnModelUpdate(const MainProblemModelHolder& model) {};
    virtual ~IScopedConstrain() = default;
};

using ConstrainPtr = std::shared_ptr<IScopedConstrain>;

struct MainSolution
{
    double        objective{ 0.0 };
    size_t        primal_branching_index = g_invalid_index;
    size_t        dual_search_index      = g_invalid_index;
    Variables     dual_variables;
    double        branching_variable = 0.0;
    ColorCoverage color_coverage;

    bool IsInteger() const
    {
        return primal_branching_index == g_invalid_index;
    }
};

struct MainProblemModel
{
    MainProblemModel(const Graph& graph);
    ~MainProblemModel();

    MainSolution Solve() const;

    bool AddVariables(const std::set<models::IndependetSet>& ind_sets);

    ConstrainPtr AddConstrain(size_t index, int type);

    const IndependetSet& GetVariable(size_t index) const
    {
        return m_vars[index];
    }

private:
    const Graph&              m_graph;
    std::set<IndependetSet>   m_sets;
    std::deque<IndependetSet> m_vars;

    std::unique_ptr<MainProblemModelHolder> m_model;
    std::vector<std::weak_ptr<IScopedConstrain>> m_constrains;
};

struct SupportSolution
{
    double        upper_bound = 0.0;
    IndependetSet ind_set;
};
using SolutionCallback = std::function<void(const SupportSolution&)>;

enum class SolutionStatus
{
    Optimal = 0,
    AbortedByTimelimit,
    AbortedByUser,

    Unknown
};

struct SupportProblemModelHolder;

struct SupportProblemModel
{
    SupportProblemModel(const Graph& inv_graph);
    ~SupportProblemModel();

    SolutionStatus Solve(const SolutionCallback& callback, const Variables& weights, bool exact = false);

    // Add forbidden set as constrain
    ConstrainPtr AddConstrain(std::set<models::IndependetSet>& sets, const IndependetSet& ind_set);

private:
    const Graph&            m_graph;
    std::set<IndependetSet> m_sets;

    std::unique_ptr<SupportProblemModelHolder> m_model;
};

};
