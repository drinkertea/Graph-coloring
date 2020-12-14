#include "solver.h"
#include "models.h"
#include "common.h"

namespace solver
{

struct SolutionManager
{
    SolutionManager(const Graph& graph)
    {
        Graph::ColorToVerts best;
        for (auto strategy : g_strategies)
        {
            auto t = graph.Colorize(strategy);
            if (t.size() > best.size())
                best.swap(t);
        }

        m_best_solution_value = (uint32_t)best.size();
        for (const auto& color : best)
            m_best_solution.color_coverage.emplace_back(color.second.begin(), color.second.end());
    }

    void OnSolution(const models::MainSolution& solution)
    {
        if (!IsInteger(solution.objective) || static_cast<uint32_t>(solution.objective) >= m_best_solution_value)
            return;

        if (m_best_solution.color_coverage.size() <= solution.color_coverage.size())
            return;

        m_best_solution_value = (uint32_t)solution.color_coverage.size();
        m_best_solution.color_coverage.clear();
        for (const auto& ind_set : solution.color_coverage)
            m_best_solution.color_coverage.emplace_back(ind_set);
    }

    uint32_t GetBestValue() const
    {
        return m_best_solution_value;
    }

    const ColorizationResult& GetBestSolution() const
    {
        return m_best_solution;
    }

private:
    uint32_t           m_best_solution_value = std::numeric_limits<uint32_t>::max();
    ColorizationResult m_best_solution{};
};

struct BnPSolver
{
    BnPSolver(const Graph& graph)
        : m_graph(graph)
        , m_inv_graph(m_graph.GetInversed())
        , m_main_model(m_graph)
        , m_support_model(m_inv_graph)
        , m_solutons(m_graph)
    {
    }

    ColorizationResult Solve()
    {
        BnP();
        return m_solutons.GetBestSolution();
    }

private:
    auto Branching(models::MainSolution& solution)
    {
        int way = int(solution.branching_variable - 0.5 > 0);
        return std::array<int, 2>{ way, way == 0 ? 1 : 0 };
    }

    bool Generation(models::MainSolution& solution, bool exact)
    {
        double prev_solution = 0.0;
        while (true)
        {
            bool find_exact = exact || solution.IsInteger();
            auto support_solution = m_support_model.Solve(solution.dual_variables, find_exact);
            auto lower_bound = std::ceil(solution.objective / support_solution.upper_bound);
            if (static_cast<uint32_t>(lower_bound) >= m_solutons.GetBestValue())
                return true;

            if (support_solution.ind_set.empty())
                return false;

            m_graph.AdjustIndSet(support_solution.ind_set);
            std::set<models::IndependetSet> ind_sets{ support_solution.ind_set };
            if (!exact && solution.dual_search_index != g_invalid_index)
            {
                m_graph.GetWeightHeuristicConstrFor(solution.dual_search_index, solution.dual_variables, [&](const auto& constr) {
                    models::IndependetSet ind_set{ constr.begin(), constr.end() };
                    if (m_forbidden_sets.count(ind_set))
                        return;

                    ind_sets.emplace(std::move(ind_set));
                });
            }

            if (!m_main_model.AddVariables(ind_sets))
                return false;

            solution = m_main_model.Solve();
            if (!exact && std::abs(solution.objective - prev_solution) < 0.0001)
                return false;

            prev_solution = solution.objective;
        }
        return false;
    }

    void BnP()
    {
        auto solution = m_main_model.Solve();

        for (auto exact : { false, true })
        {
            if (Generation(solution, exact))
                return;
        }

        if (solution.primal_branching_index == g_invalid_index)
        {
            return m_solutons.OnSolution(solution);
        }

        for (auto branch : Branching(solution))
        {
            auto main_constr = m_main_model.AddConstrain(solution.primal_branching_index, branch);
            auto sup_constr = branch == 0 ? m_support_model.AddConstrain(m_forbidden_sets, m_main_model.GetVariable(solution.primal_branching_index)) : nullptr;

            BnP();
        }
    }

private:
    const Graph&                    m_graph;
    const Graph                     m_inv_graph;
    SolutionManager                 m_solutons;
    models::MainProblemModel        m_main_model;
    models::SupportProblemModel     m_support_model;
    std::set<models::IndependetSet> m_forbidden_sets;
};

ColorizationResult Colorize(const Graph& graph)
{
    BnPSolver solver(graph);
    return solver.Solve();
}

};
