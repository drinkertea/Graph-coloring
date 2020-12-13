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
        , m_solutons(m_graph)
    {
    }

    ColorizationResult Solve()
    {
        BnP();
        return m_solutons.GetBestSolution();
    }

private:
    size_t SelectBranch(const models::Variables& vars)
    {
        double min = std::numeric_limits<double>::max();
        size_t res = g_invalid_index;

        for (size_t i = 0; i < vars.size(); ++i)
        {
            auto val = vars[i];
            if (IsInteger(val))
                continue;

            if (val >= min)
                continue;

            min = val;
            res = i;
        }

        return res;
    }

    bool Generation(models::MainSolution& solution, bool exact)
    {
        double prev_solution = 0.0;
        while (true)
        {
            models::SupportProblemModel support_model(m_inv_graph, solution.variables);
            auto support_solution = support_model.Solve(exact);
            auto lower_bound = std::ceil(solution.objective / support_solution.upper_bound);
            if (static_cast<uint32_t>(lower_bound) >= m_solutons.GetBestValue())
                return true;

            if (support_solution.ind_set.empty())
                return false;

            std::set<models::IndependetSet> ind_sets{ support_solution.ind_set };
            auto index = SelectBranch(solution.variables);
            if (index == g_invalid_index)
                return true;

            m_graph.GetWeightHeuristicConstrFor(index, solution.variables, [&](const auto& constr) {
                ind_sets.emplace(constr.begin(), constr.end());
            });

            if (!m_main_model.AddVariables(ind_sets))
                return false;

            solution = m_main_model.Solve();
            if (std::abs(solution.objective - prev_solution) < 0.01)
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
        
    }

private:
    const Graph&             m_graph;
    const Graph              m_inv_graph;
    SolutionManager          m_solutons;
    models::MainProblemModel m_main_model;
};

ColorizationResult Colorize(const Graph& graph)
{
    BnPSolver solver(graph);
    solver.Solve();
    models::MainProblemModel model{ graph };
    auto solution = model.Solve();



    auto inv_graph = graph.GetInversed();
    models::SupportProblemModel sp(inv_graph, solution.variables);
    auto sol = sp.Solve();

    return ColorizationResult();
}

};
