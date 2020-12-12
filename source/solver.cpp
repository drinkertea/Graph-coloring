#include "solver.h"
#include "models.h"
#include "common.h"

namespace solver
{

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

ColorizationResult Colorize(const Graph& graph)
{
    models::MainProblemModel model{ graph };
    auto solution = model.Solve();

    auto index = SelectBranch(solution.variables);
    if (index == g_invalid_index)
        return{};

    std::set<models::IndependetSet> ind_sets;
    graph.GetWeightHeuristicConstrFor(index, solution.variables, [&](const auto& constr) {
        ind_sets.emplace(constr.begin(), constr.end());
    });
    model.AddVariables(ind_sets);

    auto inv_graph = graph.GetInversed();
    models::SupportProblemModel sp(inv_graph, solution.variables);
    auto sol = sp.Solve();

    return ColorizationResult();
}

};
