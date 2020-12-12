#pragma warning (push, 0)
#ifndef IL_STD
#define IL_STD
#endif
#include <ilcplex/ilocplex.h>
#pragma warning (pop)

#include "models.h"
#include "graph.h"
#include "common.h"

namespace models
{

struct MainProblemModelHolder
{
    MainProblemModelHolder(const Graph& graph, const std::set<IndependetSet>& vars)
        : m_variables(m_env, vars.size())
        , m_expressions(m_env, graph.GetSize())
        , m_constrains(m_env, graph.GetSize())
        , m_model(m_env)
    {
        for (size_t i = 0; i < vars.size(); ++i)
        {
            std::string name = std::to_string(i);
            m_variables[i] = IloNumVar(m_env, 0, 1, IloNumVar::Type::Float, name.c_str());
        }

        for (size_t i = 0; i < graph.GetSize(); ++i)
        {
            m_expressions[i] = IloExpr(m_env);
        }

        uint32_t k = 0u;
        for (const auto& ind_set : vars)
        {
            for (auto v : ind_set)
            {
                m_expressions[v] += m_variables[k];
            }
            ++k;
        }

        for (size_t i = 0; i < graph.GetSize(); ++i)
        {
            std::string name = "constr[" + std::to_string(i) + "]";
            m_constrains[i] = IloRange(m_env, 1, m_expressions[i], IloInfinity, name.c_str());
        }

        IloExpr obj_expr(m_env);
        for (uint32_t i = 0; i < vars.size(); ++i)
            obj_expr += m_variables[i];
        IloObjective obj(m_env, obj_expr, IloObjective::Minimize);

        m_model.add(m_constrains);
        m_model.add(m_variables);
        m_model.add(obj);
    }

    Solution Solve() const
    {
        IloCplex solver(m_model);
        solver.setOut(m_env.getNullStream());

        if (!solver.solve())
            return {};

        Solution res{};
        res.objective = solver.getObjValue();
        res.variables.resize(m_constrains.getSize(), 0.0);

        IloNumArray dual_vars(m_env, m_constrains.getSize());
        solver.getDuals(dual_vars, m_constrains);
        for (int i = 0; i < m_constrains.getSize(); ++i)
            res.variables[i] = dual_vars[i];

        return res;
    }

    ~MainProblemModelHolder()
    {
        m_env.end();
    }

private:
    IloEnv          m_env{};
    IloRangeArray   m_constrains;
    IloModel        m_model;
    IloNumVarArray  m_variables;
    IloExprArray    m_expressions;
};

MainProblemModel::MainProblemModel(const Graph& graph)
    : m_graph(graph)
{
    std::vector<double> weights(m_graph.GetSize(), 1.0);
    for (uint32_t v = 0; v < m_graph.GetSize(); ++v)
    {
        bool was = false;
        m_graph.GetWeightHeuristicConstrFor(v, weights, [&](const auto& constr) {
            m_sets.emplace(constr.begin(), constr.end());
            was = true;
        });
        if (!was)
            m_sets.emplace(IndependetSet{ v });
    }

    m_model = std::make_unique<MainProblemModelHolder>(m_graph, m_sets);
}

MainProblemModel::~MainProblemModel() = default;

Solution MainProblemModel::Solve() const
{
    return m_model->Solve();
}

};

