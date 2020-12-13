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

struct ModelHolderBase
{
    ModelHolderBase(size_t vars_size, size_t constr_size, IloNumVar::Type type)
        : m_variables(m_env, vars_size)
        , m_expressions(m_env, constr_size)
        , m_constrains(m_env, constr_size)
        , m_model(m_env)
    {
        for (int i = 0; i < m_variables.getSize(); ++i)
        {
            std::string name = std::to_string(i);
            m_variables[i] = IloNumVar(m_env, 0, 1, type, name.c_str());
        }

        for (int i = 0; i < m_expressions.getSize(); ++i)
        {
            m_expressions[i] = IloExpr(m_env);
        }

        m_model.add(m_variables);
    }

    virtual ~ModelHolderBase()
    {
        m_env.end();
    }

protected:
    IloEnv          m_env{};
    IloRangeArray   m_constrains;
    IloModel        m_model;
    IloNumVarArray  m_variables;
    IloExprArray    m_expressions;
};

struct MainProblemModelHolder : public ModelHolderBase
{
    MainProblemModelHolder(const Graph& graph, const std::set<IndependetSet>& vars)
        : ModelHolderBase(vars.size(), graph.GetSize(), IloNumVar::Type::Float)
    {
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
        m_model.add(obj);
    }

    MainSolution Solve() const
    {
        IloCplex solver(m_model);
        solver.setOut(m_env.getNullStream());

        if (!solver.solve())
            return {};

        MainSolution res{};
        res.objective = solver.getObjValue();
        res.variables.resize(m_constrains.getSize(), 0.0);

        IloNumArray dual_vars(m_env, m_constrains.getSize());
        solver.getDuals(dual_vars, m_constrains);
        for (int i = 0; i < m_constrains.getSize(); ++i)
            res.variables[i] = dual_vars[i];

        return res;
    }

    ~MainProblemModelHolder() override = default;
};

ILOSIMPLEXCALLBACK0(MyCallback)
{
    if (isFeasible() && EpsValue(getObjValue()) > 1.0)
        abort();
}

struct SupportProblemModelHolder : public ModelHolderBase
{
    SupportProblemModelHolder(const Graph& graph, const std::set<IndependetSet>& cliques, const Variables& weights)
        : ModelHolderBase(graph.GetSize(), cliques.size(), IloNumVar::Type::Bool)
    {
        uint32_t k = 0u;
        for (const auto& constr : cliques)
        {
            for (auto vert : constr)
                m_expressions[k] += m_variables[vert];
            std::string name = "constr[" + std::to_string(k) + "]";
            m_constrains[k++] = IloRange(m_env, 0, m_expressions[k], 1, name.c_str());
        }

        IloExpr obj_expr(m_env);
        for (int i = 0; i < m_variables.getSize(); ++i)
            obj_expr += weights[i] * m_variables[i];
        IloObjective obj(m_env, obj_expr, IloObjective::Maximize);

        m_model.add(obj);
        m_model.add(m_constrains);
    }

    SupportSolution Solve(bool conditional) const
    {
        IloCplex solver(m_model);
        solver.setOut(m_env.getNullStream());

        if (conditional)
        {
            solver.setParam(IloCplex::Param::TimeLimit, 0.5);
            solver.use(MyCallback(m_env));
        }
        if (!solver.solve())
            return {};

        SupportSolution res;
        res.optimal = solver.getCplexStatus() == IloCplex::Status::Optimal;
        res.aborted = solver.getCplexStatus() == IloCplex::Status::AbortTimeLim;

        IloNumArray vars(m_env, m_variables.getSize());
        solver.getValues(vars, m_variables);
        std::vector<double> deb_vars;
        for (int i = 0; i < m_variables.getSize(); ++i)
        {
            deb_vars.emplace_back(EpsValue(vars[i]));
            if (EpsValue(vars[i]) == 1.0)
                res.ind_set.emplace(i);
        }

        return res;
    }

    ~SupportProblemModelHolder() override = default;
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

MainSolution MainProblemModel::Solve() const
{
    return m_model->Solve();
}

void MainProblemModel::AddVariables(const std::set<models::IndependetSet>& ind_sets)
{
    auto size_before = m_sets.size();
    m_sets += ind_sets;
    if (size_before == m_sets.size())
        return;

    m_model = std::make_unique<MainProblemModelHolder>(m_graph, m_sets);
}

void AddPerColorConstrains(const Graph& graph, std::set<IndependetSet>& m_sum_vert_less_one, Graph::ColorizationType type)
{
    auto verts_by_color = graph.Colorize(type);
    uint32_t k = 0;
    for (const auto& independed_nodes : verts_by_color)
    {
        if (independed_nodes.second.size() <= 2)
            continue;

        m_sum_vert_less_one.emplace(std::set<uint32_t>{
            independed_nodes.second.begin(),
            independed_nodes.second.end()
        });
    }
}

void AddNonEdgePairs(const Graph& graph, std::set<IndependetSet>& m_sum_vert_less_one)
{
    for (uint32_t i = 0; i < graph.GetSize(); ++i)
    {
        for (uint32_t j = i + 1; j < graph.GetSize(); ++j)
        {
            if (graph.HasEdge(i, j))
                continue;

            m_sum_vert_less_one.emplace(IndependetSet{ i, j });
        }
    }
}

SupportProblemModel::SupportProblemModel(const Graph& inv_graph, const Variables& weights)
    : m_graph(inv_graph)
{
    AddNonEdgePairs(m_graph, m_sets);
    for (auto type : g_strategies)
        AddPerColorConstrains(m_graph, m_sets, type);

    m_model = std::make_unique<SupportProblemModelHolder>(m_graph, m_sets, weights);
}

SupportProblemModel::~SupportProblemModel() = default;

SupportSolution SupportProblemModel::Solve(bool conditional) const
{
    return m_model->Solve(conditional);
}

};

