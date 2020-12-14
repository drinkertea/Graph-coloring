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

struct ScopeGuard
{
    std::function<void(void)> callback;

    ~ScopeGuard()
    {
        callback();
    }
};

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

struct MainProblemModelHolder;

struct ConstrainsGuard : public IScopedConstrain
{
    ConstrainsGuard(size_t variable_index, int type);

    void OnModelUpdate(const MainProblemModelHolder& model) override;

    ~ConstrainsGuard() override;

private:
    const IloModel* m_model = nullptr;
    size_t          m_variable_index = g_invalid_index;
    int             m_type = 0;
    IloExtractable  m_constrain;
};

struct MainProblemModelHolder : public ModelHolderBase
{
    friend struct ConstrainsGuard;

    MainProblemModelHolder(const Graph& graph, const std::deque<IndependetSet>& vars)
        : ModelHolderBase(vars.size(), graph.GetSize(), IloNumVar::Type::Float)
        , m_vars(vars)
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
        ScopeGuard sg{ [&solver] { solver.end(); } };
        solver.setOut(m_env.getNullStream());
        solver.setWarning(m_env.getNullStream());

        if (!solver.solve())
            return {};

        MainSolution res{};
        res.objective = solver.getObjValue();
        res.dual_variables.resize(m_constrains.getSize(), 0.0);

        IloNumArray primal_vars(m_env, m_variables.getSize());
        solver.getValues(primal_vars, m_variables);
        res.primal_branching_index = GetMaxIndex(primal_vars, [&](int i, double val) {
            if (EpsValue(primal_vars[i]) == 1.0)
                res.color_coverage.emplace_back(m_vars[i]);
        });

        IloNumArray dual_vars(m_env, m_constrains.getSize());
        solver.getDuals(dual_vars, m_constrains);
        res.dual_search_index = GetMaxIndex(dual_vars, [&](int i, double val) {
            res.dual_variables[i] = dual_vars[i];
        });

        if (res.primal_branching_index != g_invalid_index)
            res.branching_variable = primal_vars[res.primal_branching_index];

        return res;
    }

    ~MainProblemModelHolder() override = default;

private:
    size_t GetMaxIndex(const IloNumArray& vars, const std::function<void(int, double)>& callback) const
    {
        double max = 0.0;
        size_t res = g_invalid_index;

        for (int i = 0; i < vars.getSize(); ++i)
        {
            auto val = vars[i];
            callback(i, val);

            if (IsInteger(val))
                continue;

            if (EpsValue(val) <= max)
                continue;

            max = val;
            res = i;
        }

        return res;
    }

    const std::deque<IndependetSet>& m_vars;
};

ConstrainsGuard::ConstrainsGuard(size_t variable_index, int type)
    : m_variable_index(variable_index)
    , m_type(type)
{
}

void ConstrainsGuard::OnModelUpdate(const MainProblemModelHolder& model)
{
    m_model = &model.m_model;
    if (m_type == 0)
    {
        m_constrain = model.m_model.add(model.m_variables[m_variable_index] <= 0.0);
    }
    else
    {
        m_constrain = model.m_model.add(model.m_variables[m_variable_index] >= 1.0);
    }
}

ConstrainsGuard::~ConstrainsGuard()
{
    if (m_model)
        m_model->remove(m_constrain);
    m_constrain.end();
}

template <typename T>
void UpdateSolution(const SolutionCallback& callback, const IloNumVarArray& variables, T& owner)
{
    SupportSolution solution{};
    solution.upper_bound = owner.getBestObjValue();

    IloNumArray vars(owner.getEnv(), variables.getSize());
    owner.getValues(vars, variables);
    std::vector<double> deb_vars;
    for (int i = 0; i < variables.getSize(); ++i)
    {
        deb_vars.emplace_back(EpsValue(vars[i]));
        if (EpsValue(vars[i]) == 1.0)
            solution.ind_set.emplace(i);
    }

    callback(solution);
}

ILOCPLEXGOAL3(MyBranchGoal, const SolutionCallback&, m_callback, IloNumVarArray, m_variables, bool, m_exact)
{
    if (isIntegerFeasible() && EpsValue(getObjValue()) > 1.0)
    {
        UpdateSolution(m_callback, m_variables, *this);
        if (!m_exact)
            abort();
    }
    return IloCplex::Goal{};
}

struct SupportProblemModelHolder;

struct SupportConstrainsGuard : public IScopedConstrain
{
    SupportConstrainsGuard(SupportProblemModelHolder& model, std::set<models::IndependetSet>& sets, const IndependetSet& ind_set);
    ~SupportConstrainsGuard() override;

private:
    SupportProblemModelHolder& m_model;
    IloExpr                    m_expr;
    IloExtractable             m_constrains;
    std::set<models::IndependetSet>& m_sets;
    const IndependetSet& m_ind_set;
};

struct SupportProblemModelHolder : public ModelHolderBase
{
    friend struct SupportConstrainsGuard;

    SupportProblemModelHolder(const Graph& graph, const std::set<IndependetSet>& cliques)
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
            obj_expr += 1.0 * m_variables[i];
        m_obj = IloObjective(m_env, obj_expr, IloObjective::Maximize);

        m_model.add(m_obj);
        m_model.add(m_constrains);
    }

    SolutionStatus Solve(const SolutionCallback& callback, const Variables& weights, bool exact)
    {
        IloExpr obj_expr(m_env);
        for (int i = 0; i < m_variables.getSize(); ++i)
            obj_expr += weights[i] * m_variables[i];
        m_obj.setExpr(obj_expr);

        IloCplex solver(m_model);
        ScopeGuard sg{ [&solver] { solver.end(); } };
        solver.setOut(m_env.getNullStream());
        solver.setWarning(m_env.getNullStream());

        if (!exact)
            solver.setParam(IloCplex::Param::TimeLimit, 0.5);

        if (!solver.solve(MyBranchGoal(m_env, callback, m_variables, exact)))
            return {};

        auto status = solver.getCplexStatus();
        if (status != IloCplex::Status::AbortUser)
            UpdateSolution(callback, m_variables, solver);

        switch (status)
        {
        case IloCplex::Status::Optimal:      return SolutionStatus::Optimal;
        case IloCplex::Status::AbortTimeLim: return SolutionStatus::AbortedByTimelimit;
        case IloCplex::Status::AbortUser:    return SolutionStatus::AbortedByUser;
        default:                             return SolutionStatus::Unknown;
        }
    }

    ~SupportProblemModelHolder() override = default;

private:
    IloObjective m_obj;
};

SupportConstrainsGuard::SupportConstrainsGuard(SupportProblemModelHolder& model, std::set<models::IndependetSet>& sets, const IndependetSet& ind_set)
    : m_model(model)
    , m_sets(sets)
    , m_ind_set(ind_set)
    , m_expr(m_model.m_env)
{
    assert(!ind_set.empty());
    double val = static_cast<double>(ind_set.size() - 1u);
    for (auto v : ind_set)
        m_expr += m_model.m_variables[v];
    m_constrains = m_model.m_model.add(m_expr <= val);
    m_sets.emplace(ind_set);
}

SupportConstrainsGuard::~SupportConstrainsGuard()
{
    m_model.m_model.remove(m_constrains);
    m_expr.end();
    m_sets.erase(m_ind_set);
}

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

    m_vars = { m_sets.begin(), m_sets.end() };
    m_model = std::make_unique<MainProblemModelHolder>(m_graph, m_vars);
}

MainProblemModel::~MainProblemModel() = default;

MainSolution MainProblemModel::Solve() const
{
    return m_model->Solve();
}

bool MainProblemModel::AddVariables(const std::set<models::IndependetSet>& ind_sets)
{
    auto size_before = m_sets.size();
    for (const auto& ind_set : ind_sets)
    {
        if (m_sets.emplace(ind_set).second)
            m_vars.emplace_back(ind_set);
    }
    if (size_before == m_sets.size())
        return false;

    m_model = std::make_unique<MainProblemModelHolder>(m_graph, m_vars);
    std::erase_if(m_constrains, [this](auto& constr_weak) {
        auto constr = constr_weak.lock();
        if (!constr)
            return true;

        constr->OnModelUpdate(*m_model);
        return false;
    });
    return true;
}

ConstrainPtr MainProblemModel::AddConstrain(size_t index, int type)
{
    auto constr = std::make_shared<ConstrainsGuard>(index, type);
    constr->OnModelUpdate(*m_model);
    m_constrains.emplace_back(constr);
    return constr;
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

SupportProblemModel::SupportProblemModel(const Graph& inv_graph)
    : m_graph(inv_graph)
{
    AddNonEdgePairs(m_graph, m_sets);
    for (auto type : g_strategies)
        AddPerColorConstrains(m_graph, m_sets, type);

    m_model = std::make_unique<SupportProblemModelHolder>(m_graph, m_sets);
}

SupportProblemModel::~SupportProblemModel() = default;

SolutionStatus SupportProblemModel::Solve(const SolutionCallback& callback, const Variables& weights, bool exact)
{
    return m_model->Solve(callback, weights, exact);
}

ConstrainPtr SupportProblemModel::AddConstrain(std::set<models::IndependetSet>& sets, const IndependetSet& ind_set)
{
    return std::make_unique<SupportConstrainsGuard>(*m_model, sets, ind_set);
}

};

