#ifndef IL_STD
#define IL_STD
#endif
#include <ilcplex/ilocplex.h>

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
    {
        uint32_t k = 0u;
        for (const auto& ind_set : vars)
        {
            for (auto v : ind_set)
                m_expressions[v] += m_variables[k];
            ++k;
        }

        for (size_t i = 0; i < vars.size(); ++i)
        {
            std::string name = "constr[" + std::to_string(i) + "]";
            m_constrains[i] = IloRange(m_env, 0, m_expressions[i], 1, name.c_str());
        }
        m_model.add(m_constrains);
    }

    ~MainProblemModelHolder()
    {
        //m_constrains.end();
        //m_variables.end();
        //m_model.end();
        m_env.end();
    }

private:
    IloEnv          m_env{};
    IloModel        m_model;
    IloNumVarArray  m_variables;
    IloExprArray    m_expressions;
    IloRangeArray   m_constrains;
};

MainProblemModel::MainProblemModel(const Graph& graph)
    : m_graph(graph)
{
    for (auto type : g_strategies)
        m_sets += m_graph.GetHeuristicConstr(type);

    m_model = std::make_unique<MainProblemModelHolder>(m_graph, m_sets);
}

MainProblemModel::~MainProblemModel()
{

}

};

