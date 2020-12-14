#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <set>
#include <map>
#include <algorithm>
#include <numeric>
#include <functional>

template <typename T>
std::set<T> operator*(const std::set<T>& A, const std::set<T>& B)
{
    std::set<T> res;
    std::set_intersection(A.begin(), A.end(), B.begin(), B.end(), inserter(res, res.begin()));
    return res;
}

template <typename T>
std::set<T>& operator+=(std::set<T>& A, const std::set<T>& B)
{
    for (const auto& x : B)
        A.insert(x);
    return A;
}

template <typename T>
std::set<T>& operator*=(std::set<T>& A, const std::set<T>& B)
{
    std::erase_if(A, [&B](const auto& x) {
        return !B.count(x);
    });
    return A;
}

struct Graph
{
    using ColorToVerts = std::map<uint32_t, std::vector<uint32_t>>;

    explicit Graph(const std::string& path);

    explicit Graph(size_t size)
    {
        m_graph.resize(size, std::vector<bool>(size, false));
        m_adj.resize(size);
        m_random_metrics.resize(size);
    }

    enum class ColorizationType
    {
        Default = 0,
        mindegree,
        maxdegree,
        random,
        mindegree_random,
        maxdegree_random,
    };

    ColorToVerts Colorize(ColorizationType type) const;

    size_t GetSize() const
    {
        return m_graph.size();
    }

    uint32_t GetDegree(uint32_t vertex) const
    {
        return static_cast<uint32_t>(m_adj[vertex].size());
    }

    const std::set<uint32_t>& GetNeighbors(uint32_t vertex) const
    {
        return m_adj[vertex];
    }

    bool HasEdge(uint32_t i, uint32_t j) const
    {
        return m_graph[i][j];
    }

    std::set<std::set<uint32_t>> GetHeuristicConstr(ColorizationType type) const
    {
        return GetHeuristicConstr(GetOrderedNodes(type));
    }

    double GetDensity() const
    {
        return density;
    }

    Graph GetSubGraph(uint32_t vertex) const;
    Graph GetInversed() const;

    void AdjustIndSet(std::set<uint32_t>& ind_set) const
    {
        auto it = ind_set.begin();
        std::set<uint32_t> left = m_non_adj[*it++];
        while (it != ind_set.end())
            left *= m_non_adj[*it++];

        while (!left.empty())
        {
            auto node = *left.begin();
            left *= m_non_adj[node];
            ind_set.emplace(node);
        }
    }

    std::set<uint32_t> AdjustIndSet(const std::set<uint32_t>& ind_set) const
    {
        std::set<uint32_t> res = ind_set;
        AdjustIndSet(res);
        return res;
    }

    void GetWeightHeuristicConstrFor(
        size_t start,
        const std::vector<double>& weights,
        const std::function<void(std::vector<uint32_t>&&)>& callback
    ) const;

private:
    struct WeightNode
    {
        WeightNode(uint32_t l, double w, uint32_t ro)
            : label(l)
            , weight(w)
            , eps100w(uint32_t(w * 100))
            , random_order(ro)
        {
        }

        bool operator<(const WeightNode& r) const
        {
            return std::tie(eps100w, random_order, weight, label) >
                std::tie(r.eps100w, r.random_order, r.weight, r.label);
        }

        uint32_t label = 0;
        double   weight = 0.0;

    private:
        uint32_t eps100w = 0;
        uint32_t random_order = 0;
    };

    std::vector<uint32_t>             GetOrderedNodes(ColorizationType type) const;
    std::set<std::set<uint32_t>>      GetHeuristicConstr(const std::vector<uint32_t>& ordered_nodes) const;
    std::vector<std::set<WeightNode>> GetWeightlyNonAdj(size_t start, const std::vector<double>& weights) const;
    void                              Finalize();

private:
    std::vector<std::vector<bool>>  m_graph;
    std::vector<std::set<uint32_t>> m_adj;
    std::vector<std::set<uint32_t>> m_non_adj;

    std::vector<std::vector<uint32_t>> m_random_metrics;
    double density = 0.0;
};
