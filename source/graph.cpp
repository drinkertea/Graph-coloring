#include <sstream>
#include <fstream>
#include <algorithm>
#include <random>
#include <numeric>
#include "graph.h"
#include "common.h"

static uint32_t seed = 42;

Graph::Graph(const std::string& path)
{
    std::ifstream infile(path);
    if (!infile)
        throw std::runtime_error("Invalid file path!");

    std::string line;
    uint32_t vertex_count = 0u;
    uint32_t edge_count = 0u;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        char type = 0;
        if (!(iss >> type))
            continue;

        if (type == 'c')
        {
            continue;
        }
        else if (type == 'p')
        {
            std::string name;
            if (!(iss >> name >> vertex_count >> edge_count))
                continue;

            m_graph.resize(vertex_count, std::vector<bool>(vertex_count, false));
            m_adj.resize(vertex_count);
            m_random_metrics.resize(vertex_count);
        }
        else if (type == 'e')
        {
            uint32_t a = 0u;
            uint32_t b = 0u;
            if (!(iss >> a >> b))
                continue;

            if (!a || !b)
                throw std::runtime_error("Unexpected vertex index!");

            m_graph[a - 1u][b - 1u] = true;
            m_graph[b - 1u][a - 1u] = true;

            m_adj[a - 1u].emplace(b - 1u);
            m_adj[b - 1u].emplace(a - 1u);
        }
        else
        {
            throw std::runtime_error("Invalid file format!");
        }
    }

    m_non_adj.resize(m_graph.size());
    for (uint32_t i = 0; i < m_graph.size(); ++i)
    {
        for (uint32_t j = i + 1; j < m_graph.size(); ++j)
        {
            if (m_graph[i][j])
                continue;

            m_non_adj[i].emplace(j);
            m_non_adj[j].emplace(i);
        }
    }

    for (auto& rm : m_random_metrics)
    {
        auto ordered_nodes = GetOrderedNodes(ColorizationType::mindegree_random);
        rm.resize(m_graph.size());
        uint32_t order = 0;
        for (auto node : ordered_nodes)
            rm[node] = order++;
    }

    density = double(2 * edge_count) / double(vertex_count * (vertex_count - 1));
}

Graph::ColorToVerts Graph::Colorize(ColorizationType type) const
{
    auto n = m_graph.size();
    constexpr uint32_t c_no_color = std::numeric_limits<uint32_t>::max();

    ColorToVerts result;
    std::vector<bool> available(n, false);

    auto nodes = GetOrderedNodes(type);

    std::map<uint32_t, uint32_t> colors = { { nodes.front(), 0 } };

    for (const auto& node : nodes)
    {
        for (const auto& adj : m_adj[node])
            if (colors.count(adj))
                available[colors[adj]] = true;

        uint32_t cr = 0;
        for (; cr < n; cr++)
            if (available[cr] == false)
                break;

        colors[node] = cr;
        result[cr].emplace_back(node);

        for (const auto& adj : m_adj[node])
            if (colors.count(adj))
                available[colors[adj]] = false;
    }

    return result;
}

std::vector<uint32_t> Graph::GetOrderedNodes(ColorizationType type) const
{
    std::vector<uint32_t> nodes(m_graph.size(), 0);
    std::iota(nodes.begin(), nodes.end(), 0);

    std::vector<uint32_t> random_metric(m_graph.size(), 0);
    std::iota(random_metric.begin(), random_metric.end(), 0);
    std::mt19937 g(seed++);
    std::shuffle(random_metric.begin(), random_metric.end(), g);

    if (type == ColorizationType::maxdegree)
    {
        std::sort(nodes.begin(), nodes.end(), [&](const auto& l, const auto& r) {
            return GetDegree(l) > GetDegree(r);
        });
    }
    else if (type == ColorizationType::mindegree)
    {
        std::sort(nodes.begin(), nodes.end(), [&](const auto& l, const auto& r) {
            return GetDegree(l) < GetDegree(r);
        });
    }
    else if (type == ColorizationType::random)
    {
        std::shuffle(nodes.begin(), nodes.end(), g);
    }
    else if (type == ColorizationType::maxdegree_random)
    {
        std::sort(nodes.begin(), nodes.end(), [&](const auto& l, const auto& r) -> bool {
            if (GetDegree(l) == GetDegree(r))
                return random_metric[l] > random_metric[r];
            return GetDegree(l) > GetDegree(r);
        });
    }
    else if (type == ColorizationType::mindegree_random)
    {
        std::sort(nodes.begin(), nodes.end(), [&](const auto& l, const auto& r) -> bool {
            if (GetDegree(l) == GetDegree(r))
                return random_metric[l] > random_metric[r];
            return  GetDegree(l) < GetDegree(r);
        });
    }

    return nodes;
}

struct LocalSearchHelper
{
    const std::vector<std::set<WeightNode>>& non_adj_sorted;
    std::mt19937 g{ seed++ };

    void Search(const std::vector<uint32_t>& constr, const std::vector<double>& weights, uint32_t n, const std::function<void(std::vector<uint32_t>&&)>& callback)
    {
        constexpr uint32_t k = 3;
        if (constr.size() <= k * 2 + 1)
            return;

        auto nodes = constr;
        std::array<uint32_t, k> banned;
        for (uint32_t i = 0; i < k; ++i)
        {
            std::uniform_int_distribution<size_t> dist(0, nodes.size() - 1);
            auto val = dist(g);
            std::swap(nodes[val], nodes.back());
            banned[i] = nodes.back();
            nodes.pop_back();
        }

        auto t = non_adj_sorted[nodes.front()];
        double weight = weights[nodes.front()];
        for (uint32_t i = 1; i < nodes.size(); ++i)
        {
            t *= non_adj_sorted[nodes[i]];
            weight += weights[nodes[i]];
        }
        std::erase_if(t, [&banned](const auto& v) {
            return banned[0] == v.label || banned[1] == v.label || banned[2] == v.label;
        });

        if (t.size() <= k)
            return;

        for (const auto& start_node : t)
        {
            auto start = start_node.label;
            auto tt = t * non_adj_sorted[start];
            std::vector<uint32_t> new_nodes = { start };
            double add_weight = weights[start];

            while (!tt.empty())
            {
                auto node = tt.begin()->label;
                add_weight += tt.begin()->weight;
                new_nodes.emplace_back(node);

                tt *= non_adj_sorted[node];
            }

            if (EpsValue(weight + add_weight) <= 1.0)
                continue;

            if (new_nodes.size() + nodes.size() < constr.size())
                continue;

            new_nodes.insert(new_nodes.end(), nodes.begin(), nodes.end());
            callback(std::move(new_nodes));
        }
    }
};

std::vector<std::set<WeightNode>> Graph::GetWeightlyNonAdj(uint32_t start, const std::vector<double>& weights) const
{
    std::vector<std::set<WeightNode>> non_adj_sorted(m_graph.size());
    for (uint32_t i = 0; i < m_graph.size(); ++i)
    {
        for (auto j : m_non_adj[i])
        {
            non_adj_sorted[i].emplace(j, weights[j], m_random_metrics[start][j]);
            non_adj_sorted[j].emplace(i, weights[i], m_random_metrics[start][i]);
        }
    }
    return non_adj_sorted;
}

void Graph::GetWeightHeuristicConstrFor(
    uint32_t start,
    const std::vector<double>& weights,
    const std::function<void(std::vector<uint32_t>&&)>& callback
) const
{
    auto non_adj = GetWeightlyNonAdj(start, weights);
    std::set<WeightNode> nodes = non_adj[start];

    std::vector<uint32_t> constr;
    constr.reserve(m_graph.size());
    LocalSearchHelper lsh{ non_adj };
    while (!nodes.empty())
    {
        auto t = non_adj[start] * non_adj[nodes.begin()->label];
        nodes.erase(nodes.begin());

        constr.clear();
        constr.emplace_back(start);
        double weight = weights[start];
        while (!t.empty())
        {
            auto node = t.begin()->label;
            weight += t.begin()->weight;

            t *= non_adj[node];

            constr.emplace_back(node);
        }

        if (EpsValue(weight) <= 1.0)
            continue;

        lsh.Search(constr, weights, 10, callback);
        callback(std::move(constr));
    }
}