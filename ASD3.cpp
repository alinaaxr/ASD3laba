#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <random>
#include <cstdlib> 
#include <ctime>

using namespace std;
const int INF = numeric_limits<int>::max();

struct Edge {
    int ishodvertex, naznachvertex, weight;
};

class Graph {
public:
    int V;  //ко-во вершин
    Graph(int vertices) : V(vertices) {}

    vector<Edge> edges;
    void addEdge(int ishodvertex, int naznachvertex, int weight) {
        edges.push_back({ ishodvertex, naznachvertex, weight });
    }
};

Graph CreateGraph(int V, int maxWeight = 20) {
    Graph graph(V);
    srand(time(0)); 

    for (int u = 0; u < V; ++u) {    //u - выход из вершины, v - вход в вершину
        for (int v = 0; v < V; ++v) {
            if (u != v) {   //чтобы не было петель
                int weight = rand() % (maxWeight + 1);  //диапазон от 0 до max
                graph.addEdge(u, v, weight);
            }
        }
    }

    return graph;
}

//Беллман-Форд
bool BellmanFord(const Graph& graph, int ishodvertex, vector<int>& dist) {
    int V = graph.V;
    dist.assign(V, INF);
    dist[ishodvertex] = 0;

    for (int i = 1; i <= V - 1; ++i) {
        for (const auto& edge : graph.edges) {
            int u = edge.ishodvertex;
            int v = edge.naznachvertex;
            int weight = edge.weight;

            if (dist[u] != INF && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
            }
        }
    }
    //проверка на отриц циклы
    for (const auto& edge : graph.edges) {  //если после V-1 итерации еще можно
        int u = edge.ishodvertex;                  //ослабить веса, то значит у нас цикл с отриц весом
        int v = edge.naznachvertex;
        int weight = edge.weight;

        if (dist[u] != INF && dist[u] + weight < dist[v]) {
            return false;
        }
    }

    return true;
}

// Реализация алгоритма Дейкстры с использованием priority_queue
void Deikstra(const Graph& graph, int ishodvertex, const vector<vector<int>>& adjMatrix, vector<int>& dist) {
    int V = graph.V;
    dist.assign(V, INF);
    dist[ishodvertex] = 0;

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;  //пара расстояние-вершина
    pq.push({ 0, ishodvertex });

    while (!pq.empty()) {
        int u = pq.top().second;
        int current_dist = pq.top().first;  //с наим расст
        pq.pop();

        if (current_dist > dist[u]) continue;

        for (int v = 0; v < V; ++v) {
            if (adjMatrix[u][v] != INF) {
                int weight = adjMatrix[u][v];

                if (dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    pq.push({ dist[v], v });
                }
            }
        }
    }
}

// Алгоритм Джонсона
vector<vector<int>> Johnson(Graph& graph) {
    int V = graph.V;

    Graph modifiedGraph(V + 1);

    for (const auto& edge : graph.edges) {
        modifiedGraph.addEdge(edge.ishodvertex, edge.naznachvertex, edge.weight);
    }

    int s = V;
    for (int v = 0; v < V; ++v) {  //добавление ребер из новой s в каждую вершину с весом 0
        modifiedGraph.addEdge(s, v, 0);
    }

    vector<int> h(V + 1);
    if (!BellmanFord(modifiedGraph, s, h)) {
        cout << "Входной граф содержит цикл с отрицательным весом" << endl;
        return {};
    }

    vector<vector<int>> adjMatrix(V, vector<int>(V, INF));  //матрица смежности
    for (const auto& edge : graph.edges) {
        int u = edge.ishodvertex;
        int v = edge.naznachvertex;
        adjMatrix[u][v] = edge.weight + h[u] - h[v];
        if (adjMatrix[u][v] < 0) {
            cout << "Отрицательное значение" << endl;
        }
    }

    vector<vector<int>> dist(V, vector<int>(V, INF));

    for (int u = 0; u < V; ++u) {
        Deikstra(graph, u, adjMatrix, dist[u]);

        for (int v = 0; v < V; ++v) {
            if (dist[u][v] != INF) {
                dist[u][v] = dist[u][v] + h[v] - h[u];
            }
        }
    }

    return dist;
}


int main() {
    setlocale(LC_ALL, "RUS");

    int V = 5;
    Graph graph = CreateGraph(V);

    for (const auto& edge : graph.edges) {
        cout << edge.ishodvertex << " -> " << edge.naznachvertex << " : " << edge.weight << endl;
    }
    cout << endl;

    vector<vector<int>> shortestPaths = Johnson(graph);

    if (!shortestPaths.empty()) {
        cout << "Матрица кратчайших путей между всеми вершинами:" << endl;
        cout << "    ";
        for (int v = 0; v < V; ++v) {
            cout << v << "\t";
        }
        cout << endl;

        for (int u = 0; u < V; ++u) {
            cout << u << " | ";
            for (int v = 0; v < V; ++v) {
                if (u == v) {
                    cout << "0\t";
                }
                else if (shortestPaths[u][v] == INF) {
                    cout << "INF\t";
                }
                else {
                    cout << shortestPaths[u][v] << "\t";
                }
            }
            cout << endl;
        }
    }

    return 0;
}