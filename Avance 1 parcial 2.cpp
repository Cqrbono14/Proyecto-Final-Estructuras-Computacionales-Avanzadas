#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <climits> // Para INT_MAX

using namespace std;

class GraphMatrix {
private:
    int V;
    vector<vector< int > > graph;

public:
    GraphMatrix(int vertices) : V(vertices) {
        graph = vector< vector< int > >(V, vector<int>(V, 0));
    }

    void addEdge(int u, int v, int weight) {
        graph[u][v] = weight;
        graph[v][u] = weight;  // Grafo no dirigido
    }

    void printMatrix() {
        for (int i = 0; i < V; ++i) {
            for (int j = 0; j < V; ++j) {
                cout << graph[i][j] << " ";
            }
            cout << endl;
        }
    }

    bool isConnected() {
        vector<bool> visited(V, false);
        dfs(0, visited);

        for (bool visit : visited) {
            if (!visit)
                return false;
        }
        return true;
    }

    void bfs(int start) {
        vector<bool> visited(V, false);
        queue<int> q;

        visited[start] = true;
        q.push(start);

        cout << "BFS Traversal: ";
        while (!q.empty()) {
            int vertex = q.front();
            q.pop();
            cout << vertex << " ";

            for (int i = 0; i < V; ++i) {
                if (graph[vertex][i] && !visited[i]) {
                    visited[i] = true;
                    q.push(i);
                }
            }
        }
        cout << endl;
    }

    void dfs(int start) {
        vector<bool> visited(V, false);
        dfs(start, visited);
    }

    void dfs(int start, vector<bool> &visited) {
        visited[start] = true;
        cout << start << " ";

        for (int i = 0; i < V; ++i) {
            if (graph[start][i] && !visited[i]) {
                dfs(i, visited);
            }
        }
    }

    void dijkstra(int start) {
        vector<int> dist(V, INT_MAX);
        vector<bool> inSet(V, false);
        dist[start] = 0;

        for (int i = 0; i < V - 1; ++i) {
            int u = minDistance(dist, inSet);
            inSet[u] = true;
            for (int v = 0; v < V; ++v) {
                if (!inSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + graph[u][v] < dist[v]) {
                    dist[v] = dist[u] + graph[u][v];
                }
            }
        }

        cout << "Dijkstra's Shortest Paths:\n";
        for (int i = 0; i < V; ++i) {
            cout << "From " << start << " to " << i << ": " << dist[i] << endl;
        }
    }

    void kruskal() {
        vector<pair<int, pair<int, int>>> edges;
        for (int i = 0; i < V; ++i) {
            for (int j = i + 1; j < V; ++j) {
                if (graph[i][j] != 0) {
                    edges.push_back({graph[i][j], {i, j}});
                }
            }
        }

        sort(edges.begin(), edges.end());

        vector<int> parent(V);
        for (int i = 0; i < V; ++i) {
            parent[i] = i;
        }

        cout << "Minimum Spanning Tree (Kruskal's Algorithm):\n";
        for (auto edge : edges) {
            int weight = edge.first;
            int u = edge.second.first;
            int v = edge.second.second;
            int setU = find(parent, u);
            int setV = find(parent, v);

            if (setU != setV) {
                cout << u << " - " << v << " with weight " << weight << endl;
                unionSets(parent, setU, setV);
            }
        }
    }

    void prim() {
        vector<int> parent(V, -1);
        vector<int> key(V, INT_MAX);
        vector<bool> inMST(V, false);

        key[0] = 0;

        for (int i = 0; i < V - 1; ++i) {
            int u = minKey(key, inMST);
            inMST[u] = true;
            for (int v = 0; v < V; ++v) {
                if (graph[u][v] && !inMST[v] && graph[u][v] < key[v]) {
                    parent[v] = u;
                    key[v] = graph[u][v];
                }
            }
        }

        cout << "Minimum Spanning Tree (Prim's Algorithm):\n";
        for (int i = 1; i < V; ++i) {
            cout << parent[i] << " - " << i << " with weight " << graph[i][parent[i]] << endl;
        }
    }

private:
    int minKey(const vector<int> &key, const vector<bool> &inMST) {
        int min = INT_MAX, min_index = -1;
        for (int v = 0; v < V; ++v) {
            if (!inMST[v] && key[v] < min) {
                min = key[v];
                min_index = v;
            }
        }
        return min_index;
    }

    int minDistance(const vector<int> &dist, const vector<bool> &inSet) {
        int min = INT_MAX, min_index = -1;
        for (int v = 0; v < V; ++v) {
            if (!inSet[v] && dist[v] < min) {
                min = dist[v];
                min_index = v;
            }
        }
        return min_index;
    }

    int find(vector<int> &parent, int i) {
        if (parent[i] != i)
            return find(parent, parent[i]);
        return i;
    }

    void unionSets(vector<int> &parent, int x, int y) {
        int rootX = find(parent, x);
        int rootY = find(parent, y);
        parent[rootX] = rootY;
    }
};

class GraphList {
private:
    int V;
    vector<vector<pair<int, int>>> graph;

public:
    GraphList(int vertices) : V(vertices), graph(vertices) {
        for (int i = 0; i < vertices; ++i) {
            graph[i] = vector<pair<int, int>>();
        }
    }

    void addEdge(int u, int v, int weight) {
        graph[u].push_back({v, weight});
        graph[v].push_back({u, weight});  // Grafo no dirigido
    }

    void printList() {
        for (int i = 0; i < V; ++i) {
            cout << "Vertex " << i << " -> ";
            for (const auto &pair : graph[i]) {
                cout << "(" << pair.first << ", " << pair.second << ") ";
            }
            cout << endl;
        }
    }
};



int main() {
    int numVertices, choice;

    cout << "Ingrese el número de vértices del grafo: ";
    cin >> numVertices;

    GraphMatrix matrixGraph(numVertices);
    GraphList listGraph(numVertices);

    cout << "Ingrese las aristas del grafo en formato 'u v peso' (0-indexed). Para finalizar, ingrese -1 -1 -1.\n";

    int u, v, weight;
    while (true) {
        cin >> u >> v >> weight;
        if (u == -1 || v == -1 || weight == -1)
            break;
        matrixGraph.addEdge(u, v, weight);
        listGraph.addEdge(u, v, weight);
    }

    do {
        cout << "\nMenú:\n";
        cout << "1. Mostrar matriz de adyacencia\n";
        cout << "2. Mostrar lista de adyacencia\n";
        cout << "3. Verificar si el grafo es conexo\n";
        cout << "4. Realizar recorrido BFS\n";
        cout << "5. Realizar recorrido DFS\n";
        cout << "6. Aplicar el algoritmo de Dijkstra\n";
        cout << "7. Aplicar el algoritmo de Kruskal\n";
        cout << "8. Aplicar el algoritmo de Prim\n";
        cout << "0. Salir\n";
        cout << "Ingrese su elección: ";
        cin >> choice;

        switch (choice) {
            case 1:
                cout << "Matriz de adyacencia:\n";
                matrixGraph.printMatrix();
                break;
            case 2:
                cout << "Lista de adyacencia:\n";
                listGraph.printList();
                break;
            case 3:
                if (matrixGraph.isConnected())
                    cout << "El grafo es conexo.\n";
                else
                    cout << "El grafo no es conexo.\n";
                break;
            case 4:
                cout << "Ingrese el vértice de inicio para BFS: ";
                cin >> u;
                matrixGraph.bfs(u);
                break;
            case 5:
                cout << "Ingrese el vértice de inicio para DFS: ";
                cin >> u;
                cout << "DFS Traversal: ";
                matrixGraph.dfs(u);
                cout << endl;
                break;
            case 6:
                cout << "Ingrese el vértice de inicio para Dijkstra: ";
                cin >> u;
                matrixGraph.dijkstra(u);
                break;
            case 7:
                matrixGraph.kruskal();
                break;
            case 8:
                matrixGraph.prim();
                break;
            case 0:
                cout << "Saliendo del programa. ¡Hasta luego!\n";
                break;
            default:
                cout << "Opción no válida. Inténtelo de nuevo.\n";
        }
    } while (choice != 0);

    return 0;
}