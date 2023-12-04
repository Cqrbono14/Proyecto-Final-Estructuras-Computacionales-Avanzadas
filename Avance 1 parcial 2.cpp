// Librerias utilizadas en el programa
#include <iostream> //cin cout
#include <vector> //arreglos dinámicos
#include <queue> //implementación de colas
#include <stack> //implementación de pilas
#include <algorithm> //ordenamiento y búsqueda,
#include <climits> // Para INT_MAX
#include <conio.h> // Para hacer pausas
using namespace std; // para disminuir la cantidad de texto en el programa

// Clase Grafo
class GraphMatrix {
private:
    int V; // Número de vértices
    vector<vector< int > > graph; // Matriz de adyacencia

public:
    GraphMatrix(int vertices) : V(vertices) { // Constructor
        graph = vector< vector< int > >(V, vector<int>(V, 0)); // Inicializar matriz de adyacencia
    }

    

    void printMatrix() { // Imprimir matriz de adyacencia
        for (int i = 0; i < V; ++i) { 
            for (int j = 0; j < V; ++j) { 
                cout << graph[i][j] << " "; 
            }
            cout << endl;
        }
    }

     void addEdge(int u, int v, int weight) { // Agregar arista
        graph[u][v] = weight; // Grafo dirigido 
        graph[v][u] = weight;  // Grafo no dirigido
    }

    bool isConnected() { // Verificar si el grafo es conexo
        vector<bool> visited(V, false); // Inicializar vector de visitados
        dfs(0, visited); // Realizar recorrido DFS desde el vértice 0

        for (bool visit : visited) { // Verificar si todos los vértices fueron visitados
            if (!visit) // Si algún vértice no fue visitado, el grafo no es conexo
                return false; 
        }
        return true; // 
    }

    void bfs(int start) { // Recorrido BFS
        vector<bool> visited(V, false); // Inicializar vector de visitados
        queue<int> q; // Inicializar cola

        visited[start] = true; // Marcar vértice de inicio como visitado
        q.push(start); // Agregar vértice de inicio a la cola

        cout << "BFS Traversal: "; // Imprimir recorrido BFS
        while (!q.empty()) { // Mientras la cola no esté vacía
            int vertex = q.front(); // Obtener el primer elemento de la cola
            q.pop(); // Eliminar el primer elemento de la cola
            cout << vertex << " "; // Imprimir el vértice

            for (int i = 0; i < V; ++i) { // Recorrer los vértices adyacentes al vértice actual
                if (graph[vertex][i] && !visited[i]) { // Si el vértice adyacente no ha sido visitado
                    visited[i] = true; // Marcar vértice adyacente como visitado
                    q.push(i); // Agregar vértice adyacente a la cola
                }
            }
        }
        cout << endl;
    }

    void dfs(int start) { // Recorrido DFS
        vector<bool> visited(V, false); // Inicializar vector de visitados
        dfs(start, visited); // Realizar recorrido DFS desde el vértice 0
    }

    void dfs(int start, vector<bool> &visited) { // Recorrido DFS
        visited[start] = true;  // Marcar vértice actual como visitado
        cout << start << " "; // Imprimir vértice actual

        for (int i = 0; i < V; ++i) { // Recorrer los vértices adyacentes al vértice actual
            if (graph[start][i] && !visited[i]) { // Si el vértice adyacente no ha sido visitado
                dfs(i, visited); // Realizar recorrido DFS desde el vértice adyacente
            }
        }
    }

    void dijkstra(int start) { // Algoritmo de Dijkstra
        vector<int> dist(V, INT_MAX); // Inicializar vector de distancias
        vector<bool> inSet(V, false); // Inicializar vector de vértices visitados
        dist[start] = 0; // Distancia del vértice de inicio a sí mismo es 0

        for (int i = 0; i < V - 1; ++i) { // Recorrer todos los vértices
            int u = minDistance(dist, inSet); // Obtener el vértice con la distancia mínima
            inSet[u] = true; // Marcar vértice como visitado
            for (int v = 0; v < V; ++v) { // Recorrer los vértices adyacentes al vértice actual
                if (!inSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + graph[u][v] < dist[v]) { // Si el vértice no ha sido visitado, la distancia del vértice actual no es infinita y la distancia del vértice actual más la distancia al vértice adyacente es menor a la distancia del vértice adyacente
                    dist[v] = dist[u] + graph[u][v]; // Actualizar distancia del vértice adyacente
                }
            }
        }

        cout << "Dijkstra's Shortest Paths:\n"; // Imprimir distancias más cortas
        for (int i = 0; i < V; ++i) { 
            cout << "From " << start << " to " << i << ": " << dist[i] << endl;  
        }
    }

    void kruskal() { // Algoritmo de Kruskal
        vector<pair<int, pair<int, int>>> edges; // Inicializar vector de aristas
        for (int i = 0; i < V; ++i) { // Recorrer todos los vértices
            for (int j = i + 1; j < V; ++j) { // Recorrer los vértices adyacentes al vértice actual
                if (graph[i][j] != 0) { // Si existe una arista entre el vértice actual y el vértice adyacente
                    edges.push_back({graph[i][j], {i, j}}); // Agregar arista al vector de aristas
                }
            }
        }

        sort(edges.begin(), edges.end()); // Ordenar aristas por peso

        vector<int> parent(V); 
        for (int i = 0; i < V; ++i) { // Inicializar vector de padres
            parent[i] = i; 
        }

        cout << "Minimum Spanning Tree (Kruskal's Algorithm):\n"; // Imprimir árbol de expansión mínima
        for (auto edge : edges) { // Recorrer aristas
            int weight = edge.first; // Obtener peso de la arista
            int u = edge.second.first; // Obtener vértice u de la arista
            int v = edge.second.second; // Obtener vértice v de la arista
            int setU = find(parent, u); // Obtener conjunto de u
            int setV = find(parent, v); // Obtener conjunto de v

            if (setU != setV) { // Si u y v no pertenecen al mismo conjunto
                cout << u << " - " << v << " with weight " << weight << endl; // Imprimir arista
                unionSets(parent, setU, setV); // Unir conjuntos
            }
        }
    }

    void prim() { // Algoritmo de Prim
        vector<int> parent(V, -1); // Inicializar vector de padres
        vector<int> key(V, INT_MAX); // Inicializar vector de distancias
        vector<bool> inMST(V, false); // Inicializar vector de vértices visitados

        key[0] = 0; // Distancia del vértice de inicio a sí mismo es 0

        for (int i = 0; i < V - 1; ++i) { // Recorrer todos los vértices
            int u = minKey(key, inMST); // Obtener el vértice con la distancia mínima
            inMST[u] = true; // Marcar vértice como visitado
            for (int v = 0; v < V; ++v) { // Recorrer los vértices adyacentes al vértice actual
                if (graph[u][v] && !inMST[v] && graph[u][v] < key[v]) { // Si el vértice no ha sido visitado, la distancia del vértice actual no es infinita y la distancia del vértice actual más la distancia al vértice adyacente es menor a la distancia del vértice adyacente
                    parent[v] = u; // Actualizar padre del vértice adyacente
                    key[v] = graph[u][v]; // Actualizar distancia del vértice adyacente
                }
            }
        }

        cout << "Minimum Spanning Tree (Prim's Algorithm):\n"; // Imprimir árbol de expansión mínima
        for (int i = 1; i < V; ++i) { 
            cout << parent[i] << " - " << i << " with weight " << graph[i][parent[i]] << endl; 
        }
    }

void matching() { 		//Funcion para la opcion de pareo en un grafo normal
        vector<int> match(V, -1); // Inicializar vector de emparejamiento
        int matchingCount = 0; // Contador de emparejamiento

        for (int u = 0; u < V; ++u) {
            vector<bool> visited(V, false); // Inicializar vector de visitados
            if (bpm(u, match, visited)) {
                matchingCount++;
            }
        }

        cout << "Emparejamiento máximo: " << matchingCount / 2 << endl;
    }

private:
    int minKey(const vector<int> &key, const vector<bool> &inMST) { // Obtener el vértice con la distancia mínima
        int min = INT_MAX, min_index = -1; // Inicializar distancia mínima y vértice con distancia mínima
        for (int v = 0; v < V; ++v) { // Recorrer todos los vértices
            if (!inMST[v] && key[v] < min) { // Si el vértice no ha sido visitado y la distancia del vértice actual es menor a la distancia mínima
                min = key[v]; // Actualizar distancia mínima
                min_index = v; // Actualizar vértice con distancia mínima
            } 
        }
        return min_index; // Retornar vértice con distancia mínima
    }

    int minDistance(const vector<int> &dist, const vector<bool> &inSet) { // Obtener el vértice con la distancia mínima
        int min = INT_MAX, min_index = -1; // Inicializar distancia mínima y vértice con distancia mínima
        for (int v = 0; v < V; ++v) { // Recorrer todos los vértices
            if (!inSet[v] && dist[v] < min) { // Si el vértice no ha sido visitado y la distancia del vértice actual es menor a la distancia mínima
                min = dist[v]; // Actualizar distancia mínima
                min_index = v; // Actualizar vértice con distancia mínima
            }
        }
        return min_index; // Retornar vértice con distancia mínima
    }

    int find(vector<int> &parent, int i) { // Encontrar conjunto de un vértice
        if (parent[i] != i) // Si el padre del vértice actual no es el mismo vértice
            return find(parent, parent[i]); // Buscar conjunto del padre del vértice actual
        return i; // Retornar conjunto del vértice actual
    }

    void unionSets(vector<int> &parent, int x, int y) { // Unir conjuntos
        int rootX = find(parent, x); // Obtener conjunto de x
        int rootY = find(parent, y); // Obtener conjunto de y
        parent[rootX] = rootY; // Unir conjuntos
    }

    bool bpm(int u, vector<int>& match, vector<bool>& visited) {
        for (int v = 0; v < V; ++v) {
            if (graph[u][v] && !visited[v]) {
                visited[v] = true;
                if (match[v] == -1 || bpm(match[v], match, visited)) {
                    match[v] = u;
                    return true;
                }
            }
        }
        return false;
    }
};

class GraphList { // Clase Grafo
private:
    int V; // Número de vértices
    vector<vector<pair<int, int>>> graph; // Lista de adyacencia

public:
    GraphList(int vertices) : V(vertices), graph(vertices) { // Constructor
        for (int i = 0; i < vertices; ++i) { // Inicializar lista de adyacencia
            graph[i] = vector<pair<int, int>>(); 
        }
    }

    void addEdge(int u, int v, int weight) { // Agregar arista
        graph[u].push_back({v, weight}); // Grafo dirigido
        graph[v].push_back({u, weight});  // Grafo no dirigido
    }

    void printList() { // Imprimir lista de adyacencia
        for (int i = 0; i < V; ++i) { 
            cout << "Vertex " << i << " -> "; // Imprimir vértice
            for (const auto &pair : graph[i]) { // Recorrer los vértices adyacentes al vértice actual
                cout << "(" << pair.first << ", " << pair.second << ") "; // Imprimir vértice adyacente
            }
            cout << endl;
        }
    }
};



int main() {
    setlocale(LC_ALL, "spanish"); //Comando para al momento de imprimir se puedan imprimir los acentos y las ñ's
    int numVertices, choice; // Número de vértices y elección del usuario

    cout << "Ingrese el número de vértices del grafo: "; // Obtener número de vértices
    cin >> numVertices; // Leer número de vértices

    GraphMatrix matrixGraph(numVertices); // Crear grafo
    GraphList listGraph(numVertices); // Crear grafo

    cout << "Ingrese las aristas del grafo en formato 'u v peso' (0-indexed). Para finalizar, ingrese -1 -1 -1.\n"; // Obtener aristas

    int u, v, weight; // Vértices y peso de la arista
    while (true) { // Mientras no se ingrese -1 -1 -1
        cin >> u >> v >> weight; // Leer arista
        if (u == -1 || v == -1 || weight == -1) // Si se ingresa -1 -1 -1, salir del ciclo
            break;
        matrixGraph.addEdge(u, v, weight); // Agregar arista al grafo
        listGraph.addEdge(u, v, weight); // Agregar arista al grafo
    }
    do { // Menú
        system("cls");
        cout << "\nMenú:\n";
        cout << "1. Mostrar matriz de adyacencia\n";
        cout << "2. Mostrar lista de adyacencia\n";
        cout << "3. Verificar si el grafo es conexo\n";
        cout << "4. Realizar recorrido BFS\n";
        cout << "5. Realizar recorrido DFS\n";
        cout << "6. Aplicar el algoritmo de Dijkstra\n";
        cout << "7. Aplicar el algoritmo de Kruskal\n";
        cout << "8. Aplicar el algoritmo de Prim\n";
        cout << "9. Realizar pareo en un grafo normal\n";
        cout << "0. Salir\n";
        cout << "Ingrese su elección: ";
        cin >> choice;
        // Realizar la acción correspondiente a la elección del usuario
        switch (choice) {
            case 1: // Mostrar matriz de adyacencia
                cout << "Matriz de adyacencia:\n"; 
                matrixGraph.printMatrix();
                getch();
                break;
            case 2: // Mostrar lista de adyacencia
                cout << "Lista de adyacencia:\n";
                listGraph.printList();
                getch();
                break;
            case 3: // Verificar si el grafo es conexo
                if (matrixGraph.isConnected())
                    cout << "El grafo es conexo.\n";
                else
                    cout << "El grafo no es conexo.\n";
                getch();
                break;
            case 4: // Realizar recorrido BFS
                cout << "Ingrese el vértice de inicio para BFS: ";
                cin >> u;
                matrixGraph.bfs(u);
                getch();
                break;
            case 5: // Realizar recorrido DFS
                cout << "Ingrese el vértice de inicio para DFS: ";
                cin >> u;
                cout << "DFS Traversal: ";
                matrixGraph.dfs(u);
                cout << endl;
                getch();
                break;
            case 6: // Aplicar el algoritmo de Dijkstra
                cout << "Ingrese el vértice de inicio para Dijkstra: ";
                cin >> u;
                matrixGraph.dijkstra(u);
                getch();
                break;
            case 7: // Aplicar el algoritmo de Kruskal
                matrixGraph.kruskal();
                getch();
                break;
            case 8: // Aplicar el algoritmo de Prim
                matrixGraph.prim();
                getch();
                break;
            case 9: // opcion de pareo en un grafo normal
	            matrixGraph.matching();
	            getch();
	            break;
            case 0: // Salir
                cout << "Saliendo del programa. ¡Hasta luego!\n";
                getch();
                break;
            default: // Opción no válida
                cout << "Opción no válida. Inténtelo de nuevo.\n";
                getch();
        }
    } while (choice != 0); // Mientras no se ingrese 0
    return 0;
}
