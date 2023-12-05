// Librerias utilizadas en el programa
#include <iostream> //cin cout
#include <vector> //arreglos dinámicos
#include <queue> //implementación de colas
#include <list> //implementación de listas
#include <stack> //implementación de pilas
#include <algorithm> //ordenamiento y búsqueda,
#include <climits> // Para INT_MAX
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

     void bfs(int start) { // Recorrido BFS
        vector<bool> visited(V, false); // Inicializar vector de visitados
        queue<int> q; // Inicializar cola

        visited[start] = true; // Marcar vértice de inicio como visitado
        q.push(start); // Agregar vértice de inicio a la cola

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

class GrafoDirigido {
    int V; // Número de vértices
    list<int> *adj; // Lista de adyacencia

public:
    GrafoDirigido(int v) : V(v) {
        adj = new list<int>[V];
    }

    void agregarArista(int v, int w) {
        adj[v].push_back(w);
    }

    void BFS(int inicio) {
        // Marcadores para llevar un registro de los nodos visitados
        bool *visitado = new bool[V];
        for (int i = 0; i < V; i++)
            visitado[i] = false;

        // Cola para el BFS
        queue<int> cola;

        // Marcar el nodo actual como visitado y agregarlo a la cola
        visitado[inicio] = true;
        cola.push(inicio);

        while (!cola.empty()) {
            // Sacar un vértice de la cola e imprimirlo
            inicio = cola.front();
            cout << inicio << " ";
            cola.pop();

            // Obtener todos los vértices adyacentes al vértice sacado de la cola
            for (auto i = adj[inicio].begin(); i != adj[inicio].end(); ++i) {
                if (!visitado[*i]) {
                    visitado[*i] = true;
                    cola.push(*i);
                }
            }
        }
    }

    void DFSUtil(int v, bool visitado[]) {
        // Marcar el nodo actual como visitado e imprimirlo
        cout << v << " ";
        visitado[v] = true;

        // Recorrer todos los nodos adyacentes al nodo actual
        for (auto i = adj[v].begin(); i != adj[v].end(); ++i) {
            if (!visitado[*i]) {
                DFSUtil(*i, visitado);
            }
        }
    }

    void DFS(int inicio) {
        // Marcadores para llevar un registro de los nodos visitados
        bool *visitado = new bool[V];
        for (int i = 0; i < V; i++)
            visitado[i] = false;

        // Llamar a la función de utilidad DFS
        DFSUtil(inicio, visitado);
    }

    
};

int main() {
    setlocale(LC_ALL, "spanish"); //Comando para al momento de imprimir se puedan imprimir los acentos y las ñ's
    int numVertices, choice; // Número de vértices y elección del usuario

    cout << "Ingrese el número de vértices del grafo: "; // Obtener número de vértices
    cin >> numVertices; // Leer número de vértices

    GraphMatrix matrixGraph(numVertices); // Crear grafo
    GraphList listGraph(numVertices); // Crear grafo
    GrafoDirigido grafo(numVertices);

    cout << "Ingrese las aristas del grafo en formato 'u v peso' (0-indexed). Para finalizar, ingrese -1 -1 -1.\n"; // Obtener aristas

    int u, v, weight; // Vértices y peso de la arista
    while (true) { // Mientras no se ingrese -1 -1 -1
        cin >> u >> v >> weight; // Leer arista
        if (u == -1 || v == -1 || weight == -1) // Si se ingresa -1 -1 -1, salir del ciclo
            break;
        matrixGraph.addEdge(u, v, weight); // Agregar arista al grafo
        listGraph.addEdge(u, v, weight); // Agregar arista al grafo
        grafo.agregarArista(u, v); // Agregar arista al grafo
    }
    do { // Menú
        cin.get();
        system("clear");
        cout << "\nMenú:\n";
        cout << "1. Mostrar matriz de adyacencia\n";
        cout << "2. Mostrar lista de adyacencia\n";
        cout << "3. Pareo de un grafo normal\n";
        cout << "4. Pareo de un grafo bipartito\n";
        cout << "5. Realizar recorrido BFS Grafo Dirigido\n";
        cout << "6. Realizar recorrido DFS Grafo Dirigido\n";
        cout << "7. Comparación DFS y BFS con los algoritmos de grafos dirigidos y grafos no dirigidos\n";
        
        cout << "0. Salir\n";
        cout << "Ingrese su elección: ";
        cin >> choice;
        // Realizar la acción correspondiente a la elección del usuario
        switch (choice) {
            case 1: // Mostrar matriz de adyacencia
                cout << "Matriz de adyacencia:\n"; 
                matrixGraph.printMatrix();
                cin.get();
                break;
            case 2: // Mostrar lista de adyacencia
                cout << "Lista de adyacencia:\n";
                listGraph.printList();
                cin.get();
                break;
            case 3: 
                
                cin.get();
                break;
            case 4: 
                
                cin.get();
                break;
            case 5: 
                cout << "Ingrese el vértice de inicio para BFS: ";
                cin >> u;
                grafo.BFS(u);
                cin.get();
                break;
            case 6: 
                cout << "Ingrese el vértice de inicio para DFS: ";
                cin >> u;
                grafo.DFS(u);
                cin.get();
                break;
            case 7: 
                cout << "Ingrese el vértice de inicio para DFS y BFS: ";
                cin >> u;
                cout << "DFS Grafo No Dirigido: ";
                matrixGraph.dfs(u);
                cout << endl;
                cout << "DFS Grafo Dirigido: ";
                grafo.DFS(u);
                cout << endl;
                cout << "BFS Grafo No Dirigido: ";
                matrixGraph.bfs(u);
                cout << endl;
                cout << "BFS Grafo Dirigido: ";
                grafo.BFS(u);
                cout << endl;
                cin.get();
                break;
            
            case 0: // Salir
                cout << "Saliendo del programa. ¡Hasta luego!\n";
                system("pause");
                break;
            default: // Opción no válida
                cout << "Opción no válida. Inténtelo de nuevo.\n";
                system("pause");
        }
    } while (choice != 0); // Mientras no se ingrese 0
    return 0;
}
