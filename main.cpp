//Inclusión de librerías.
// #include <bits/stdc++.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <stack>

// --------------------------------------------------

#define edge std::pair<int, int>

// Constructor from an adjacency matrix.
// Time complexity: O(v^2)
std::vector<std::pair<int, edge> > edges(std::vector< std::vector<int> > matrix) {
    std::vector<std::pair<int, edge> > edges;
    int numV = matrix.size();
    for (int i = 0; i < numV; i++) {
        for (int j = 0; j < numV; j++) {
            if (matrix[i][j] != 0) {
                edges.push_back(std::make_pair(matrix[i][j], edge(i, j)));
            }
        }
    }
    return edges;
}

// Finds the set of a vertex.
// Time complexity: O(1)
int findSet(int i, std::vector<int> parent) {
    // Recursive function to find the parent of a node.

    // Base case: if the node is its own parent, return it.
    if (i == parent[i]) {
        return i;
    } else {
    // Recursive case: if the node is not its own parent, find the parent of the parent.
        return findSet(parent[i], parent);
    }
}

// Kruskal's algorithm for finding the minimum spanning tree (MST).
// Time complexity: O(E log E)
std::vector<std::pair<int, edge> > kruskal(std::vector<std::pair<int, edge> > edges, int numV) {
    std::vector<int> parent;
    parent.resize(numV);
    for (int i = 0; i < numV; i++) {
        parent[i] = i;
    }
    std::vector<std::pair<int, edge> > mst;
    int i, setV1, setV2;
    std::sort(edges.begin(), edges.end());  // Sort edges by weight in ascending order.
    for (i = 0; i < edges.size(); i++) { // Iterate through all the edges.
        // edges[i][0] = weight
        // edges[i][1] = edge
        setV1 = findSet(edges[i].second.first, parent);
        setV2 = findSet(edges[i].second.second, parent);

        // If there are no cycles (the nodes are not in the same set), add the edge to the MST.
        if (setV1 != setV2) {
            mst.push_back(edges[i]);  // Add to tree.
            parent[setV1] = parent[setV2]; // Link the nodes' sets.
        }
    }
    return mst;
}

void printMST( std::vector<std::pair<int, edge> > mst, int numV) {
    // matrix of the MST
    std::vector<std::vector<int> > matrix(numV, std::vector<int>());
    for (int i = 0; i < mst.size(); i++) {
        edge e = mst[i].second;
        matrix[e.first].push_back(e.second);
        matrix[e.second].push_back(e.first);
    }

    int current = 0;
    while (matrix[current].size() != 1 && current < matrix.size()) {
        current++;
    }

    // Print the matrix.
    std::cout << "(";
    int previous;
    for (int i = 0; i < matrix.size() - 1; i++) {
        std::cout << current << ", ";
        int next = matrix[current][0] != previous ? matrix[current][0] : matrix[current][1]; 
        previous = current;
        current = next;
    }

    std::cout << current << ")" << std::endl;

    // int cost = 0;
    // for (int i = 0; i < mst.size(); i++) {
    //     cost += mst[i].first;
    // }
    // std::cout << "Cost: " << cost << std::endl;
}


// --------------------------------------------------

int pathCost(std::vector<int> set, int end, std::vector< std::vector<int> > costs) {
    int size = set.size();
    // If the set has only two elements, return the distance form 0 to the end element.
    if (size == 2) {
        return costs[0][end];
    }
    // Otherwise, find the minimum cost of the set without the end element and every element of the set plus the distance from the element to the first node.
    int minCost = 0;

    // Remove the end element from the set.
    std::vector<int> newSet;
    for (int i = 0; i < size; i++) {
        if (set[i] != end) {
            newSet.push_back(set[i]);
        }
    }

    for (int i = 1; i < size - 1; i++) {
        int cost = pathCost(newSet, newSet[i], costs) + costs[newSet[i]][end];
        if (minCost == 0 || cost < minCost) {
            minCost = cost;
        }
    }
    return minCost;
}

int tsp(std::vector< std::vector<int> > matrix) {
    std::vector<int> cycleCosts;
    std::vector<int> set;
    for (int i = 0; i < matrix.size(); i++) {
        set.push_back(i);
    }
    for (int i = 1; i < matrix.size(); i++) {
        // Find minimum cost path starting from vertex 1, passing through every node once and ending at vertex i.
        // The cost of the Hamiltonian cycle is the sum of the minimum cost path and the cost of the edge from the last node to the first node.
        int totalCost = pathCost(set, i, matrix) + matrix[i][0];
        cycleCosts.push_back(totalCost);
    }
    // Find the minimum cost Hamiltonian cycle.
    sort(cycleCosts.begin(), cycleCosts.end());
    return cycleCosts[0];
}

// --------------------------------------------------

//Ajuste a estandar.
using namespace std;

//Definición de estrucutra Punto.
struct Punto {
    int x;
    int y;
};

void espacio(){ //Complejidad Computacional: O(1), es una ejecución lineal en el contenido de la función.
    cout << endl;
}

//Implementación de Búsqueda en Anchura
bool bfs(vector<vector<int>>& rMatriz, int origen, int destino, vector<int>& parentezco){ //Complejidad Computacional: O(VE^2) Siendo V la cantidad de Vértices y E la cantidad de Aristas.

    int vertices;
    queue<int> cola;

    vertices = rMatriz.size();

    vector<bool> visitados(vertices, false);

    cola.push(origen);
    visitados[origen] = true;
    parentezco[origen] = -1;

    while (!cola.empty()){
        int u;
        u = cola.front();
        cola.pop();

        for (int v = 0; v < vertices; v++){
            if (visitados[v] == false && rMatriz[u][v] > 0){
                if (v == destino){
                    parentezco[v] = u;
                    return true;
                }
                cola.push(v);
                parentezco[v] = u;
                visitados[v] = true;
            }
        }

    }

    return false;

}

//Implementación del Algoritmo Ford-Fulkerson
void fordFulkerson(vector<vector<int>>& matriz, int origen, int destino){ //Complejidad Computacional: O(EV^3) Siendo V la cantidad de Vértices y E la cantidad de Aristas.

    int u;
    int v;
    int vertices;
    int flujoMaximo;
    
    vertices = matriz.size();

    vector<vector<int>> rMatriz(vertices,vector<int>(vertices));
    vector<int> parentezco(vertices);

    for (u = 0; u < vertices; u++){
        for (v = 0; v < vertices; v++){
            rMatriz[u][v] = matriz[u][v];
        }
    }

    flujoMaximo = 0;

    while(bfs(rMatriz,0,vertices-1,parentezco)){ //Complejidad Computacional: O(VE^2) Siendo V la cantidad de Vértices y E la cantidad de Aristas.

        int trayectoriaDeFlujo;
        trayectoriaDeFlujo = INT32_MAX;

        for (v = destino; v!= origen; v = parentezco[v]){
            u = parentezco[v];
            trayectoriaDeFlujo = min(trayectoriaDeFlujo,rMatriz[u][v]);
        }

        for (v = destino; v != origen; v = parentezco[v]){
            u = parentezco[v];
            rMatriz[u][v] -= trayectoriaDeFlujo;
            rMatriz[v][u] += trayectoriaDeFlujo;
        }

        flujoMaximo += trayectoriaDeFlujo;

    }

    espacio();
    cout << "Flujo Maximo: " << flujoMaximo << endl;
    espacio();

}

//Punto global que funciona como auxiliar para realizar un ordenamiento.
Punto punto0;

Punto siguientoAlTop(stack<Punto>& pila) { //Complejidad Computacional: O(1), es una ejecución lineal en el contenido de la función.
    Punto punto;
	Punto resultado;

	punto = pila.top();
	pila.pop();
	resultado = pila.top();
	pila.push(punto);

	return resultado; 
}

void intercambio(Punto& punto1, Punto& punto2) { //Complejidad Computacional: O(1), es una ejecución lineal en el contenido de la función.

    Punto auxiliar;

	auxiliar = punto1;
	punto1 = punto2;
	punto2 = auxiliar;
}

int distanciaCuadrada(Punto punto1, Punto punto2) { //Complejidad Computacional: O(1), es una ejecución lineal en el contenido de la función.
    return (punto1.x - punto2.x) * (punto1.x - punto2.x) + (punto1.y - punto2.y) * (punto1.y - punto2.y);
}

int direccion(Punto p, Punto q, Punto r) { //Complejidad Computacional: O(1), es una ejecución lineal en el contenido de la función.
    
    int valor;

	valor = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);

	if (valor == 0) {
		return 0;
	}
	else if (valor > 0) {
		return 1;
	}
	else {
		return 2;
	}
}

int comparar(const void* voidPunto1, const void* voidPunto2) { //Complejidad Computacional: O(1), es una ejecución lineal en el contenido de la función.
    
    int sentido;
	Punto* punto1;
	Punto* punto2;

	punto1 = (Punto*)voidPunto1;
	punto2 = (Punto*)voidPunto2;
	sentido = direccion(punto0, *punto1, *punto2);

	if (sentido == 0) {
		if (distanciaCuadrada(punto0, *punto2) >= distanciaCuadrada(punto0, *punto1)) {
			return -1;
		}
		else {
			return 1;
		}
	}
	else if (sentido == 2) {
		return -1;
	}
	else {
		return 1;
	}
}

void cascaraConvexaGraham(vector<Punto>& puntos, int n) { //Complejidad Computacional: O(nlogn), siendo n la cantidad de puntos.
    int yMinima;
	int minimo;
	int tamanio;
	stack<Punto> pilaResultante;
	vector<Punto> resultado;

	yMinima = puntos[0].y;
	minimo = 0;

	for (int i = 1; i < n; i++) { //Complejidad Computacional: O(n), siendo n la cantidad de puntos.

		int y;
		y = puntos[i].y;

		if ((y < yMinima) || (yMinima == y && puntos[i].x < puntos[minimo].x)) {
			yMinima = puntos[i].y;
			minimo = i;
		}
	}

	intercambio(puntos[0], puntos[minimo]);
	punto0 = puntos[0];
	qsort(&puntos[1], n - 1, sizeof(Punto), comparar); //Complejidad Computacional: O(nlogn), siendo n la cantidad de puntos.
	tamanio = 1;

	for (int i = 1; i < n; i++) { //Complejidad Computacional: O(n), siendo n la cantidad de puntos.

		while (i < n - 1 && direccion(punto0, puntos[i], puntos[i + 1]) == 0) {
			i++;
		}

		puntos[tamanio] = puntos[i];
		tamanio++;
	}

	if (tamanio < 3) {
		espacio();
		cout << "¡¡¡Imposible la creacion de un poligono convexo!!!";
		espacio();
		espacio();
		return;
	}

	pilaResultante.push(puntos[0]);
	pilaResultante.push(puntos[1]);
	pilaResultante.push(puntos[2]);

	for (int i = 3; i < tamanio; i++) { //Complejidad Computacional: O(m), siendo m el tamaño del arreglo modificado.

		while (pilaResultante.size() > 1 && direccion(siguientoAlTop(pilaResultante), pilaResultante.top(), puntos[i]) != 2) {
			pilaResultante.pop();
		}

		pilaResultante.push(puntos[i]);
	}

	while (!pilaResultante.empty()) {
		Punto p = pilaResultante.top();
		resultado.push_back(p);
		pilaResultante.pop();
	}

	espacio();
	//cout << "POLIGONO CONVEXO MAS PEQUENIO: [PUNTOS DADOS EN SENTIDO CONTRARIO A LAS MANECILLAS DEL RELOJ]" << endl;

	for (int i = resultado.size() - 1; i >= 0; i--) { //Complejidad Computacional: O(v), siendo v el tamaño del vector resultado.
		cout << "(" << resultado[i].x << "," << resultado[i].y << ") ";
		espacio();
	}

	espacio();
}

int main() {
    
    int n; //Cantidad de Colonias en la Ciudad
    vector<vector<int>> distancias; //Distancias entre colonias.
    vector<vector<int>> flujos; //Capacidad Máxima de Flujos.
    vector<Punto> centrales; //Ubicación en un plano coordenado de las centrales.

    cout << "----- INPUT: -----" << endl;
    n = 0;
    cin >> n;

    for (int i = 0; i < n; i++) {
        vector<int> auxiliar;
        for (int j = 0; j < n; j++) {
            int temporal;
            cin >> temporal;
            auxiliar.push_back(temporal);
        }
        distancias.push_back(auxiliar);
    }

    for (int i = 0; i < n; i++) {
        vector<int> auxiliar;
        for (int j = 0; j < n; j++) {
            int temporal;
            cin >> temporal;
            auxiliar.push_back(temporal);
        }
        flujos.push_back(auxiliar);
    }

    for (int i = 0; i < n; i++) {
        string punto;
        int coma;
        Punto auxiliar;
        int temporalX;
        int temporalY;
        cin >> punto;
        punto = punto.substr(1, punto.size() - 2);
        coma = punto.find(',');
        temporalX = stoi(punto.substr(0, coma));
        temporalY = stoi(punto.substr(coma+1,punto.size()-coma));
        auxiliar.x = temporalX;
        auxiliar.y = temporalY;
        centrales.push_back(auxiliar);
    }

    espacio();

    cout << "----- OUTPUT: -----" << endl;
    cout << "--- Algoritmo de Kruskal ---" << endl;
    std::vector<std::pair<int, edge> > graph = edges(distancias);
    std::vector<std::pair<int, edge> > mst = kruskal(graph, n);
    printMST(mst, n);
    
    cout << "--- TSP (Traveling Salesman Problem) ---" << endl;
    int minHamilton = tsp(distancias);
    std::cout << "Costo Minimo: " << minHamilton << std::endl;
    cout << "--- Algoritmo de Ford-Fulkerson ---" << endl;
	fordFulkerson(flujos,0,n-1);
    cout << "--- Convex Hull - Escaneo de Graham ---" << endl;
    cascaraConvexaGraham(centrales,n);

    return 0;
}