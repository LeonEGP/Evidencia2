//Inclusión de librerías.
#include <bits/stdc++.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <stack>

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
    cout << "--- TSP (Traveling Salesman Problem) ---" << endl;
    cout << "--- Algoritmo de Ford-Fulkerson ---" << endl;
    cout << "--- Convex Hull - Escaneo de Graham ---" << endl;
    cascaraConvexaGraham(centrales,n);

    return 0;
}