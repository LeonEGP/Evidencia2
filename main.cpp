
#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>

using namespace std;

//Definición de estrucutra Punto.
struct Punto {
    int x;
    int y;
};

//Función main que ejecuta el programa, no recibe parámetros, retorna un entero [return 0].
int main() {
    
    int n; //Cantidad de Colonias en la Ciudad
    vector<vector<int>> distancias; //Distancias entre colonias.
    vector<vector<int>> flujos; //Capacidad Máxima de Flujos.
    vector<Punto> centrales; //Ubicación en un plano coordenado de las centrales.

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

    return 0;
}