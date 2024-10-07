#include <iostream>
#include <vector>
#include <stack>
#include <limits>
#include <string>
#include <cmath>     // For mathematical functions
#include <sstream>
#include <fstream>   // For file I/O (ifstream, ofstream)

using namespace std;

struct Sensor {
    double x, y;    // Coordinates where the sensor is placed
    double Tx;      // Transmission range means how far it can transmit
    double speed;   // How fast it can transmit data
};

double euclideanDistance(const Sensor &s1, const Sensor &s2) {
    return sqrt(pow(s2.x - s1.x, 2) + pow(s2.y - s1.y, 2));
}

// Function which reads data from a data file
vector<Sensor> filedata(const string& filename) {
    vector<Sensor> sensors; // Vector where all sensors with their parameters will be saved
    ifstream file(filename); // Predefined method used to open the file
    string line; // Hold temporarily each line of text from file when it is read

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        Sensor sensor;

        // Read and parse each value separated by commas
        std::getline(ss, token, ',');
        sensor.x = std::stod(token);

        std::getline(ss, token, ',');
        sensor.y = std::stod(token);

        std::getline(ss, token, ',');
        sensor.Tx = std::stod(token);

        std::getline(ss, token, ',');
        sensor.speed = std::stod(token);

        sensors.push_back(sensor);
    }
    return sensors;
}

// Function to create the adjacency matrix
vector<vector<double>> AdjacencyMatrix(const vector<Sensor>& sensors) {
    int n = sensors.size();
    vector<vector<double>> adjMatrix(n, vector<double>(n, numeric_limits<double>::infinity()));

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i != j) {
                double distance = euclideanDistance(sensors[i], sensors[j]);
                if (distance <= sensors[i].Tx) {
                    adjMatrix[i][j] = distance / sensors[i].speed;
                }
            }
        }
    }
    return adjMatrix;
}

// Function to print the adjacency matrix
void printAdjacencyMatrix(const vector<vector<double>>& adjMatrix) {
    int n = adjMatrix.size();
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (adjMatrix[i][j] != numeric_limits<double>::infinity())
                cout << adjMatrix[i][j] << "\t";
            else
                cout << "@\t";
        }
        cout << endl;
    }
}

// Kosaraju's algorithm to find SCCs
void dfs(int node, const vector<vector<double>>& adjMatrix, vector<bool>& visited, stack<int>& Stack) {
    visited[node] = true;
    for (int i = 0; i < adjMatrix.size(); i++) {
        if (adjMatrix[node][i] != numeric_limits<double>::infinity() && !visited[i]) {
            dfs(i, adjMatrix, visited, Stack);
        }
    }
    Stack.push(node);
}

void reverseDfs(int node, const vector<vector<double>>& reverseAdjMatrix, vector<bool>& visited, vector<int>& component) {
    visited[node] = true;
    component.push_back(node);
    for (int i = 0; i < reverseAdjMatrix.size(); i++) {
        if (reverseAdjMatrix[node][i] != numeric_limits<double>::infinity() && !visited[i]) {
            reverseDfs(i, reverseAdjMatrix, visited, component);
        }
    }
}

// Function to find SCCs using Kosaraju's algorithm
void findSCCs(const vector<vector<double>>& adjMatrix) {
    int n = adjMatrix.size();
    vector<bool> visited(n, false);
    stack<int> Stack;

    // Step 1: Fill vertices in stack according to their finishing times
    for (int i = 0; i < n; i++) {
        if (!visited[i]) {
            dfs(i, adjMatrix, visited, Stack);
        }
    }

    // Step 2: Transpose the graph
    vector<vector<double>> reverseAdjMatrix(n, vector<double>(n, numeric_limits<double>::infinity()));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (adjMatrix[i][j] != numeric_limits<double>::infinity()) {
                reverseAdjMatrix[j][i] = adjMatrix[i][j];
            }
        }
    }

    // Step 3: Do DFS on the transposed graph
    fill(visited.begin(), visited.end(), false);
    cout << "\nStrongly Connected Components (SCCs):" << endl;
    while (!Stack.empty()) {
        int node = Stack.top();
        Stack.pop();

        if (!visited[node]) {
            vector<int> component;
            reverseDfs(node, reverseAdjMatrix, visited, component);

            // Print the SCC
            cout << "SCC: ";
            for (int v : component) {
                cout << v << " ";
            }
            cout << endl;
        }
    }
}

int main() {
    // EXO 1    Get sensor datas, create and print the adjacency matrix
    string filename = "../data_sensors10.txt";
    vector<Sensor> sensors = filedata(filename);
    if (sensors.empty()) {
        cout << "No sensors found in the file." << endl;
        return 1;
    }
    // Create the adjacency matrix
    vector<vector<double>> adjMatrix = AdjacencyMatrix(sensors);
    // Print the adjacency matrix
    cout << "Adjacency Matrix (Transmission Times):" << endl;
    printAdjacencyMatrix(adjMatrix);

    // EXO 2    Find and print SCCs
    findSCCs(adjMatrix);

    // EXO 3    Get the shortest path between the SCCs (i think)

    return 0;
}
