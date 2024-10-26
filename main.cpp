#include <iostream>
#include <vector>
#include <stack>
#include <limits>
#include <string>
#include <cmath>     // For mathematical functions
#include <sstream>
#include <fstream>   // For file I/O (ifstream, ofstream)
#include <queue>     // For Dijkstra's algorithm
#include <algorithm> // For std::reverse
#include <set>

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
vector<Sensor> filedata(const string &filename) {
    vector<Sensor> sensors; // Vector where all sensors with their parameters will be saved
    ifstream file(filename); // Predefined method used to open the file
    string line;             // Hold temporarily each line of text from file when it is read

    while (getline(file, line)) {
        stringstream ss(line);
        string token;
        Sensor sensor;

        // Read and parse each value separated by commas
        getline(ss, token, ',');
        sensor.x = stod(token);

        getline(ss, token, ',');
        sensor.y = stod(token);

        getline(ss, token, ',');
        sensor.Tx = stod(token);

        getline(ss, token, ',');
        sensor.speed = stod(token);

        sensors.push_back(sensor);
    }
    return sensors;
}

// Function to create the adjacency matrix
vector<vector<double>> AdjacencyMatrix(const vector<Sensor> &sensors) {
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
void printAdjacencyMatrix(const vector<vector<double>> &adjMatrix) {
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
void dfs(int node, const vector<vector<double>> &adjMatrix, vector<bool> &visited, stack<int> &Stack) {
    visited[node] = true;
    for (int i = 0; i < adjMatrix.size(); i++) {
        if (adjMatrix[node][i] != numeric_limits<double>::infinity() && !visited[i]) {
            dfs(i, adjMatrix, visited, Stack);
        }
    }
    Stack.push(node);
}

void reverseDfs(int node, const vector<vector<double>> &reverseAdjMatrix, vector<bool> &visited, vector<int> &component) {
    visited[node] = true;
    component.push_back(node);
    for (int i = 0; i < reverseAdjMatrix.size(); i++) {
        if (reverseAdjMatrix[node][i] != numeric_limits<double>::infinity() && !visited[i]) {
            reverseDfs(i, reverseAdjMatrix, visited, component);
        }
    }
}

// Function to find SCCs using Kosaraju's algorithm
void findSCCs(const vector<vector<double>> &adjMatrix, vector<vector<int>> &clusters) {
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
    while (!Stack.empty()) {
        int node = Stack.top();
        Stack.pop();

        if (!visited[node]) {
            vector<int> component;
            reverseDfs(node, reverseAdjMatrix, visited, component);
            clusters.push_back(component);

            // Print the SCC
            cout << "SCC: ";
            for (int v : component) {
                cout << v << " ";
            }
            cout << endl;
        }
    }
}

// Tarjan's DFS-based algorithm to find articulation points
void articulationPointsDFS(int u, int parent, const vector<vector<int>> &adjList, vector<bool> &visited,
                           vector<int> &disc, vector<int> &low, vector<int> &parentTracker,
                           vector<bool> &isArticulationPoint, int &time) {
    visited[u] = true;
    disc[u] = low[u] = ++time;
    int children = 0;

    for (int v : adjList[u]) {
        if (!visited[v]) {
            children++;
            parentTracker[v] = u;
            articulationPointsDFS(v, u, adjList, visited, disc, low, parentTracker, isArticulationPoint, time);

            low[u] = min(low[u], low[v]);

            if (parentTracker[u] == -1 && children > 1) {
                isArticulationPoint[u] = true;
            }

            if (parentTracker[u] != -1 && low[v] >= disc[u]) {
                isArticulationPoint[u] = true;
            }
        } else if (v != parent) {
            low[u] = min(low[u], disc[v]);
        }
    }
}

// Function to find articulation points in a cluster
vector<int> findArticulationPointsInCluster(const vector<vector<int>> &adjList, int numSensors) {
    vector<bool> visited(numSensors, false);
    vector<int> disc(numSensors, -1);
    vector<int> low(numSensors, -1);
    vector<int> parentTracker(numSensors, -1);
    vector<bool> isArticulationPoint(numSensors, false);
    int time = 0;

    for (int i = 0; i < numSensors; i++) {
        if (!visited[i]) {
            articulationPointsDFS(i, -1, adjList, visited, disc, low, parentTracker, isArticulationPoint, time);
        }
    }

    vector<int> articulationPoints;
    for (int i = 0; i < numSensors; i++) {
        if (isArticulationPoint[i]) {
            articulationPoints.push_back(i);
        }
    }
    return articulationPoints;
}

// Function to create an adjacency list from the sensors data
vector<vector<int>> createAdjacencyList(const vector<Sensor> &sensors) {
    int n = sensors.size();
    vector<vector<int>> adjList(n);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i != j) {
                double distance = euclideanDistance(sensors[i], sensors[j]);
                if (distance <= sensors[i].Tx && distance <= sensors[j].Tx) {
                    adjList[i].push_back(j);
                }
            }
        }
    }
    return adjList;
}

// Dijkstra's algorithm to find the shortest path from a source node to all other nodes
void dijkstra(const vector<vector<double>> &adjMatrix, int src, vector<double> &distances, vector<int> &predecessors) {
    int n = adjMatrix.size();
    vector<bool> visited(n, false);
    distances[src] = 0;

    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    pq.push({0, src});

    while (!pq.empty()) {
        auto [dist, node] = pq.top();
        pq.pop();

        if (visited[node]) continue;
        visited[node] = true;

        for (int neighbor = 0; neighbor < n; ++neighbor) {
            if (adjMatrix[node][neighbor] != numeric_limits<double>::infinity()) {
                double newDist = distances[node] + adjMatrix[node][neighbor];
                if (newDist < distances[neighbor]) {
                    distances[neighbor] = newDist;
                    predecessors[neighbor] = node;
                    pq.push({newDist, neighbor});
                }
            }
        }
    }
}

// Function to find shortest paths in each cluster and print them
void findShortestPathsInClusters(const vector<vector<int>> &clusters, const vector<vector<double>> &adjMatrix) {
    for (const auto &cluster : clusters) {
        int clusterSize = cluster.size();
        if (clusterSize < 2) continue;

        cout << "Shortest Paths in Cluster: ";
        for (int sensor : cluster) cout << sensor << " ";
        cout << endl;

        for (int src : cluster) {
            vector<double> distances(adjMatrix.size(), numeric_limits<double>::infinity());
            vector<int> predecessors(adjMatrix.size(), -1);
            dijkstra(adjMatrix, src, distances, predecessors);

            for (int dest : cluster) {
                if (src != dest) {
                    cout << "Shortest path from " << src << " to " << dest << ": ";
                    int u = dest;
                    vector<int> path;
                    while (u != -1) {
                        path.push_back(u);
                        u = predecessors[u];
                    }
                    reverse(path.begin(), path.end());
                    for (int node : path) cout << node << " ";
                    cout << " (Cost: " << distances[dest] << ")\n";
                }
            }
            cout << endl;
        }
    }
}

#include <set>

// Function to find the minimum number of sensors to modify for full connectivity
void findMinimumAdjustmentsForFullConnectivity(const vector<vector<int>> &clusters, const vector<Sensor> &sensors) {
    // Track all unique clusters
    set<int> modifiedSensors; // To keep track of sensors we need to modify
    int n = clusters.size();

    // Determine existing connections between clusters
    vector<vector<bool>> canConnect(n, vector<bool>(n, false));

    // Iterate through each cluster and check connectivity with other clusters
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            for (int u : clusters[i]) {
                for (int v : clusters[j]) {
                    if (euclideanDistance(sensors[u], sensors[v]) <= sensors[u].Tx ||
                        euclideanDistance(sensors[v], sensors[u]) <= sensors[v].Tx) {
                        canConnect[i][j] = true;
                        canConnect[j][i] = true; // Undirected graph
                        break; // No need to check more pairs in this iteration
                    }
                }
            }
        }
    }

    // Identify clusters that are not directly connected
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (!canConnect[i][j]) {
                // For each pair of clusters that are not connected,
                // determine which sensor to modify in one of the clusters
                for (int sensorInClusterA : clusters[i]) {
                    for (int sensorInClusterB : clusters[j]) {
                        double distance = euclideanDistance(sensors[sensorInClusterA], sensors[sensorInClusterB]);
                        if (distance > sensors[sensorInClusterA].Tx) {
                            // The sensor in cluster A needs to be modified
                            modifiedSensors.insert(sensorInClusterA);
                        }
                        if (distance > sensors[sensorInClusterB].Tx) {
                            // The sensor in cluster B needs to be modified
                            modifiedSensors.insert(sensorInClusterB);
                        }
                    }
                }
            }
        }
    }

    // Output the results
    cout << "Sensors to modify for full connectivity: ";
    for (int sensor : modifiedSensors) {
        cout << sensor << " ";
    }
    cout << endl;
}

int main() {
    // Load sensors from "data_sensors10.txt"
    vector<Sensor> sensors = filedata("../data_sensors10.txt");

    // Generate the adjacency matrix
    vector<vector<double>> adjMatrix = AdjacencyMatrix(sensors);

    // Print the adjacency matrix
    cout << "Adjacency Matrix:" << endl;
    printAdjacencyMatrix(adjMatrix);

    // Find SCCs in the network
    vector<vector<int>> clusters;
    findSCCs(adjMatrix, clusters);

    // Create the adjacency list for articulation points
    vector<vector<int>> adjList = createAdjacencyList(sensors);

    // Display articulation points in each SCC cluster
    cout << "Articulation Points in Each Cluster:" << endl;
    for (const auto &cluster : clusters) {
        // Create a subgraph for the current cluster
        vector<vector<int>> subAdjList(cluster.size());
        for (size_t i = 0; i < cluster.size(); ++i) {
            for (size_t j = 0; j < cluster.size(); ++j) {
                if (i != j) {
                    int u = cluster[i];
                    int v = cluster[j];
                    // Check for connections in the original adjacency list
                    if (find(adjList[u].begin(), adjList[u].end(), v) != adjList[u].end()) {
                        subAdjList[i].push_back(j); // add directed edge to subgraph
                    }
                }
            }
        }

        vector<int> articulationPoints = findArticulationPointsInCluster(subAdjList, cluster.size());
        cout << "Articulation Points for Cluster: ";
        for (int point : articulationPoints) {
            cout << point << " ";
        }
        cout << endl;
    }

    // Find and print shortest paths within each SCC cluster
    cout << "Shortest Paths in Each Cluster:" << endl;
    findShortestPathsInClusters(clusters, adjMatrix);

    // Find minimum adjustments for full connectivity
    findMinimumAdjustmentsForFullConnectivity(clusters, sensors);

    return 0;
}
