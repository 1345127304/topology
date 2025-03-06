#include <cstdio>
#include <iostream>
#include <sstream>
#include <map>
#include <fstream>
#include <string>
#include <vector>
#include <tuple>
#include <fstream>
#include <algorithm>
#include <set>
using namespace std;
//problem: test case 10, 3, 7,9 time limit exceeded for changing one node to unreachable
    // problem: the vector still exists even when all vectors within are emptied
map<int, vector<int>> dijkstra(map<int, vector<vector<int>>> graph, int source){
    map<int, int> distance;
    map<int, vector<int>> predecessor;
    map<int, vector<vector<int>>> front = graph;
    for (const auto&pair: graph){
        distance[pair.first] = 10000000;
        predecessor[pair.first] = {-1, 10000};
    }
    distance[source] = 0;
    int min = 100000;
    int node = -1;
    int last_node = -2;
    for(auto&pair: front){
        if(pair.second.size() == 0){
            front.erase(pair.first);
            predecessor.erase(pair.first);
        }
    }
    
    while(front.empty()!= true){
        // find the smallest distance in front
        //cout<<"min: "<<min<<endl;
        //cout<<"node: "<<node<<endl;

        // getting the node with smallest distance cost
        for (const auto&pair: front){
            if(distance[pair.first] < min){
                min = distance[pair.first];
                node = pair.first;
                
                //cout<<"min: "<<min<<endl;
                //cout<<"node: "<<node<<endl;
            }
        }
        if(last_node == node){  // the rest of destination can't be reached from this source
            for (const auto&pair: front){   // 
                predecessor[pair.first] = {-2, -999};
            }
            break;
        }
        // remove the node in front
        front.erase(node);
        //cout<<"front size after erase: "<<front.size()<<endl;
        cout<<"node: "<<node<<endl;
        //cout<<"vector size: "<<graph[node].size()<<endl;
        // problem: could met empty vector
        // traverse the neighbor node, key = node, 1st = neighbor, 2nd = cost
        cout<<"found 1: "<<front.count(1)<<endl;
        cout<<"found 2: "<<front.count(2)<<endl;
        cout<<"found 3: "<<front.count(3)<<endl;
        cout<<"found 4: "<<front.count(4)<<endl;
        cout<<"found 5: "<<front.count(5)<<endl;
        for (int i = 0; i < graph[node].size(); i++){
            //cout<<"i: "<<i<<endl;

            int neighbor = graph[node][i][0];
            cout<<"neighbor: "<<neighbor<<endl;
            int cost = graph[node][i][1];
            //cout<<" cost: "<<cost<<endl;
            //cout<<"vector size: "<<graph[node].size()<<endl;
            // compare cost
            if (distance[node] + cost < distance[neighbor]){    // smaller cost treatment
                cout<<"option 1"<<endl;
                distance[neighbor] = distance[node]+cost;
                predecessor[neighbor] = {node, distance[node]+cost};
                //cout<<"new shortest distance from "<<neighbor<<" to "<<node<<": "<<distance[node]+cost<<endl;
                //cout<<"new predecessor for node "<<neighbor<<": "<<node<<endl;
            }else if (distance[node] + cost == distance[neighbor]){     // equal cost treatment
                // store the node with the smaller id/lexicographically smaller sequence
                // check predecessor for node
                cout<<"option 2"<<endl;
                if(predecessor.count(neighbor) == 1){ // predecessor exist
                    if(predecessor[neighbor][0]>node && predecessor[neighbor][0] != -1){  // if the new neighbor's id is smaller than the old neighbor and it isn't the placeholder -1, switch
                        predecessor[neighbor][0] = node;
                    }
                }

            }
        }
        min = 100000;
        last_node = node;
    }
    cout<<"dijkstra done"<<endl;
    cout<<" for source "<<source<<endl;
    for(const auto&pair: predecessor){
        cout<<"dest: "<<pair.first;
        cout<<" pred: "<<pair.second[0];
        cout<<" cost: "<<pair.second[1]<<endl;
    }
    int a = predecessor[10][2];

    return predecessor;

}

// algorithm that generaets forwarding table from dijkstra's result, format: dest, next hop, cost
vector<int> forwardingPath(map<int, vector<int>> graph,int source, int destination){
    // recall that dijkstra's format: dest: predecessor, cost
    bool status = false;
    int curr = destination;
    vector<int> path;
    if(graph[destination][0] == -2){
        path.push_back(-2);
    }else{
        int cost = graph[destination][1];
        int hold;
        while(!status){
            if(graph.count(curr)==1){
                hold = graph[curr][0];
                //cout<<"curr: "<<curr<<endl;
                //cout<<"hold: "<<hold<<endl;
                if((hold == source)|| (hold == -1)){        // path found or source == destination on first iteration
                    status =true;
                } else{
                    curr = hold;
                }
            }
        }
        if (hold == -1){                                // local, so dest = source
            path = {destination, source, 0};
        }else {
            path = {destination, curr, cost};
        }
        }
    // path gives the destination, next hop, and cost
    // if path only contains -2, it means that the destination can't be reached
    return path;
}

map<int, map<int, vector<int>>> forwardingTable(map<int, vector<vector<int>>> graph){
    map<int, map<int, vector<int>>> forwarding_path_table;
    map<int, vector<int>> hold;
    vector<int> result;
    for(const auto&pair: graph){// pair.first is the source/node, entries.first is destination
        //cout<<"node on forwarding table: "<<pair.first<<endl;
        cout<<"pair.first: "<<pair.first<<endl;
        if(graph[pair.first].size() == 0){      // dealt with isolated node
            forwarding_path_table[pair.first][pair.first] = {{pair.first, 0}};
        }else{
            hold = dijkstra(graph, pair.first);
            cout<<"check2"<<endl;
            for(const auto&entries: hold){
                result = forwardingPath(hold, pair.first, entries.first);
                if(result[0] != -2){    // skip the entry if result[0] = -2, which means the destination is unreachable
                    cout<<"check3"<<endl;
                    //cout<<"here"<<endl;
                    if (forwarding_path_table[pair.first].count(result[0]) > 0){       // key already exist
                        /*forwarding_path_table[pair.first][result[0]] = push_back(result);
                    } else{*/
                        forwarding_path_table[pair.first][result[0]] = {{result[1], result[2]}};
                    }else {
                        forwarding_path_table[pair.first][result[0]] = {{result[1], result[2]}};
                    }
                    }
            }
        }
        
    }
    return forwarding_path_table;

}

vector<string> extractMessage(string message){
    istringstream msg(message);
    string node1, node2, actual_msg;

    // extract the first two word;
    msg >> node1 >> node2;
    // get the rest as message;
    getline(msg>>ws, message);
    return vector<string> {node1, node2, message};
}

vector<int> findPath(map<int, map<int, vector<int>>> table, int source, int dest){

    // note: table gives the forward table for node "key"
    // for_table format: <dest, <next hop, cost>>, index+1 = node
    bool status = false;
    vector<int> path;
    set<int> visited;    // check if the node is already visited, serve as loop detection
    int curr = source;
    
    if (table[source].count(dest) == 0){ // destination can't be reached
        path.push_back(-1);
    }else{
        int hold = table[curr][dest][0];                // next hop
        //cout<<"curr: "<<curr<<endl;
        //cout<<"hold: "<<hold<<endl;
        while(!status){
            if (visited.find(curr) != visited.end()) {
                path.clear();
                path.push_back(-1); // Indicate unreachable
                break;
            }
            visited.insert(curr);
            cout<<"curr: "<<curr<<endl;
            cout<<"hold: "<<hold<<endl;
            path.push_back(curr);
            if((hold == dest)|| (hold == -1)){        // path found or source == destination on first iteration
                status =true;
                cout << "achieved"<<endl;
            } else{
                curr = hold;            // new source
                hold = table[curr][dest][0];
            }
        }
    }

    return path;

}

vector<string> formatMessage(vector<string> messages,  map<int, map<int, vector<int>>> table){
    // format: “from <x> to <y> cost <path_cost> hops <hop1> <hop2> <...> message <message>
    // TODO: format the string for the case when the path doesn't exist
    vector<string> formatMsg;
    for(const auto&line: messages){
        vector<string> holder = extractMessage(line);
        int s = stoi(holder[0]);
        int d = stoi(holder[1]);

        // find the path and cost from source to destination
        vector<int> path =  findPath(table, s,d);
        if(path[0] == -1){  // can't reach dest from source
            string invalidPath = "from "+holder[0]+" to "+ holder[1] + " cost infinite hops unreachable message " + holder[2]+"\n";
            formatMsg.push_back(invalidPath);
        }else{
            string cost = to_string(table[s][d][1]);
            string hops = "";
            for (int i = 0; i < path.size(); i++){
                hops = hops + to_string(path[i]) + " ";
            }
            string msg = "from "+holder[0]+" to "+ holder[1] + " cost " + cost + " hops " + hops + "message " + holder[2]+"\n";
            formatMsg.push_back(msg);
        }
    }
    return formatMsg;
}

bool isOnlySpaces(const std::string& str) {
    return std::all_of(str.begin(), str.end(), [](char c) { return c == ' '; });
}

int main(int argc, char** argv) {
    //printf("Number of arguments: %d", argc);
    vector<string> hold;
    string word;
    if (argc != 4) {
        printf("Usage: ./linkstate topofile messagefile changesfile\n");
        return -1;
    }
    // map to store vertices and edges with the cost
    map<int, vector<vector<int>>> topology;
    ifstream topo;
    topo.open(argv[1]);
    if (topo.is_open()){
        string line;
        
        
        while (getline(topo, line)){
            //cout<<line<<endl;
            if(isOnlySpaces(line)){
                continue;
            }
            // store the source node
            istringstream extractor(line);
            while (extractor >> word) {  // Extract words separated by spaces
                hold.push_back(word);
            }
            if (topology.count(stoi(hold[0])) > 0){  // if the node already exist
                topology[stoi(hold[0])].push_back({stoi(hold[1]), stoi(hold[2])});
            }else {                                // if the node doesn't exist
                topology[stoi(hold[0])] = {{stoi(hold[1]), stoi(hold[2])}};
            }

            /*// store the dest node
            if (topology.count(line[2]-'0') == 0){
                topology[line[2]-'0'].push_back({});
            }*/
        // store the path in reverse order
            if (topology.count(stoi(hold[1])) > 0){   // if the node already exist
                topology[stoi(hold[1])].push_back({stoi(hold[0]), stoi(hold[2])});
            }else {                                 // if the node doesn't exist
                topology[stoi(hold[1])] = {{stoi(hold[0]), stoi(hold[2])}};
            }
            hold.clear();

        }
        
        topo.close();
    } else {
        cout <<"unable to open the topology file"<<endl;
    }

    //cout << argv[1];
    // check if topology is correct
    
    for(const auto&pair: topology){
        cout<< "node: "<<pair.first<<endl;
        for(int i = 0; i < pair.second.size(); i++){
            for(int j = 0; j < pair.second[i].size(); j++){
                cout<<pair.second[i][j]<<" ";
            }
            cout<<endl;
        }

    }
    // sent LSA to neighbors
    // LSA = int id, int seqNum, vector<int> (id, dest, cost)
    // local LSA: map<int id, tuple(int id, int seqNum, vector<int> (id, dest, cost))>
    // idea, use struct node to create a map<int id, local_LSA>, for each pair in the map, run the link state algorithm, after it's done run dijkstra
    // however, in this MP we assume that all node knows of the initial topology and the change to topology, therefore we can
    // went straight for dijkstra's algorithm to compute the shortest path for the initial and changed topology, this makes it so 
    // each node should store the forwarding table from dijkstra
    // note: the predecessor table reads as: destination, predecessor, cost, we regard -1 predecessor as local
    /*map<int, vector<int>> table = dijkstra(topology, 2);

    for(const auto&pair: table){
        cout<< "destination: "<<pair.first;
        cout<< " predecessor: "<<pair.second[0];
        cout<< " cost: "<<pair.second[1]<<endl;
    }*/
    //vector<int> a = forwardingPath(table, 2,1);
    //cout<<"destination: "<<a[0];
    //cout<<" next hop: "<<a[1];
    //cout<<" cost: "<<a[2]<<endl;
    // compute forwarding table for all node
    map<int, map<int, vector<int>>> table = forwardingTable(topology);
    /*map<int, vector<int>> t = table[2];
    for(const auto&pair: t){

        cout<<"destination: "<<pair.first;
        cout<<" next hop: "<<pair.second[0];
        cout<<" cost: "<<pair.second[1]<<endl;
        
    }*/
    
    // write to file
    ofstream fpOut("output.txt");
    // pair(node, table), entry(destination, (next hop, cost))
    for(const auto&pair: table){
        for(const auto&entry: pair.second){

            string path = to_string(entry.first) + " "+to_string(entry.second[0])+" "+to_string(entry.second[1])+"\n";
            fpOut << path;
        }
    }
    cout<<"successfully write to file the tables (unchanged)"<<endl;
    // sent message preparation(in output.txt)
    string line2;
    ifstream msgFile;
    vector<string> messages;

    // store the messages 
    msgFile.open(argv[2]);
    if (msgFile.is_open()){
        while (getline(msgFile, line2)){
            if(isOnlySpaces(line2) == false){
                messages.push_back(line2);
            }
        }
    }
    msgFile.close();
    vector<string> formattedMsg = formatMessage(messages, table);
    // senting the message
    for(const auto& line: formattedMsg){
        fpOut << line;
    }
    // next: get changed topology
    // for broken link (-999), remove the link from source to neighbor and vice versa

    // store changes
    ifstream changeFile;
    string line3;
    vector<string> changes;
    changeFile.open(argv[3]);
    if (changeFile.is_open()){
        while (getline(changeFile, line3)){
            if(isOnlySpaces(line3) == false){
                changes.push_back(line3);
            }
        }
    }
    for(const auto&line: changes){
        // format: source, neighbor, cost (-999 is broken link)
        vector<string> condition = extractMessage(line);
        cout<<"changes: "<<line<<endl;

        int s = stoi(condition[0]);
        int neighbor = stoi(condition[1]);
        int cost = stoi(condition[2]);

        // changing the topology
        // problem: properly remove the neighbor when its empty

        if(topology.count(s) == 0 && cost != -999){ // key doesn't exist before because the link doesn't exist previously
            // add the link for source to neighbor
            topology[s] = {{neighbor, cost}};

            // add the link for neighbor to source
            if(topology.count(neighbor) > 0){ // neighbor exist
                topology[neighbor].push_back({s, cost});
            }else{                            // neighbor doesn't exist
                topology[neighbor] = {{s, cost}};
            }
        } else {    // key still exists
            cout<<"key exist"<<endl;
            bool exist = false;
            for (int i = 0; i <topology[s].size(); i++){    // check if neighbor exist
                if(topology[s][i][0] == neighbor){          // neighbor exist
                    exist = true;     
                    cout<<"key exist"<<endl;
                    if(cost != -999){                       // regular cost adjustment
                        // adjust itself
                        topology[s][i][1] = cost;

                        // adjust neighbor
                        for (int j = 0; j <topology[neighbor].size(); j++){
                            if(topology[neighbor][j][0] == s){
                                topology[neighbor][j][1] = cost;
                            }
                        }
                        break;
                    }else{                                  // link break
                        topology[s].erase(topology[s].begin()+i);
                        if(topology[s].empty()){
                            topology.erase(s);
                        }
                        // check neighbor
                        for (int j = 0; j <topology[neighbor].size(); j++){
                            if(topology[neighbor][j][0] == s){
                                topology[neighbor].erase(topology[neighbor].begin()+j);
                                if(topology[neighbor].empty()){
                                    cout<<"neighbor: "<<neighbor<<endl;
                                    cout<<"empty vector"<<endl;
                                    topology.erase(neighbor);
                                    cout<<"erased?: "<<topology.count(neighbor)<<endl;
                                }
                            }
                        }
                        break;
                    }
                }
            }
            if (exist == false){    // neighbor doesn't exist, need to add it in, we already checked that source node exist
                if(cost != -999){   // link isn't broken
                cout<<"existed"<<endl;
                    topology[s].push_back({neighbor, cost});
                    if(topology.count(neighbor) > 0){       // check if neighbor exist
                        topology[neighbor].push_back({s, cost});
                    }else{                                  // if neighbor doesn't exist
                        topology[neighbor]={{s, cost}};
                    }
                }// if link is broken and neighbor doesn't exist, we don't do anything
            }
    }

        // checking the new topology:
        for(const auto&pair: topology){
            cout<< "node: "<<pair.first<<endl;
            for(int i = 0; i < pair.second.size(); i++){
                for(int j = 0; j < pair.second[i].size(); j++){
                    cout<<pair.second[i][j]<<" ";
                }
                cout<<endl;
            }
            /*if(pair.second.empty() == true){
                topology.erase(pair.first);
                }*/
    
        }
        cout<<"check if still exist: "<<topology.count(3)<<endl;

        // construct the updated forwarding table
        cout<<"check1"<<endl;
        table = forwardingTable(topology);
        cout<<"check2"<<endl;
        // write the updated table to output
        for(const auto&pair: table){
            for(const auto&entry: pair.second){
    
                string path = to_string(entry.first) + " "+to_string(entry.second[0])+" "+to_string(entry.second[1])+"\n";
                fpOut << path;
            }
        }

        // senting the message in the updated table
        // TODO: get the formatted Msg from the new table
        formattedMsg = formatMessage(messages, table);
        for(const auto& line: formattedMsg){
            fpOut << line;
        }
    }

    cout<<"topology done"<<endl;



    fpOut.close();

    // You may choose to use std::fstream instead
    // std::ofstream fpOut("output.txt");

    return 0;
}
