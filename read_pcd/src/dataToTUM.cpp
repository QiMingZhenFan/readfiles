#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

int main(){

    string folder_name = "/home/simulation/workspace/my_ws/src/data/";
    string filename[] = {"trajectory_aftmapped.csv", "trajectory_gt.csv"};

    for(auto name : filename){
        ifstream inFile(folder_name + name, ios::in);
        if(!inFile){
            cout << "Cannot read file : " << name << endl;
            return 1;
        }
        vector<vector<string> > content;
        string line;
        while (getline(inFile, line)) {
            istringstream ss(line);
            vector<string> data;
            string temp;
            while (getline(ss, temp, '\t')) {
                data.push_back(temp);   // time x y z
            }
            content.push_back(data);
        }
        ofstream outFile(folder_name + name, ios::out);
        if(!outFile){
            cout << "Cannot write file : " << name << endl;
            return 1;
        }
        for (auto ptr = content.begin(); ptr != content.end(); ptr++) {
            outFile << ptr->data()[0] << ' ' << ptr->data()[1] << ' ' << ptr->data()[2] << ' ' << ptr->data()[3] << ' ' << 0 << ' ' << 0 << ' ' << 0 << ' ' << 0 << endl;
        }
        outFile.close();

    }
    cout << " convert to TUM finished! " << endl;
    return 0;
}