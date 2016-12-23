#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>


using namespace std;

void fparse(const char *fname);
//void sparse(std::string s, std::string d, std::vector<std::string> &v);
//std::vector<std::string> split(const std::string& str, const std::string& delim);
void split(const string& s, char c,
           vector<string>& v);

int main(int argc, char const *argv[]){
    /* code */
    fparse("test.txt");
    return 0;
}

void fparse(const char *fname){
    std::ifstream infile(fname);
    std::string line;
    std::string d = ",";
    std::vector<std::string> v;
    std::string s;
    while (std::getline(infile, line)) {
        if(line.empty()){
            perror("Empty line.");
            break;
        }
        /* Strip whitespace from string */
        std::string::iterator end_pos = std::remove(line.begin(), line.end(), ' ');
        line.erase(end_pos, line.end());
        //cout << "passing string " << line << endl;
        split(line, ',', v);
    }
      for(int i = 0; i < v.size(); i++){
        std::cout << v[i] << std::endl;
    }
}


void split(const string& s, char c,
           vector<string>& v) {
   string::size_type i = 0;
   string::size_type j = s.find(c);

   while (j != string::npos) {
      v.push_back(s.substr(i, j-i));
      i = ++j;
      j = s.find(c, j);

      if (j == string::npos)
         v.push_back(s.substr(i, s.length()));
   }
}
