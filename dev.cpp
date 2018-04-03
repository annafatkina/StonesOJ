#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

int main() {
    std::ifstream infile("ou12.obj");
    std::ofstream out1("ov1.xyz");

    std::ofstream out2("of1.xyz");
    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        char c;
        //if (!(iss >> a >> b)) { break; } // error
        if (line[0] == 'v') {out1 << line.substr(2) << std::endl;}
        if (line[0] == 'f') {
		int x, y, z;
		iss >> c >> x >> y >> z; 
		out2 << x-1 << " " << y-1 << " " << z-1  << std::endl;
	}
	
        // process pair (a,b)
    }

	
	return 0;
}
