#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
    return 0;
}
#define MAPLINE_TOTAL 115

class baseFrameReader
{
    public:
        double BASE_XY[MAPLINE_TOTAL];
        double BASE_THETA[MAPLINE_TOTAL]; 
        double BASE_KAPPA[MAPLINE_TOTAL];
        int length;
    baseFrameReader()
    {
        string FilePath = "/home/menguiin/catkin_ws/src/macaron_2/path/manhae_base_frame.txt";
        FileRead(FilePath);
    }
    void FileRead(string FILEPATH)
    {
        string token;
        int index = 0;
	    ifstream openFile(FILEPATH.data());
	    if( openFile.is_open() ){
	    	string line;
		    while(getline(openFile, line))
            {   
			    stringstream ss(line);
                while(getline(ss,token,','))
                {
                    switch(index%3)
                    {
                        case 0 : 
                            BASE_XY[index++] = stod(token); break;
                        case 1:
                            BASE_THETA[index++] = stod(token); break;
                        case 2:
                            BASE_KAPPA[index++] = stod(token); break;
                    }
                }
		    }
 		    openFile.close();
	    }
    }
};

