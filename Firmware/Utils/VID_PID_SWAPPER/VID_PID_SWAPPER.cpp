/*Author: overwraith
Purpose: to automatically change the vidpid.bin file on the USB rubber ducky using an executable. 
Disclaimer: I admit to having a little help from online forums etc. */
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

using namespace std;

string getRandLineFromFile(const char* filename);
int getNumLines(const char* filename);

int main(int argc, char *argv[])
{   
    string ctline = getRandLineFromFile("VIDPID.txt");
    
    cout << ctline << endl;
    
    char buffer[9];
    strncpy(buffer, ctline.c_str(), sizeof(buffer));
    buffer[sizeof(buffer) - 1] = 0;
    
    
    ofstream myFile("vidpid.bin", ios::out | ios::binary);
    myFile.write (buffer, 8);
    
    return EXIT_SUCCESS;
}

string getRandLineFromFile(const char* filename){
    ifstream input(filename);
    string line;
    
    srand(time(NULL));
    
    if(input.is_open()){
            for(int i = 0, lines = getNumLines(filename); i < lines && i < rand() % (lines + 1) ; i++){
                    //get the line
                    getline(input, line);
            }//end loop
     input.close();
    }
    else
        cout << "Unable to open file" << endl;
    
    return line;
}

int getNumLines(const char* filename){
    ifstream input(filename);
    string line;
    int i = 0;

    if(input.is_open()){
     while(input.good()){
      getline(input, line);
      //cout << line << '\n';
      i++;
     }//end loop
     input.close();
    }
    else
        cout << "Unable to open file";
    return i;
}
