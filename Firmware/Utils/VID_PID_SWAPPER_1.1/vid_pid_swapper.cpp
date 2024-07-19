/*Author: overwraith
Purpose: to automatically change the vidpid.bin file on the USB rubber ducky using an executable. 
Disclaimer: I admit to having a little help from online forums etc. */
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <cstdlib>

using namespace std;

std::string getRandLineFromFile(const char* filename);
int getNumLines(const char* filename);

int main(int argc, char *argv[])
{   
    std::string ctline = getRandLineFromFile("VIDPID.txt");
    
    cout << ctline << endl;
    
    char buffer[9];
    std::size_t length = ctline.copy(buffer,0,8);
    buffer[length]='\0';

    int n,m=0;
    char buf[5];
    std:string hexmap= "0123456789abcdef";
    for(int i=0; i<8; i++)
    {
        for(int j=0;j<16;j++){
             //printf("%c vs %c\n",ctline[i],hexmap[j]);
            if(ctline.compare(i,1,hexmap,j,1)==0){
               if(i%2==0) j=j*16;
		//printf("I got %c and %d\n",ctline[i],j);
               n=n+j;
               break;
            }
        }
	if (i%2==0) buf[m]=n;
        if (i%2==1){ 
		buf[m]=buf[m]+n;
		m++;
	}
        n=0;
    }
    ofstream myFile("vidpid.bin", ios::out | ios::binary);
    myFile.write(buf,4);
    
    return EXIT_SUCCESS;
}

string getRandLineFromFile(const char* filename){
    std::ifstream input(filename);
    std::string line;
    
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
    std::ifstream input(filename);
    std::string line;
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
