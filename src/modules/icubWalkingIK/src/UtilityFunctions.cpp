#include<iostream>
#include<sstream>
#include<fstream>
#include "UtilityFunctions.h"
#include <stdio.h>
#include <string>

using namespace std;

void readFromCSV(string filename, vector<RigidBodyDynamics::Math::VectorNd>& mat)
{
  ifstream file(filename.c_str());
  string line;
  int rows;
  int cols;
  rows = mat.size();
  cols = mat[0].size();

//  for(int row = 0; row < rows; row++)
//  {
//      getline(file, line);
//      if ( !file.good() )
//      break;
//
//      stringstream iss(line);
//      for (int col = 0; col < cols; col++)
//      {
//      string val;
//      getline(iss, val, ',');
//      if ( !iss.good() )
//          break;
//
//      stringstream convertor(val);
//      convertor >> mat[row][col];
//      }
//  }
    
    int r = 0;
    int c = 0;
    //cout << filename << endl;
    // Another way of parsing. Added by Jorhabib Eljaik
    while( std::getline( file, line ) )
    {
        std::istringstream iss( line );
        std::string result;
        while( std::getline( iss, result , ',') )
        {
            std::stringstream convertor(result);
            convertor >> mat[r][c];
            //cout << "| " << mat[r][c] << " |  ";
            c++;
        }
        //cout << " " << endl;
        r++;
        c = 0;
    }

  file.close();
}

void writeOnCSV(RigidBodyDynamics::Math::VectorNd time, vector<RigidBodyDynamics::Math::VectorNd>& data, string file_name, const char* header)
{
  FILE *fp;
  stringstream output_filename ("");  

  output_filename << file_name;
  
  fp = fopen (output_filename.str().c_str(), "w");

  if (!fp) {
    fprintf (stderr, "Error: Could not open file '%s'!\n", output_filename.str().c_str());
    return;
  }
  
  if(header != "")
    fprintf (fp, "%s\n", header);
  
  for(int i = 0; i < time.size(); i++)
  {
    // time
    fprintf (fp, "%e,\t", time[i]);

    // joint values
    for (int j = 0; j < data[i].size(); j++)
      fprintf (fp, "%e,\t", data[i][j]);
      
    if(i < (time.size() - 1))
      fprintf (fp, "\n");
  }
  
  fclose (fp);
}

void writeOnCSV(vector<RigidBodyDynamics::Math::VectorNd>& data, string file_name)
{
  FILE *fp;
  stringstream output_filename ("");  

  output_filename << file_name;
  
  fp = fopen (output_filename.str().c_str(), "w");

  if (!fp) {
    fprintf (stderr, "Error: Could not open file '%s'!\n", output_filename.str().c_str());
    return;
  }
    //std::cout << data.size() << endl;
    //std::cout << data[0].size() << endl;
  
  for(int i = 0; i < data.size(); i++)
  {
    // joint values
    for (int j = 0; j < data[i].size(); j++)
    {
        if (j == data[i].size() - 1) {
          //cout << data[i][j] << " " << endl;
          fprintf(fp, "%e", data[i][j]);
        }
        else {
            //cout << data[i][j] << endl;
            fprintf (fp, "%e,\t", data[i][j]);
        }
    }
    
    fprintf (fp, "\n");
  }
  
  fclose (fp);
}