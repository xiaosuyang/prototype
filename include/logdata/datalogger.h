#ifndef LOGDATA_H
#define LOGDATA_H

#include <fstream>
#include <iostream>
#include <vector>

class DataLogger
{

public:
    DataLogger(const DataLogger &) = delete;
    static DataLogger &GET()
    {
        static DataLogger S_logger;
        return S_logger;
    }

    void Savedata()
    {
   
        if (openFile(filepath))
        {
            file << rj2 << '  ';
            file << rj2des << '\n';
            file.close();
        }
        
    }
    ~DataLogger()
    {
        if (file.is_open())
        {
            file.close();
        }
    }

    float rj2=0, rj2des=0;

private:
    std::ofstream file;
    std::string filepath;
    std::string wholefile;
    const std::string prefix = "datalog/";

    DataLogger()
    {
        std::ofstream file1;
        wholefile = prefix + filepath;
        file1.open(wholefile.c_str());
    }
    bool openFile(const std::string &filename)
    {

        file.open((prefix + filename).c_str(), std::ios::app);
        if (!file.is_open())
        {
            std::cout << "无法打开文件！" << prefix + filename << std::endl;
            return false;
        }
        return true;
    }
};

#endif