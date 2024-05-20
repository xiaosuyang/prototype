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
            file << rj2 << '\t'<<'\t';
            file << rj2des <<'\t'<<'\t';
            file << rj3 << '\t'<<'\t';
            file << rj3des << '\t'<<'\t';
            file << rj4 << '\t'<<'\t';
            file << rj4des << '\t'<<'\t';
            file<<pidoutput[0]<<'\t'<<'\t';
            file<<pidoutput[1]<<'\t'<<'\t';
            file<<pidoutput[2]<<std::endl;
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

    float rj2=0, rj2des=0,rj3=0,rj3des=0,rj4=0,rj4des=0;
    float pidoutput[3];//rj2,3,4 pid输出
    

private:
    std::ofstream file;
    std::string filepath;
    std::string wholefile;
    const std::string prefix = "datalog/";

    DataLogger()
    {
        filepath=std::string("rj2.txt");
        std::ofstream file1;
        wholefile = prefix + filepath;
        file1.open(wholefile.c_str());
        for(int i=0;i<3;i++) pidoutput[i]=0;
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