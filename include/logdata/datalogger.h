#ifndef LOGDATA_H
#define LOGDATA_H

#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>

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
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< rj0 ;
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< rj0des; 
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< rj1 ;
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< rj1des ;
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< rj2 ;
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< rj2des; 
            
            file<<rj2<<"  ";
            file<<rj2des<<"  ";
            file<<rj3<<"  " ;
            file<<rj3des<<"  ";
            file<<rj1<<"  ";
            file<<rj1des<<"  ";
            file<<rj0<<"  ";
            file<<rj0des<<"  ";
            file<<rj1<<"  ";
            file<<rj1des<<"  ";

            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< rj4 ;
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< rj4des ;
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< pidoutput[0] ;
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< pidoutput[1] ;
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< pidoutput[2] ;
            // file<<std::left<<std::setw(20)<<std::setprecision(4)<< pidoutput[3] ;


            file<<'\n';
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< rj2length ;
            // file <<std::left<<std::setw(20)<<std::setprecision(4)<< rj2lengthdes << std::endl;
            
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

    float rj2 = 0, rj2des = 0, rj3 = 0, rj3des = 0, rj4 = 0, rj4des = 0,rj0=0,rj0des=0,rj1=0,rj1des=0;
    float pidoutput[12]; // rj2,3,4 pid输出
    float rj2length=0, rj2lengthdes=0;

private:
    std::ofstream file;
    std::string filepath;
    std::string wholefile;
    const std::string prefix = "datalog/";

    DataLogger()
    {
        filepath = std::string("rj2.txt");
        std::ofstream file1;
        wholefile = prefix + filepath;
        file1.open(wholefile.c_str());
        for (int i = 0; i < 3; i++)
            pidoutput[i] = 0;
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