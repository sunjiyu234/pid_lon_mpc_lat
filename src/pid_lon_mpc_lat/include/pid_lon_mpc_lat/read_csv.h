#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#ifndef CSV_READER_H
#define CSV_READER_H
using namespace std;
class CSVReader{
public:
    string filename_;
    string delimiter_;

    /*
        csv文件阅读器构造函数
        输入：文件名，分割符
    */
    CSVReader(string filename, string delimiter) : filename_(filename), delimiter_(delimiter){ }
    
    /*
        逐行阅读csv文件
        输入：无
        输出：string的二维vector
    */
    vector<vector<string>> readData();
};

vector<vector<string>> CSVReader::readData(){
    ifstream file(filename_);
    vector<vector<string>> result_list;
    string line = "";
    while (getline(file, line)){
        vector<string> cur_line_vector;
        boost::algorithm::split(cur_line_vector, line, boost::is_any_of(delimiter_));
        result_list.push_back(cur_line_vector);
    }
    file.close();

    return result_list; 
}

#endif