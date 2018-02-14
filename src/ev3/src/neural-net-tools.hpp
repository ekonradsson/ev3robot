#pragma once

#include "neural-net.hpp"

typedef struct networkData
{
    std::vector<unsigned> topology;
    std::vector<double> inputMean;
    std::vector<double> inputVariance;
    std::vector< std::vector <std::vector <double > > > weights;
}networkData;

typedef struct dataLine
{
    std::vector<double> input;
    std::vector<double> target;
}dataLine;

bool saveNetwork(std::string fileName, networkData &network);
bool loadNetwork(std::string fileName, networkData &network);

void normalizeTarget(std::vector<double> &in, std::vector<double> &out);
void mean(const std::vector<dataLine> &v, std::vector<double> &result);
void variance(const std::vector<dataLine> &v, std::vector<double> &result);
void limits(const std::vector<dataLine> &v, std::vector<double> &minVector, std::vector<double> &maxVector);
void printWeights(const std::vector< std::vector <std::vector <double > > > weights);

void getSelector(const int type, std::vector<int> &selector);
void normalizeInput(const std::vector<double> &input, const std::vector<double> &mean, const std::vector<double> &variance, std::vector<double> &output);
void getInput(const std_msgs::Float32MultiArray::ConstPtr& statusData, const std::vector<int> &selector, std::vector<double> &input);

double mean ( const std::vector<double> &v );
double variance ( const std::vector<double> &v);
void softMax(const std::vector<double> &v, std::vector<double> &result);

template<class T>
inline void showVectorVals(const std::string label, const std::vector<T> &v)
{
    std::cout << label << " ";
    for(unsigned i = 0; i < v.size(); ++i)
    {
        std::cout << v[i] << " ";
    }
    std::cout << std::endl;
}

template<class T>
inline bool saveVector(std::ofstream &file,  const std::string name, const std::vector <T> &v)
{
    std::string line;
    std::stringstream ss(line);

    ss << name;
    for (int i=0;i<v.size();++i)
    {
        ss << v[i];
        if (i<v.size()-1)
        {
            ss << " ";
        }
    }
    ss << '\n';

    file << ss.str();
}


template<class T>
inline bool loadVector(std::ifstream &file,  const std::string name, std::vector <T> &v)
{
    std::string line;
    std::string label;

    std::getline(file, line);

    std::stringstream ss(line);
    ss >> label;
    if(file.eof())
    {
        std::cout << "Error loading " << name << std::endl;
        return false;
    }
    if (label.compare(name) != 0)
    {
        std::cout << "Expected " << name << ", got " << label << std::endl;
        return false;
    }

    v.clear();

    while(!ss.eof())
    {
        double n;
        ss >> n;
        v.push_back(n);
    }

    return true;
}
