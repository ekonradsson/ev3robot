#include <cstdlib>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <cmath>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "neural-net-tools.hpp"

using namespace std;

bool saveNetwork(std::string fileName, networkData &network)
{
    std::ofstream file (fileName.c_str(), fstream::out | fstream::trunc);

    if (!file || !file.is_open())
    {
        std::cout << "could not create file!\n";
        return false;
    }

    saveVector<unsigned>(file, "topology: ", network.topology);
    saveVector<double>(file, "mean: ", network.inputMean);
    saveVector<double>(file, "variance: ", network.inputVariance);

    for(unsigned l = 0; l < network.weights.size(); ++l)
    {
        for(unsigned n = 0; n < network.weights[l].size(); ++n)
        {
            saveVector<>(file, "weights: ", network.weights[l][n]);
        }
    }
    file.close();
}

bool loadNetwork(std::string fileName, networkData &network)
{
    bool error = false;
    std::ifstream file (fileName.c_str(), fstream::in);

    if (!file || !file.is_open())
    {
        std::cout << "could not open file!\n";
        return false;
    }

    if (loadVector<unsigned>(file, "topology:", network.topology) && (loadVector<double>(file, "mean:", network.inputMean)) && (loadVector<double>(file, "variance:", network.inputVariance)))
    {
        network.weights.clear();

        std::vector <std::vector <double > > layer;
        std::vector <double > node;

        for (int l=0;l<network.topology.size()-1;++l)
        {
            layer.clear();
            for(unsigned n = 0; n < network.topology[l]+1; ++n)
            {
                if (!loadVector<double>(file, "weights:", node))
                {
                    error = true;
                    break;
                } else
                {
                    layer.push_back(node);
                }
            }
            if (error)
            {
                break;
            } else {
                network.weights.push_back(layer);
            }
        }
    } else
    {
        error = true;
    }
    file.close();
    return !error;
}

void normalizeTarget(std::vector<double> &in, std::vector<double> &out)
{
    out.clear();

    for (int i=0; i<in.size();++i)
    {
        out.push_back(scale(in[i], 0.0, 1.0, 0.1, 0.9));
    }
}

void mean(const std::vector<dataLine> &v, std::vector<double> &result)
{
    std::vector<double> temp;
    result.clear();
    assert(v.size()>0 && v[0].input.size()>0);
    for ( int j=0; j < v[0].input.size(); ++j)
    {
        temp.clear();
        for ( int i=0; i < v.size(); ++i)
        {
            temp.push_back(v[i].input[j]);
        }
        result.push_back(mean(temp));
    }
}

void variance(const std::vector<dataLine> &v, std::vector<double> &result)
{
    std::vector<double> temp;
    result.clear();
    assert(v.size()>0 && v[0].input.size()>0);
    for ( int j=0; j < v[0].input.size(); ++j)
    {
        temp.clear();
        for ( int i=0; i < v.size(); ++i)
        {
            temp.push_back(v[i].input[j]);
        }
        result.push_back(variance(temp));
    }
}

void limits(const std::vector<dataLine> &v, std::vector<double> &minVector, std::vector<double> &maxVector)
{
    std::vector<double> temp;
    minVector.clear();
    maxVector.clear();
    assert(v.size()>0 && v[0].input.size()>0);
    for ( int j=0; j < v[0].input.size(); ++j)
    {
        temp.clear();
        for ( int i=0; i < v.size(); ++i)
        {
            temp.push_back(v[i].input[j]);
        }
        auto min = std::min_element(temp.begin(), temp.end());
        auto max = std::max_element(temp.begin(), temp.end());

        minVector.push_back((*min));
        maxVector.push_back((*max));
    }
}

void printWeights(const std::vector< std::vector <std::vector <double > > > weights)
{
    cout << "weights" << endl;

    for(unsigned l = 0; l < weights.size(); ++l)
    {
        for(unsigned n = 0; n < weights[l].size(); ++n)
        {
            for (int i=0; i< weights[l][n].size();++i)
            {
                cout << weights[l][n][i] << " ";
            }
            cout << endl;
        }
        cout << endl << endl;
    }
}

double mean (const std::vector<double> &v )
{
    double return_value = 0.0;

    for ( int i=0; i < v.size(); ++i)
    {
        return_value += v[i];
    }

    return return_value / v.size();
}

double variance (const std::vector<double> &v)
{
    double m = mean(v);
    double s = 0.0;

    for ( int i=0; i < v.size(); ++i)
    {
        s += (v[i] - m)*(v[i] - m);
    }

    return sqrt(s/(v.size()-1));
}

void softMax(const std::vector<double> &v, std::vector<double> &result)
{
    result.clear();
    auto max = std::max_element(v.begin(), v.end());
    double sum = 0;
    for ( int i=0; i < v.size(); ++i)
    {
        sum += exp(v[i]-(*max));
    }

    for ( int i=0; i < v.size(); ++i)
    {
        result.push_back(exp(v[i]-(*max))/sum);
    }
}


void getSelector(const int type, std::vector<int> &selector)
{
    // 0 right
    // 1 left
    // 2 rear
    // 3 roll
    // 4 pitch
    // 5 yaw
    // 6 roll rate
    // 7 pitch rate
    // 8 yaw rate
    // 9 accel x
    // 10 accel y
    // 11 accel z
    // 12 left vel
    // 13 rvel vel

    switch (type)
    {
    case 1: //edge
        selector = {0, 1, 2, 3, 4};
        break;

    case 2: //front edge
        selector = {0, 1};
        break;

    case 4: //impact
        selector = {9, 10};
        break;
    }
}

void getInput(const std_msgs::Float32MultiArray::ConstPtr& statusData, const std::vector<int> &selector, std::vector<double> &input)
{
    input.clear();
    for (int i=0;i<selector.size(); ++i)
    {
        input.push_back(statusData->data[selector[i]]);
        /*
            switch(selector[i])
            {
                case 0:
                case 1:
                case 2:
                    input.back() = rangeToCm(input.back());
                break;
            }
            */
    }
}

void normalizeInput(const std::vector<double> &input, const std::vector<double> &mean, const std::vector<double> &variance, std::vector<double> &output)
{
    assert(input.size()==mean.size());
    assert(input.size()==variance.size());

    output.clear();

    for (int i=0;i<input.size();++i)
    {
        output.push_back( (input[i]-mean[i])/variance[i] );
    }
}
