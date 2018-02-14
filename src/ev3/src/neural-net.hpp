#pragma once

#include <vector>
#include <iostream>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace std;

class TrainingData
{
public:
    TrainingData(const string filename);
    bool isEof(void)
    {
        return m_trainingDataFile.eof();
    }
    void getTopology(vector<unsigned> &topology);

    // Returns the number of input values read from the file:
    unsigned getNextInputs(vector<double> &inputVals);
    unsigned getTargetOutputs(vector<double> &targetOutputVals);
    void reset();
private:
    ifstream m_trainingDataFile;
};


struct Connection
{
    double weight;
    double deltaWeight;
};

class Neuron;

typedef vector<Neuron> Layer;

// ****************** class Neuron ******************

class Neuron
{
public:
    Neuron(unsigned numOutputs, unsigned myIndex);

    static constexpr double learnRate = 0.05; // [0.0...1.0] overall net training rate
    static constexpr double momentum = 0.01; // [0.0...n] multiplier of last weight change [momentum]
    static constexpr double decay = 0.0001;
    void setOutputVal(double val) { m_outputVal = val; }
    double getOutputVal(void) const { return m_outputVal; }
    void feedForward(const Layer &prevLayer);
    void calcOutputGradients(double targetVals);
    void calcHiddenGradients(const Layer &nextLayer);
    void updateInputWeights(Layer &prevLayer);
    void getWeights(vector<double> &v);
    void setWeights(const vector<double> &v);
private:
    static double transferFunction(double x);
    static double transferFunctionDerivative(double x);

    // randomWeight: -1 to 1
    static double randomWeight(void) { return ((rand() / double(RAND_MAX)*2)-1); }
    //static double randomWeight(void) { return (rand() / double(RAND_MAX)); }
    double sumDOW(const Layer &nextLayer) const;
    double m_outputVal;
    vector<Connection> m_outputWeights;
    unsigned m_myIndex;
    double m_gradient;
};

// ****************** class Net ******************
class Net
{
public:
    Net(const vector<unsigned> &topology);
    void feedForward(const vector<double> &inputVals);
    void backProp(const vector<double> &targetVals);
    void getResults(vector<double> &resultVals) const;
    double getRecentAverageError(void) const { return m_recentAverageError; }
    void getWeights(std::vector< std::vector <std::vector <double > > > &v);
    void setWeights(const std::vector<std::vector<std::vector<double> > > &v);

    static constexpr double m_recentAverageSmoothingFactor = 100;
    double m_error;

private:
    vector<Layer> m_layers; //m_layers[layerNum][neuronNum]
    double m_recentAverageError;
};

