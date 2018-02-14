
//#define DEBUG_DRIVE
//#define DEBUG_DRIVE_PID
//#define DEBUG_HUB
//#define DEBUG_RC
//#define DEBUG_IMU
//#define DEBUG_RECEIVER
//#define DEBUG_CAMERA


/*
networkData edgeNetwork = {
    //avg error: 0.01244
    .topology = {6, 7, 3},
    .inputMean = {244.9, 234.7, 220.6, 199.2, -0.003602, 0.01646},
    .inputVariance = {70.7, 71.02, 61.93, 57.35, 0.0366, 0.02519},
    .weights = {
        {
            {0.06845 ,1.476 ,-0.1526 ,4.662 ,-0.9165 ,1.297 ,0.6614 },
            {0.3988 ,1.254 ,0.1047 ,3.428 ,0.9148 ,-0.4092 ,-0.7023 },
            {-1.408 ,1.424 ,-0.1569 ,4.048 ,-1.113 ,-0.192 ,0.09274 },
            {0.6131 ,1.283 ,0.3715 ,3.722 ,0.4269 ,-0.2148 ,0.8991 },
            {-0.295 ,0.1916 ,-0.1373 ,-0.538 ,0.01559 ,-0.4728 ,-0.1904 },
            {0.1334 ,-0.1697 ,-0.3321 ,0.6366 ,0.02496 ,0.1043 ,-0.1607 },
            {0.2735 ,3.877 ,-0.1725 ,-3.507 ,0.5957 ,-0.3026 ,-0.7737 }
        },
        {
            {-4.858e-18 ,0.005216 ,-0.3743 },
            {-1.335e-18 ,-0.0004255 ,0.5671 },
            {-6.099e-19 ,-0.0005676 ,-0.1492 },
            {5.563e-19 ,0.6911 ,-0.6594 },
            {1.17e-18 ,0.001644 ,0.4875 },
            {2.03e-18 ,0.002876 ,0.2625 },
            {9.153e-19 ,-0.002842 ,0.1967 },
            {0.1003 ,0.7825 ,0.2068 }
        }
    }
};
*/
/*

    std::vector< std::vector <std::vector <double > > > weights;

*/

/*
Net *edgeDetector;
*/

/*

    edgeDetector = new Net(edgeNetwork.topology);
    printWeights(edgeNetwork.weights);
    edgeDetector->setWeights(edgeNetwork.weights);



input.clear();

for (int i=0;i<ev3ros::sensorCount + ev3ros::sonarCount;++i)
{
    input.push_back(hubReadings[i]);
}

input.push_back(roll);
input.push_back(pitch);

normalizeInput(input, edgeNetwork.inputMean, edgeNetwork.inputVariance, inputNorm);

edgeDetector->feedForward(inputNorm);

// Collect the net's actual results:
edgeDetector->getResults(resultVals);

softMax(resultVals, resultNorm);

int state = std::distance(resultNorm.begin(), std::max_element (resultNorm.begin(), resultNorm.end()));
if (state != lastState)
{
    cout << "State : " << state << " " << std::setprecision (4) << resultNorm[state]*100 << "%" << std::endl;
    lastState = state;
}
*/
