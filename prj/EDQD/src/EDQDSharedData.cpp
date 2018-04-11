/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */



#include "EDQD/include/EDQDSharedData.h"
#include "RoboroboMain/roborobo.h"

double EDQDSharedData::gSigmaMin = 0.0;
double EDQDSharedData::gProbaMutation = 0.0;
double EDQDSharedData::gUpdateSigmaStep = 0.0;
double EDQDSharedData::gSigmaRef = 0.0; // reference value of sigma
double EDQDSharedData::gSigmaMax = 0.0; // maximal value of sigma
int EDQDSharedData::gEvaluationTime = 0; // how long a controller will be evaluated on a robot

bool EDQDSharedData::gSynchronization = true;

bool EDQDSharedData::gEnergyRequestOutput = 1;

double EDQDSharedData::gMonitorPositions;

bool EDQDSharedData::gPropertiesLoaded = false; // global variable local to file -- TODO: move specific properties loader in dedicated WorldObserver

int EDQDSharedData::gNbHiddenLayers = 1;
int EDQDSharedData::gNbNeuronsPerHiddenLayer = 5;
int EDQDSharedData::gNeuronWeightRange = 800;

bool EDQDSharedData::gSnapshots = true; // take snapshots
int EDQDSharedData::gSnapshotsFrequency = 50; // every N generations

int EDQDSharedData::gControllerType = -1; // cf. header for description

bool EDQDSharedData::gLimitGenomeTransmission = false; // default: do not limit.
int EDQDSharedData::gMaxNbGenomeTransmission = 65535; // default: arbitrarily set to 65535.

int EDQDSharedData::gSelectionMethod = 0; // default: random selection
bool EDQDSharedData::gEDQDMapSelection = true; // default: enabled

int EDQDSharedData::gNotListeningStateDelay = 0;    // -1: infinite ; 0: no delay ; >0: delay
int EDQDSharedData::gListeningStateDelay = -1;      // -1: infinite ; 0: no delay ; >0: delay (ignored if gNotListeningStateDelay=-1)

bool EDQDSharedData::gLogGenome = false;
bool EDQDSharedData::gLogGenomeSnapshot = false;

double EDQDSharedData::gIndividualMutationRate = 1.0;

int EDQDSharedData::gMutationOperator = 1; // 0: uniform, 1: gaussian

double EDQDSharedData::gSigma = 0.01; // 0.01 is just some random value.

bool EDQDSharedData::gEDQDOnlyKeepMapsForGeneration = true;
bool EDQDSharedData::gEDQDIncludeLocalMapForSelectionMerge = false;
bool EDQDSharedData::gEDQDMergeAllMapsIntoLocalMap = false;

//const int EDQDSharedData::gEDQDNbOfDimensions = 2; // commented out as needs to be set at compile time. only use header declaration.
//const int EDQDSharedData::gEDQDNbOfIntervals = 5; // commented out as needs to be set at compile time. only use header declaration.
double EDQDSharedData::gEDQDFitEpsilon = 1.0;

bool EDQDSharedData::gPhysicalObjectRepositioning = false;
int EDQDSharedData::gPhysicalObjectRepositionStartTime = EDQDSharedData::gEvaluationTime * 0.1;
int EDQDSharedData::gPhysicalObjectRepositionInterval = EDQDSharedData::gEvaluationTime * 0.05;

int EDQDSharedData::gArenaRadius = 500;

std::ofstream EDQDSharedData::gEOGLogFile;
LogManager* EDQDSharedData::gEOGLogManager = NULL;

std::ofstream EDQDSharedData::gMapsLogFile;
LogManager* EDQDSharedData::gMapsLogManager = NULL;

std::ofstream gLitelogFile;
LogManager *gLitelogManager = NULL;
