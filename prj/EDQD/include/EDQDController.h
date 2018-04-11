/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */



#ifndef EDMAPELITESCONTROLLER_H
#define EDMAPELITESCONTROLLER_H

#include "RoboroboMain/common.h"
#include "RoboroboMain/roborobo.h"
#include "Controllers/Controller.h"
#include <neuralnetworks/NeuralNetwork.h>
#include "EDQD/include/EDQDSharedData.h"
#include "EDQD/include/EDQDMap.h"
//#include "neuralnetworks/nn2/nn.hpp"

using namespace Neural;
//using namespace nn;
//using namespace EDQD;

class EDQDRobotWorldModel;
class EDQDMap;

class EDQDController : public Controller
{
//private:
//	static int _nextId;

protected:
    int _iteration;
    int _birthdate; // evaluation when this controller was initialized.
        
    bool _isListening;
    int _notListeningDelay;
    int _listeningDelay;

//    int _nbGenomeTransmission;
    int _nbGenomeTransmission;
    
    std::vector<double> _parameters;
    std::string _nnType;
    std::vector<int> _nbHiddenNeuronsPerLayer;
    std::vector<int> _nbBiasNeuronsPerLayer;
    NeuralNetwork* nn;
//    NN<Neuron<PfWSum<>, AfTanh<float> >, Connection<> > nn;

//    std::vector<double> inputs;

    void createNN();
    
    //bool _isAlive; // agent stand still if not.
    bool _isNewGenome;

    void selectRandomGenomeFromRandomMap();
    void selectRandomGenomeFromMergedMap();
    void selectRandomGenome();
    void selectFirstGenome();
    void selectBestGenome();
    void selectFitProp();
    
    void mutateGaussian( float sigma );
    void mutateUniform();
    
    void mutateSigmaValue();
    
    std::vector<double> getInputs();
    void setIOcontrollerSize();
    
    virtual void initController(); // called by resetRobot
    virtual void stepController();
    
    void stepEvolution();
    
    virtual void broadcastGenome();
    
    void loadNewGenome();
    
    unsigned int computeRequiredNumberOfWeights();
    
    //        void setAliveStatus( bool isAlive ) { _isAlive = isAlive; }
    bool getNewGenomeStatus() { return _isNewGenome; }
    void setNewGenomeStatus( bool __status ) { _isNewGenome = __status; }
    
    // map

    EDQDMap *_map;
    EDQDMap *_mergedMap;

    // incoming genomes reservoir
    
    std::map< std::pair<int,int>, std::vector<double> > _genomesList;
    std::map< std::pair<int,int>, float > _sigmaList;
    std::map< std::pair<int,int>, float > _fitnessValuesList;
    
    std::map< int, EDQDMap* > _mapList;

    // current genome
    
    std::vector<double> _currentGenome;
    float _currentSigma;
    
    std::pair<int,int> _ancestor;

    // ANN
    double _minValue;
    double _maxValue;
    unsigned int _nbInputs;
    unsigned int _nbOutputs;
    unsigned int _nbHiddenLayers;
    std::vector<unsigned int>* _nbNeuronsPerHiddenLayer;
    
    // logging purpose
    double _Xinit;
    double _Yinit;
    double _Xlast;
    double _Ylast;
    double _dSumTravelled;
    double _dMaxTravelled;
    double _dPotMaxTravelled;
    int _nbComTx;
    int _nbComRx;

    int _currentCellId;

    
    std::map<int,int> _objectCounters;

//    bool storeGenome(EDQD::Entry& mapEntry);
    bool storeGenome(std::vector<double> genome, std::pair<int,int> senderId, float sigma, float fitness, EDQDMap* map);
    bool storeMap(EDQDMap* map, int senderId);
	void reset();
    
    void clearReservoir(); // clear genomesList, sigmaList, fitnessesList and birthdayList
    
    virtual void logCurrentState();
    
    virtual void mapGenotypeToPhenotype();
    
    virtual void performSelection();
    virtual void performVariation();
    
    virtual void resetFitness();
    virtual void updateFitness();
    
public:
    
    EDQDController(EDQDRobotWorldModel *wm);
    ~EDQDController();
    
    void step();
    
//    static int getNextId();
    void writeEOGEntry(LogManager* lm);

    int getBirthdate() { return _birthdate; }
    
    bool isListening() { return _isListening; }
    
    virtual double getFitness();

    EDQDRobotWorldModel* getWorldModel() { return (EDQDRobotWorldModel*)_wm; }
    
	const std::map<int,int>& getObjectCounters() const {
		return _objectCounters;
	}

	void incObjectCounter(int group) {
		_objectCounters[group]++;
	}

	void resetObjectCounter() {
		for (auto &pog : gPhysicalObjectGroups)
		{
			_objectCounters[pog->getId()] = 0;
		}
	}

	double getMaxTravelled() const {
		return _dMaxTravelled;
	}

	double getPotMaxTravelled() const {
		return _dPotMaxTravelled;
	}

	double getSumTravelled() const {
		return _dSumTravelled;
	}

	EDQDMap*& getMap() {
		return (_map);
	}

	const std::vector<double>& getCurrentGenome() const {
		return _currentGenome;
	}

	float getCurrentSigma() const {
		return _currentSigma;
	}

	void resetPotMaxTravelled();

	int updateCellId();
};


#endif

