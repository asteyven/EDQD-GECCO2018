/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 * 20171218(AS): 	- removed object types
 *  				- added object groups
 *  				- removed additional agent inputs
 */

#include "EDQD/include/EDQDController.h"
#include "EDQD/include/EDQDWorldObserver.h"
#include "EDQD/include/EDQDSharedData.h"
#include "EDQD/include/EDQDRobotWorldModel.h"
#include "RoboroboMain/roborobo.h"
#include "WorldModels/RobotWorldModel.h"
#include "World/World.h"
#include <neuralnetworks/MLP.h>
#include <neuralnetworks/Perceptron.h>
#include <neuralnetworks/Elman.h>
#include "Utilities/Misc.h"

//#include "neuralnetworks/nn2/mlp.hpp"

using namespace Neural;
using namespace EDQD;

//int EDQDController::_nextId = 0;

EDQDController::EDQDController( EDQDRobotWorldModel *wm )
{
    _wm = wm;
    
    nn = NULL;

    _map = new EDQDMap();
    _mergedMap = new EDQDMap();
//    _map = getMap();

    // evolutionary engine
    
    _minValue = -1.0;
    _maxValue = 1.0;
    
    _currentSigma = EDQDSharedData::gSigmaRef;
    
    // behaviour
    
    _iteration = 0;
    
    _birthdate = 0;
    _ancestor = std::make_pair(_wm->getId(),getBirthdate());

    _isListening = true;
    _notListeningDelay = EDQDSharedData::gNotListeningStateDelay;
    _listeningDelay = EDQDSharedData::gListeningStateDelay;
    
    _nbGenomeTransmission = 0;
    
    _dSumTravelled = 0.0;
    _dMaxTravelled = 0.0;
    _dPotMaxTravelled = 0.0;
    _nbComTx = 0;
    _nbComRx = 0;
    _currentCellId = 0;

    if ( gEnergyLevel )
        _wm->setEnergyLevel(gEnergyInit);
    
    if ( gNbOfLandmarks > 0 )
        _wm->updateLandmarkSensor(); // wrt closest landmark
    
    reset();
    resetFitness();
    
    _wm->setAlive(true);
    _wm->setRobotLED_colorValues(255, 0, 0);
    
}

EDQDController::~EDQDController()
{
//	delete _map;
    _parameters.clear();
    delete _nbNeuronsPerHiddenLayer;
    delete nn;
    nn = NULL;

}

void EDQDController::step() // handles control decision and evolution (but: actual movement is done in roborobo's main loop)
{
    _iteration++;
    
    // * step evolution
    
    stepEvolution();
    
    // * step controller
    
    if ( _wm->isAlive() )
    {
        stepController();
        updateFitness();
    }
    else
    {
        _wm->_desiredTranslationalValue = 0.0;
        _wm->_desiredRotationalVelocity = 0.0;
    }
    
    // * updating listening state
    
    if ( _wm->isAlive() == false )
    {
        assert ( _notListeningDelay >= -1 ); // -1 means infinity
        
        if ( _notListeningDelay > 0 )
        {
            _notListeningDelay--;
            
            if ( _notListeningDelay == 0 )
            {
                
                _listeningDelay = EDQDSharedData::gListeningStateDelay;
                
                if ( _listeningDelay > 0 || _listeningDelay == -1 )
                {
                    _isListening = true;
                    
                    _wm->setRobotLED_colorValues(0, 255, 0); // is listening
                    
                    std::string sLog = std::string("");
                    sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",status,listening\n";
                    gLogManager->write(sLog);
                    gLogManager->flush();
                }
                else
                {
                    std::string sLog = std::string("");
                    sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",status,inactive\n"; // never listen again.
                    gLogManager->write(sLog);
                    gLogManager->flush();
                }
            }
        }
        else
        {
            if ( _notListeningDelay != -1 && _listeningDelay > 0 )
            {
                assert ( _isListening == true );
                
                _listeningDelay--;
                
                if ( _listeningDelay == 0 )
                {
                    _isListening = false;
                    // Logging: robot is dead
                    std::string sLog = std::string("");
                    sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",status,inactive\n";
                    gLogManager->write(sLog);
                    gLogManager->flush();

                    _notListeningDelay = -1; // agent will not be able to be active anymore
                    _wm->setRobotLED_colorValues(0, 0, 255); // is not listening
                    
                    reset(); // destroy then create a new NN
                    
                    _wm->setAlive(false);
                }
            }
        }
    }
}


// ################ ######################## ################
// ################ ######################## ################
// ################ BEHAVIOUR METHOD(S)      ################
// ################ ######################## ################
// ################ ######################## ################


std::vector<double> EDQDController::getInputs(){
    
    
    // WHAT FOLLOWS IS AN EXAMPLE OF LOADING A NN-CONTROLER WITH A FULL-FLEDGE SENSORY INPUT INFORMATION
    // Rewrite to match your own extended input scheme, if needed.
    // Note that you may tune it on/off using gExtendedSensoryInputs defined in the properties file.
    // When modifying this code, dont forget to update the initialization in the reset() method
    // Example:
    //      - you may want to distinguish between robot's groups (if more than 1)
    //      - you may want to restrict the number of objects that can be identified (if not all possible objects are in use)
    //      - you may want to compress inputs (e.g. binary encoding instead of one-input-per-object-type.
    //      - (...)
    //
    // In the following, sensory inputs for each sensor are ordered (and filled in) as follow:
    // - N range sensors
    //      - distance to obstacle (max if nothing)
    //      IF extendedInputs is True:
    //          - [0...N_physicalobjecttypes] Is it an object of type i? (0: no, 1: yes) -- type: 0 (round), 1 (energy item), 2 (gate), 3 (switch), ...? (cf. PhysicalObjectFactory)
    //          - is it a robot? (1 or 0)
    //          - is it from the same group? (1 or 0)
    //          - relative orientation wrt. current robot (relative orientation or 0 if not a robot)
    //          - is it a wall? (ie. a non-distinguishible something) (1 or 0)
    // - floor sensor, red value
    // - floor sensor, green value
    // - floor sensor, blue value
    // - relative direction to the closest landmark
    // - distance to the closest landmark
    // - normalized energy level (if any)
    //
    // => number of sensory inputs: N * rangeSensors + 6, with rangeSensors varying from 1 to many, if extended sensory inputs are on.
    

    
    std::vector<double> inputs;
    
    inputs.reserve(_nbInputs);

    // distance sensors
    for(int i  = 0; i < _wm->_cameraSensorsNb; i++)
    {
        inputs.push_back( _wm->getDistanceValueFromCameraSensor(i) / _wm->getCameraSensorMaximumDistanceValue(i) );
        
        if ( gExtendedSensoryInputs ) // EXTENDED SENSORY INPUTS: code provided as example, should be rewritten to suit your need.
        {
            int objectId = _wm->getObjectIdFromCameraSensor(i);
            
            // input: physical object? which type?
            if ( PhysicalObject::isInstanceOf(objectId) )
            {
            	// 20171218(AS): removed object types and added object groups
//                int nbOfTypes = PhysicalObjectFactory::getNbOfTypes();
//                for ( int j = 0 ; j != nbOfTypes ; j++ )
//                {
//                    if ( j == gPhysicalObjects[objectId - gPhysicalObjectIndexStartOffset]->getType() )
//                        inputs.push_back( 1 ); // match
//                    else
//                        inputs.push_back( 0 );
//                }

                int __objectGroup = gPhysicalObjects[objectId - gPhysicalObjectIndexStartOffset]->getGroup();
				for ( auto &it : gPhysicalObjectGroups )
				{
					if ( it->getId() == __objectGroup )
						inputs.push_back( 1 ); // match
					else
						inputs.push_back( 0 );
				}
            }
            else
            {
            	// 20171218(AS): 	- removed object types
            	//					- added object groups
//                // not a physical object. But: should still fill in the inputs (with zeroes)
//                int nbOfTypes = PhysicalObjectFactory::getNbOfTypes();
//                for ( int j = 0 ; j != nbOfTypes ; j++ )
//                {
//                    inputs.push_back( 0 );
//                }
                // set all object group inputs to zero
                for ( int j = 0 ; j != gNbOfPhysicalObjectGroups ; j++ )
				{
					inputs.push_back( 0 );
				}
            }
            
            // input: another agent? If yes: same group?
            if ( Agent::isInstanceOf(objectId) )
            {
                // this is an agent
                inputs.push_back( 1 );
                
                // 20171218(AS): removed additional agent inputs
//                // same group?
//                if ( gWorld->getRobot(objectId-gRobotIndexStartOffset)->getWorldModel()->getGroupId() == _wm->getGroupId() )
//                {
//                    inputs.push_back( 1 ); // match: same group
//                }
//                else
//                {
//                    inputs.push_back( 0 ); // not the same group
//                }
//
//                // relative orientation? (ie. angle difference wrt. current agent)
//                double srcOrientation = _wm->_agentAbsoluteOrientation;
//                double tgtOrientation = gWorld->getRobot(objectId-gRobotIndexStartOffset)->getWorldModel()->_agentAbsoluteOrientation;
//                double delta_orientation = - ( srcOrientation - tgtOrientation );
//                if ( delta_orientation >= 180.0 )
//                    delta_orientation = - ( 360.0 - delta_orientation );
//                else
//                    if ( delta_orientation <= -180.0 )
//                        delta_orientation = - ( - 360.0 - delta_orientation );
//                inputs.push_back( delta_orientation/180.0 );
            }
            else
            {
                inputs.push_back( 0 ); // not an agent...
                // 20171218(AS): removed additional agent inputs
//                inputs.push_back( 0 ); // ...therefore no match wrt. group.
//                inputs.push_back( 0 ); // ...and no orientation.
            }
            
            // input: wall or empty?
            if ( objectId >= 0 && objectId < gPhysicalObjectIndexStartOffset ) // not empty, but cannot be identified: this is a wall.
                inputs.push_back( 1 );
            else
                inputs.push_back( 0 ); // nothing. (objectId=-1)
        }
    }
    
    
    // floor sensor
    inputs.push_back( (double)_wm->getGroundSensor_redValue()/255.0 );
    inputs.push_back( (double)_wm->getGroundSensor_greenValue()/255.0 );
    inputs.push_back( (double)_wm->getGroundSensor_blueValue()/255.0 );
    
    // landmark (closest landmark)
    if ( gNbOfLandmarks > 0 )
    {
        _wm->updateLandmarkSensor(); // update with closest landmark
        inputs.push_back( _wm->getLandmarkDirectionAngleValue() );
        inputs.push_back( _wm->getLandmarkDistanceValue() );
    }
    
    
    // energy level
    if ( gEnergyLevel )
    {
        inputs.push_back( _wm->getEnergyLevel() / gEnergyMax );
    }
    
    return inputs;
}

void EDQDController::stepController()
{

    // ---- compute and read out ----
    
    nn->setWeights(_parameters); // set-up NN
//    nn.set_all_weights(_parameters);
    std::vector<double> inputs = getInputs(); // Build list of inputs (check properties file for extended/non-extended input values
    
    nn->setInputs(inputs);
    
    nn->step();
    
    std::vector<double> outputs = nn->readOut();
    
    // std::cout << "[DEBUG] Neural Network :" << nn->toString() << " of size=" << nn->getRequiredNumberOfWeights() << std::endl;
    
    _wm->_desiredTranslationalValue = outputs[0];
    _wm->_desiredRotationalVelocity = outputs[1];
    
    if ( EDQDSharedData::gEnergyRequestOutput )
    {
        _wm->setEnergyRequestValue(outputs[2]);
    }
    
    // normalize to motor interval values
    _wm->_desiredTranslationalValue = _wm->_desiredTranslationalValue * gMaxTranslationalSpeed;
    _wm->_desiredRotationalVelocity = _wm->_desiredRotationalVelocity * gMaxRotationalSpeed;
    
}


void EDQDController::createNN()
{
    setIOcontrollerSize(); // compute #inputs and #outputs
    
    if ( nn != NULL ) // useless: delete will anyway check if nn is NULL or not.
        delete nn;
    
    switch ( EDQDSharedData::gControllerType )
    {
        case 0:
        {
            // MLP
            nn = new MLP(_parameters, _nbInputs, _nbOutputs, *(_nbNeuronsPerHiddenLayer));
//            nn = Mlp<Neuron<PfWSum<>, AfSigmoidNoBias<> >, Connection<> >(
//            				_nbInputs,
//							_nbHiddenLayers * EDQDSharedData::gNbNeuronsPerHiddenLayer,
//							_nbOutputs);
//            nn.init();
            break;
        }
        case 1:
        {
            // PERCEPTRON
            nn = new Perceptron(_parameters, _nbInputs, _nbOutputs);
            break;
        }
        case 2:
        {
            // ELMAN
            nn = new Elman(_parameters, _nbInputs, _nbOutputs, *(_nbNeuronsPerHiddenLayer));
            break;
        }
        default: // default: no controller
            std::cerr << "[ERROR] gController type unknown (value: " << EDQDSharedData::gControllerType << ").\n";
            exit(-1);
    };
}


unsigned int EDQDController::computeRequiredNumberOfWeights()
{
    unsigned int res = nn->getRequiredNumberOfWeights();
    return res;
}

// ################ ######################## ################
// ################ ######################## ################
// ################ EVOLUTION ENGINE METHODS ################
// ################ ######################## ################
// ################ ######################## ################

void EDQDController::stepEvolution()
{
	if( gWorld->getIterations() > 0 && gWorld->getIterations() % EDQDSharedData::gEvaluationTime == 0 )
    {
        // * lifetime ended: replace genome (if possible)
    	// !!! add needs to happen before new genome is loaded to ensure the map has an entry.
    	// moved to agent observer
//    	_map->add(_wm->getId(),
//    			this,
//    			_currentGenome,
//				_currentSigma
//				);
//    	std::cout << _dSumTravelled << " " << _dMaxTravelled << " " << _dPotMaxTravelled << std::endl; // DEBUG

    	loadNewGenome();
        _nbGenomeTransmission = 0;
        resetFitness();
    }
    else
    {
        // * broadcasting genome : robot broadcasts its genome to all neighbors (contact-based wrt proximity sensors)
        // note: no broadcast if last iteration before replacement -- this is enforced as roborobo update method is random-asynchroneous. This means that robots broadcasting may transmit genomes to robot already at the next generation depending on the update ordering (should be avoided).
        if ( _wm->isAlive() == true && gRadioNetwork )  	// only if agent is active (ie. not just revived) and deltaE>0.
        {
            broadcastGenome();
        }

        int __tmpMaxDistance = getEuclideanDistance( _wm->getXReal(), _wm->getYReal(), _Xinit, _Yinit );
        _dSumTravelled += getEuclideanDistance( _wm->getXReal(), _wm->getYReal(), _Xlast, _Ylast ); //remark: incl. squareroot.
        _Xlast = _wm->getXReal();
		_Ylast = _wm->getYReal();
		if ( _dMaxTravelled < __tmpMaxDistance ) {
			_dMaxTravelled = __tmpMaxDistance;
		}
    }
    
    // log the genome (only at the second iteration during snapshot time)
    if ( EDQDSharedData::gLogGenomeSnapshot && gWorld->getIterations() % ( EDQDSharedData::gEvaluationTime * EDQDSharedData::gSnapshotsFrequency ) == 1 )
    {
        // Logging: full genome
        std::string sLog = std::string("");
        sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",genome,";
        if ( _wm->isAlive() )
        {
            for(unsigned int i=0; i<_currentGenome.size(); i++)
            {
                sLog += std::to_string(_currentGenome[i]);
                if ( i < _currentGenome.size()-1 )
                    sLog += ",";
            }
        }
        else
            sLog += "n/a"; // do not write genome
        
        sLog += "\n";
        gLogManager->write(sLog);
        gLogManager->flush();
    }
    
    
    if ( getNewGenomeStatus() ) // check for new NN parameters
    {
        mapGenotypeToPhenotype();
        setNewGenomeStatus(false);
    }
}

void EDQDController::mapGenotypeToPhenotype()
{
    _parameters.clear();
    _parameters.reserve(_currentGenome.size());
    _parameters = _currentGenome;
}

void EDQDController::performVariation()
{
    if ( EDQDSharedData::gIndividualMutationRate > random() ) // global mutation rate (whether this genome will get any mutation or not) - default: always
    {
        switch ( EDQDSharedData::gMutationOperator )
        {
            case 0:
                mutateUniform();
                break;
            case 1:
                mutateSigmaValue(); // self-contained sigma. mutated before use (assume: performVariation is called after selection of new current genome)
                mutateGaussian(_currentSigma); // original MEDEA [ppsn2010], as used before year 2015
                break;
            case 2:
                mutateGaussian(EDQDSharedData::gSigma); // fixed mutation rate
                break;
            default:
                std::cerr << "[ERROR] unknown variation method (gMutationOperator = " << EDQDSharedData::gMutationOperator << ")\n";
                exit(-1);
        }
    }
    
}

void EDQDController::selectRandomGenomeFromRandomMap() // if called, assume genomeList.size()>0
{
    int randomIndex = randint()%_mapList.size();

    std::map<int, EDQDMap* >::iterator it = _mapList.begin();
    while (randomIndex !=0 )
    {
        it ++;
        randomIndex --;
    }
    
    assert( (*(*it).second).hasEntry() == true );

    EDQD::behav_index_t __index;
	do {
		__index = (*(*it).second).getRandomIndex();
	} while ( (*(*it).second).get(__index)->genome.size() == 0 );

    _currentGenome = (*it).second->get(__index)->genome;
    _currentSigma = (*it).second->get(__index)->sigma;
    _birthdate = gWorld->getIterations();
    
    _ancestor = (*it).second->get(__index)->id;

    setNewGenomeStatus(true); 
}

void EDQDController::selectRandomGenomeFromMergedMap() // if called, assume genomeList.size()>0
{

//	std::cout << "[DEBUG] :: receivedMaps:" << _mapList.size() << std::endl;
    std::map<int, EDQDMap* >::iterator it = _mapList.begin();
    while ( it != _mapList.end() ) {
    	it->second->mergeInto(*_mergedMap);
//    	for (size_t i = 0; i < it->second->getMap().num_elements(); i ++) {
//    		if ( _mergedMap->getMap().data()[i].fitness == -1 ||
//    				_mergedMap->getMap().data()[i] < it->second->getMap().data()[i] )
//    		{
//    			_mergedMap->getMap().data()[i] = it->second->getMap().data()[i];
//    		}
//    	}
//    	std::cout << "[DEBUG] :: receivedMap: " << (*it->second).getNumFilledCells() << std::endl << (*it->second) << std::endl;
        it ++;
    }
//    _mergedMap->updateNumFilledCells();
    /*  // DEBUG
	std::cout << "[DEBUG] :: mergedMap: " << mergedMap.getNumFilledCells() << std::endl << mergedMap << std::endl;
    */

//    assert( mergedMap.hasEntry() == true );

    EDQD::behav_index_t __index;
	do {
		__index = _mergedMap->getRandomIndex();
	} while ( _mergedMap->get(__index)->genome.size() == 0 );

    _currentGenome = _mergedMap->get(__index)->genome;
    _currentSigma = _mergedMap->get(__index)->sigma;
    _birthdate = gWorld->getIterations();

    _ancestor = _mergedMap->get(__index)->id;

    setNewGenomeStatus(true);
}

void EDQDController::selectRandomGenome() // if called, assume genomeList.size()>0
{
    int randomIndex = randint()%_genomesList.size();

    std::map<std::pair<int,int>, std::vector<double> >::iterator it = _genomesList.begin();
    while (randomIndex !=0 )
    {
        it ++;
        randomIndex --;
    }

    _currentGenome = (*it).second;
    _currentSigma = _sigmaList[(*it).first];
    _birthdate = gWorld->getIterations();

    _ancestor = (*it).first;

    setNewGenomeStatus(true);
}

// used for debugging
void EDQDController::selectFirstGenome()  // if called, assume genomeList.size()>0
{
    _currentGenome = (*_genomesList.begin()).second;
    _currentSigma = _sigmaList[(*_genomesList.begin()).first];
    _birthdate = gWorld->getIterations();

    _ancestor = (*_genomesList.begin()).first;

    setNewGenomeStatus(true);
}

void EDQDController::selectBestGenome()
{
    std::pair<int,int> bestId;

    std::map<std::pair<int,int>, float >::iterator fitnessesIt = _fitnessValuesList.begin();

    float bestFitnessValue = (*fitnessesIt).second;
    bestId = (*fitnessesIt).first;

    ++fitnessesIt;

    int nbSimilar = 0;

    for ( int i = 1 ; fitnessesIt != _fitnessValuesList.end(); ++fitnessesIt, i++)
    {
        if ( (*fitnessesIt).second >= bestFitnessValue )
        {
            if ( (*fitnessesIt).second > bestFitnessValue )
            {
                bestFitnessValue = (*fitnessesIt).second;
                bestId = (*fitnessesIt).first;
                nbSimilar = 0;
            }
            else
            {
                nbSimilar++;
            }
        }
    }

    if ( nbSimilar > 0 ) // >1 genomes have the same fitness best value. Pick randomly among them
    {
        int count = 0;
        int randomPick = randint() % ( nbSimilar + 1 );

        if ( randomPick != 0 ) // not already stored (i.e. not the first one)
        {
            fitnessesIt = _fitnessValuesList.begin();
            for ( int i = 0 ; ; ++fitnessesIt, i++)
            {
                if ( (*fitnessesIt).second == bestFitnessValue )
                {
                    if ( count == randomPick )
                    {
                        bestId = (*fitnessesIt).first;
                        break;
                    }
                    count++;
                }
            }
        }
    }

    _birthdate = gWorld->getIterations();

    _currentGenome = _genomesList[bestId];
    _currentSigma = _sigmaList[bestId];

    _ancestor = bestId;

    setNewGenomeStatus(true);
}

void EDQDController::selectFitProp()
{
    std::pair<int,int> selectId;

    // compute sum of fitness

    float sumOfFit = 0;

    std::map<std::pair<int,int>, float >::iterator fitnessesIt = _fitnessValuesList.begin();

    for ( ; fitnessesIt != _fitnessValuesList.end(); ++fitnessesIt )
    {
        sumOfFit += (*fitnessesIt).second;
    }

    // randomly draw a value in [0,sum_of_fitness] -- assume maximisation

    float fitnessTarget = random()*sumOfFit;

    // find the parent

    float currentSum = 0;

    fitnessesIt = _fitnessValuesList.begin();

    for ( ; fitnessesIt != _fitnessValuesList.end(); ++fitnessesIt )
    {
        currentSum += (*fitnessesIt).second;
        if ( currentSum >= fitnessTarget )
        {
            selectId = (*fitnessesIt).first;
            break;
        }
    }

    // update current genome with selected parent (mutation will be done elsewhere)

    _birthdate = gWorld->getIterations();

    _currentGenome = _genomesList[selectId];
    _currentSigma = _sigmaList[selectId];

    _ancestor = selectId;

    setNewGenomeStatus(true);
}

/* manage storage of a map received from a neighbour
 *
 * Note that in case of multiple encounters with the same robot (same id, same "birthdate"), genome is stored only once, and last known fitness value is stored (i.e. updated at each encounter).
 */
bool EDQDController::storeMap(EDQDMap* map, int senderId) // fitness is optional (default: 0)
{
    if ( !_isListening )
    {
        return false; // current agent is not listening: do nothing.
    }
    else
    {
        _nbComRx++;

        std::map<int, EDQDMap* >::const_iterator it = _mapList.find(senderId);

        if ( it != _mapList.end() ) // this exact agent's map is already stored. Exact means: same robot, same generation. Then: update fitness value (the rest in unchanged)
        {
            return false;
        }
        else
        {
        	_mapList[senderId] = map;
            return true;
        }
    }
}

/* manage storage of a genome received from a neighbour
 *
 * Note that in case of multiple encounters with the same robot (same id, same "birthdate"), genome is stored only once, and last known fitness value is stored (i.e. updated at each encounter).
 */
bool EDQDController::storeGenome(std::vector<double> genome, std::pair<int,int> senderId, float sigma, float fitness, EDQDMap* map) // fitness is optional (default: 0)
{
    if ( !_isListening )
    {
        return false; // current agent is not listening: do nothing.
    }
    else
    {
    	_nbComRx++;

        std::map<std::pair<int,int>, std::vector<double> >::const_iterator it = _genomesList.find(senderId);

        if ( it != _genomesList.end() ) // this exact agent's genome is already stored. Exact means: same robot, same generation. Then: update fitness value (the rest in unchanged)
        {
            _fitnessValuesList[senderId] = fitness; // update with most recent fitness (IMPLEMENTATION CHOICE) [!n]
            return false;
        }
        else
        {
        	_genomesList[senderId].clear();
            _genomesList[senderId] = genome;
            _sigmaList[senderId] = sigma;
            _fitnessValuesList[senderId] = fitness;

            _mapList[senderId.first] = map;
            return true;
        }
    }
}

void EDQDController::mutateGaussian(float sigma) // mutate within bounds.
{
    _currentSigma = sigma;
    
    for (unsigned int i = 0 ; i != _currentGenome.size() ; i++ )
    {
        double value = _currentGenome[i] + randgaussian()*_currentSigma;
        // bouncing upper/lower bounds
        if ( value < _minValue )
        {
            double range = _maxValue - _minValue;
            double overflow = - ( (double)value - _minValue );
            overflow = overflow - 2*range * (int)( overflow / (2*range) );
            if ( overflow < range )
                value = _minValue + overflow;
            else // overflow btw range and range*2
                value = _minValue + range - (overflow-range);
        }
        else if ( value > _maxValue )
        {
            double range = _maxValue - _minValue;
            double overflow = (double)value - _maxValue;
            overflow = overflow - 2*range * (int)( overflow / (2*range) );
            if ( overflow < range )
                value = _maxValue - overflow;
            else // overflow btw range and range*2
                value = _maxValue - range + (overflow-range);
        }
        
        _currentGenome[i] = value;
    }
    
}


void EDQDController::mutateUniform() // mutate within bounds.
{
    for (unsigned int i = 0 ; i != _currentGenome.size() ; i++ )
    {
        float randomValue = float(randint()%100) / 100.0; // in [0,1[
        double range = _maxValue - _minValue;
        double value = randomValue * range + _minValue;
        
        _currentGenome[i] = value;
    }
}


void EDQDController::setIOcontrollerSize()
{
    // wrt inputs
    
    _nbInputs = 0;
    
    if ( gExtendedSensoryInputs ) // EXTENDED SENSORY INPUTS: code provided as example, can be rewritten to suit your need.
    {
    	// nbOfTypes + ( isItAnAgent? + isItSameGroupId? + agentAngleDifference?) + isItAWall?
//        _nbInputs = ( PhysicalObjectFactory::getNbOfTypes()+3+1 ) * _wm->_cameraSensorsNb;
    	// 20171218(AS): 	- removed object types
		//					- added object groups
    	//					- removed additional agent inputs
    	// nbOfGroups + ( isItAnAgent? ) + isItAWall?
        _nbInputs = ( gNbOfPhysicalObjectGroups+1+1 ) * _wm->_cameraSensorsNb;

    }
    
    // 20180204(AS): this can be taken out, as redundant. Types are determined through sensory inputs.
    _nbInputs += _wm->_cameraSensorsNb + 3; // proximity sensors + ground sensor (3 values)
    if ( gEnergyLevel )
        _nbInputs += 1; // incl. energy level
    if ( gNbOfLandmarks > 0 )
        _nbInputs += 2; // incl. landmark (angle,dist)
    
    // wrt outputs
    
    _nbOutputs = 2;
    
    if ( EDQDSharedData::gEnergyRequestOutput )
        _nbOutputs += 1; // incl. energy request
}

void EDQDController::initController()
{
    _nbHiddenLayers = EDQDSharedData::gNbHiddenLayers;
    _nbNeuronsPerHiddenLayer = new std::vector<unsigned int>(_nbHiddenLayers);
    for(unsigned int i = 0; i < _nbHiddenLayers; i++)
        (*_nbNeuronsPerHiddenLayer)[i] = EDQDSharedData::gNbNeuronsPerHiddenLayer;
    
    createNN();

    unsigned int const nbGene = computeRequiredNumberOfWeights();
    
    if ( gVerbose )
        std::cout << std::flush ;
    
    _currentGenome.clear();
//    if ( _currentGenome.size() < nbGene )
//    {
//    	_currentGenome.reserve(nbGene);
//    }
    
    // Intialize genome
    
    for ( unsigned int i = 0 ; i != nbGene ; i++ )
    {
        _currentGenome.push_back((double)(randint()%EDQDSharedData::gNeuronWeightRange)/(EDQDSharedData::gNeuronWeightRange/2)-1.0); // weights: random init between -1 and +1
//        _currentGenome[i] = ((double)(randint()%EDQDSharedData::gNeuronWeightRange)/(EDQDSharedData::gNeuronWeightRange/2)-1.0); // weights: random init between -1 and +1
    }
    
    setNewGenomeStatus(true);
    
    clearReservoir(); // will contain the genomes received from other robots
}


void EDQDController::clearReservoir()
{
    //std::cout << "[DEBUG] genomesList for agent #" << _wm->getId() << "::" << getBirthdate() << ", at time " << gWorld->getIterations() << "\n";
    //for ( std::map<std::pair<int,int>, std::vector<double> >::iterator it = _genomesList.begin() ; it != _genomesList.end() ; it++ )
    //    std::cout << "[DEBUG] " << (*it).first.first << "::" << (*it).first.second << "\n";
    
    _genomesList.clear(); // empty the list of received genomes
    _sigmaList.clear();
    _fitnessValuesList.clear();

    // always clear _mapList, as results have been merged.
    _mapList.clear();

    if ( EDQDSharedData::gEDQDOnlyKeepMapsForGeneration ||
    		gWorld->getIterations() == 0 )
    {
    	delete _mergedMap;
        _mergedMap = new EDQDMap();
    }
}

void EDQDController::reset()
{
    initController();
}


void EDQDController::mutateSigmaValue()
{
    float dice = float(randint()%100) / 100.0;
    
    if ( dice <= EDQDSharedData::gProbaMutation )
    {
        dice = float(randint() %100) / 100.0;
        if ( dice < 0.5 )
        {
            _currentSigma = _currentSigma * ( 1 + EDQDSharedData::gUpdateSigmaStep ); // increase sigma
            
            if (_currentSigma > EDQDSharedData::gSigmaMax)
            {
                _currentSigma = EDQDSharedData::gSigmaMax;
            }
        }
        else
        {
            _currentSigma = _currentSigma * ( 1 - EDQDSharedData::gUpdateSigmaStep ); // decrease sigma
            
            if ( _currentSigma < EDQDSharedData::gSigmaMin )
            {
                _currentSigma = EDQDSharedData::gSigmaMin;
            }
        }
    }
}


void EDQDController::broadcastGenome()
{
    // remarque \todo: limiting genome transmission is sensitive to sensor order. (but: assume ok)
    
    for( int i = 0 ; i < _wm->_cameraSensorsNb && ( EDQDSharedData::gLimitGenomeTransmission == false || ( EDQDSharedData::gLimitGenomeTransmission == true && _nbGenomeTransmission < EDQDSharedData::gMaxNbGenomeTransmission ) ); i++)
    {
        int targetIndex = _wm->getObjectIdFromCameraSensor(i);
        
        if ( targetIndex >= gRobotIndexStartOffset && targetIndex < gRobotIndexStartOffset+gNbOfRobots )   // sensor ray bumped into a robot : communication is possible
        {
            targetIndex = targetIndex - gRobotIndexStartOffset; // convert image registering index into robot id.
            
            EDQDController* targetRobotController = static_cast<EDQDController*>(gWorld->getRobot(targetIndex)->getController());
            
            if ( ! targetRobotController )
            {
                std::cerr << "Error from robot " << _wm->getId() << " : the observer of robot " << targetIndex << " is not compatible." << std::endl;
                exit(-1);
            }
            
            if ( targetRobotController->isListening() )
            {
            	bool success = false;
            	if (EDQDSharedData::gEDQDMapSelection) {
					success =
							targetRobotController->storeMap(
									_map,
									_wm->getId()
							); // other agent stores a genome from my map
            	} else {
//	        		EDQD::behav_index_t __index;
//	            	if ( _map->hasEntry() )
//	            	{
//						do {
//							__index = _map->getRandomIndex();
//						} while (_map->get(__index)->genome.size() == 0);
//						EDQD::Entry* __entry = _map->get(__index);
//						success =
//								targetRobotController->storeGenome(
//										__entry->genome,
//										__entry->id,
//										__entry->sigma,
//										__entry->fitness,
//										_map
//								); // other agent stores a genome from my map
//	            	} else {
	            		success =
								targetRobotController->storeGenome(
										_currentGenome,
										std::make_pair(_wm->getId(), _birthdate),
										_currentSigma,
										_wm->_fitnessValue,
										_map
								); // other agent stores my genome if map is empty.
//	            	}
            	}

                if ( success == true )
                    _nbGenomeTransmission++; // count unique transmissions (ie. nb of different genomes stored).

                _nbComTx++;
            }
        }
    }
}

void EDQDController::performSelection() // called only if at least 1 genome was stored.
{
	if (EDQDSharedData::gEDQDMapSelection) {
		if (EDQDSharedData::gEDQDIncludeLocalMapForSelectionMerge) {
			_mapList[_wm->getId()] = _map;
		}

		switch ( EDQDSharedData::gSelectionMethod )
		{
			case 0:
				selectRandomGenomeFromRandomMap();
				break;
			case 1:
				selectRandomGenomeFromMergedMap();
				break;
			default:
				std::cerr << "[ERROR] unknown selection method (gSelectionMethod = " << EDQDSharedData::gSelectionMethod << ")\n";
				exit(-1);
		}
	} else {
		switch ( EDQDSharedData::gSelectionMethod )
		{
			case 5:
				selectRandomGenome();
				break;
			case 6:
				selectFirstGenome();
				break;
			case 7:
				selectBestGenome();
				break;
			case 8:
				selectFitProp();
				break;
			default:
				std::cerr << "[ERROR] unknown selection method (gSelectionMethod = " << EDQDSharedData::gSelectionMethod << ")\n";
				exit(-1);
		}

	}

	if (EDQDSharedData::gEDQDMergeAllMapsIntoLocalMap) {
		_mergedMap->mergeInto(*_map);
	}
    
    // Logging: track descendance
    std::string sLog = std::string("");
    sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",descendsFrom," + std::to_string(_ancestor.first) + "::" + std::to_string(_ancestor.second) + "\n";
    gLogManager->write(sLog);
    gLogManager->flush();
}


void EDQDController::loadNewGenome()
{
    if ( _wm->isAlive() || gEnergyRefill )  // ( gEnergyRefill == true ) enables revive
    {
        if ( _wm->isAlive() )
            logCurrentState();
        
        // note: at this point, agent got energy, whether because it was revived or because of remaining energy.
        
        if ( ( EDQDSharedData::gEDQDMapSelection && _mapList.size() > 0 )
        		|| ( !EDQDSharedData::gEDQDMapSelection && _genomesList.size() > 0 ) )
        {
            // case: 1+ genome(s) imported, random pick.
            
            performSelection();
            performVariation();
            clearReservoir();
            
            //logCurrentState();
            
            _wm->setAlive(true);
            
            std::string sLog = std::string("");
            sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",status,active\n";
            gLogManager->write(sLog);
            gLogManager->flush();
            
            if ( _wm->getEnergyLevel() == 0 )
                _wm->setEnergyLevel(gEnergyInit);
            
            _Xinit = _wm->getXReal();
            _Yinit = _wm->getYReal();
            _Xlast = _Xinit;
            _Ylast = _Yinit;
            _dSumTravelled = 0.0;
            _dMaxTravelled = 0.0;
            _nbComTx = 0;
            _nbComRx = 0;
            _currentCellId = 0;
//            _dPotMaxTravelled = getEuclideanDistance(gScreenWidth/2.0, gScreenHeight/2.0, _Xinit, _Yinit) +
//            		EDQDSharedData::gArenaRadius; // crude way to get average radius if ellipse is not a circle
            
            _wm->setRobotLED_colorValues(255, 0, 0);
            
            // log the genome (or at least the existence of a genome)
            if ( _wm->isAlive() )
            {
                // Logging: full genome
                std::string sLog = std::string("");
                sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",genome,";
                
                /*
                 // write genome (takes a lot of disk space)
                 for(unsigned int i=0; i<_genome.size(); i++)
                 {
                 sLog += std::to_string(_genome[i]) + ",";
                 //gLogFile << std::fixed << std::showpoint << _wm->_genome[i] << " ";
                 }
                 */
                sLog += "(...)"; // do not write genome
                
                sLog += "\n";
                gLogManager->write(sLog);
                gLogManager->flush();
            }
        }
        else
        {
            // case: no imported genome and the robot is/was active - robot is set to inactive (which means: robot is put off-line (if gDeathState is true), then wait for new genome (if gListenState is true))
            
            if ( _wm->isAlive() == true )
            {
                
                // Logging: "no genome"
                std::string sLog = std::string("");
                sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",genome,n/a.\n";
                gLogManager->write(sLog);
                gLogManager->flush();
                
                reset(); // destroy then create a new NN
                
                _wm->setAlive(false); // inactive robot *must* import a genome from others (ie. no restart).
                
                if ( EDQDSharedData::gNotListeningStateDelay != 0 ) // ie. -1 (infinite,dead) or >0 (temporary,mute)
                {
                    _isListening = false;

                    _notListeningDelay = EDQDSharedData::gNotListeningStateDelay;
                    _listeningDelay = EDQDSharedData::gListeningStateDelay;
                    _wm->setRobotLED_colorValues(0, 0, 255); // is not listening
                    
                    std::string sLog = std::string("");
                    sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",status,inactive\n";
                    gLogManager->write(sLog);
                    gLogManager->flush();
                    
                }
                else
                {
                    _listeningDelay = EDQDSharedData::gListeningStateDelay;

                    if ( _listeningDelay > 0 || _listeningDelay == -1 )
                    {
                        _isListening = true;
                        
                        _wm->setRobotLED_colorValues(0, 255, 0); // is listening
                        
                        std::string sLog = std::string("");
                        sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",status,listening\n";
                        gLogManager->write(sLog);
                        gLogManager->flush();
                    }
                    else
                    {
                        std::string sLog = std::string("");
                        sLog += "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) + ",status,inactive\n";
                        gLogManager->write(sLog);
                        gLogManager->flush();
                    }
                }
            }
        }
    }
}

void EDQDController::logCurrentState()
{
    // Logging
    std::string sLog = "" + std::to_string(gWorld->getIterations()) + "," + std::to_string(_wm->getId()) + "::" + std::to_string(_birthdate) +
    ",age," + std::to_string(gWorld->getIterations()-_birthdate) +
    ",energy," +  std::to_string(_wm->getEnergyLevel()) +
    ",rxMapListSize," + std::to_string(_mapList.size()) +
    ",rxGenomesListSize," + std::to_string(_genomesList.size()) +
    ",sigma," + std::to_string(_currentSigma) +
    ",x_init," + std::to_string(_wm->getXReal()) +
    ",y_init," + std::to_string(_wm->getYReal()) +
    ",x_current," + std::to_string(_Xinit) +
    ",y_current," + std::to_string(_Yinit) +
    ",dist," + std::to_string( getEuclideanDistance( _Xinit, _Yinit, _wm->getXReal(), _wm->getYReal() ) ) +
    ",maxDist," + std::to_string( _dMaxTravelled ) +
    ",groupId," + std::to_string(_wm->getGroupId()) +
    ",fitnessValue," + std::to_string(_wm->_fitnessValue) + 
    "\n";
    gLogManager->write(sLog);
    gLogManager->flush();
}

void EDQDController::writeEOGEntry(LogManager* lm){
	/*
	 * generation,iteration,robot,birthday,ancestorId,ancestorBirthday,age,energy,sigma,genomes,fitness,cellsInMap,distMax,distPotMax,distTotal,comRx,comTx,g0,g1
    }*/
	std::string sLog = "" + std::to_string(gWorld->getIterations()/EDQDSharedData::gEvaluationTime)
			+ "," + std::to_string(gWorld->getIterations())
			+ "," + std::to_string(_wm->getId())
			+ "," + std::to_string(_birthdate)
			+ "," + std::to_string(_ancestor.first)
			+ "," + std::to_string(_ancestor.second)
			+ "," + std::to_string(_currentCellId)
	    	+ "," + std::to_string(gWorld->getIterations()-_birthdate)
			+ "," + std::to_string(_wm->getEnergyLevel())
	    	+ "," + std::to_string(_currentSigma)
			+ "," + std::to_string(_mapList.size())
			+ "," + std::to_string(_genomesList.size())
	    	+ "," + std::to_string(_wm->_fitnessValue)
	    	+ "," + std::to_string(_map->getNumFilledCells())
			+ "," + std::to_string(_dMaxTravelled)
			+ "," + std::to_string(_dPotMaxTravelled)
	    	+ "," + std::to_string(_dSumTravelled)
	    	+ "," + std::to_string(_nbComRx)
	    	+ "," + std::to_string(_nbComTx);

	for (int i = 0; i < gNbOfPhysicalObjectGroups; i++) {
	    sLog += "," + std::to_string(_objectCounters[i]);
	}

	sLog += "\n";

	lm->write(sLog);
//	    lm->flush();
}

double EDQDController::getFitness()
{
    return _wm->_fitnessValue;
}

/*
 * note: resetFitness is first called by the Controller's constructor.
 */
void EDQDController::resetFitness()
{
    _wm->_fitnessValue = 0;
   resetObjectCounter();
}

void EDQDController::updateFitness()
{
    // nothing to do
//    std::cout << "[WARNING] EDQDController::updateFitness() called -- nothing to do. Forgot to implement?\n";
	int sum = 0;
	for (auto &oc : _objectCounters)
	{
		sum += oc.second;
	}
	_wm->_fitnessValue = sum;
}

//int EDQDController::getNextId()
//{
//    int retValue = _nextId;
//    _nextId++;
//    return retValue;
//}
void EDQDController::resetPotMaxTravelled() {
	_Xinit = _wm->getXReal();
	_Yinit = _wm->getYReal();
	_Xlast = _Xinit;
	_Ylast = _Yinit;
	_dPotMaxTravelled =
			getEuclideanDistance(gScreenWidth/2.0, gScreenHeight/2.0, _Xinit, _Yinit) +
				EDQDSharedData::gArenaRadius; // crude way to get average radius if ellipse is not a circle
}

int EDQDController::updateCellId() {
	EDQD::behav_index_t __index = EDQDMap::computeIndex(_objectCounters, _dMaxTravelled, _dPotMaxTravelled);
	_currentCellId = __index[0] * EDQDSharedData::gEDQDNbOfIntervals  + __index[1];

	return _currentCellId;
}
