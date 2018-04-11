/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "EDQD/include/EDQDWorldObserver.h"
#include "WorldModels/RobotWorldModel.h"
#include "World/World.h"
#include "EDQD/include/EDQDController.h"
#include "EDQD/include/EDQDRobotWorldModel.h"
#include "EDQD/include/EDQDAgentObserver.h"
#include "EDQD/include/EDQDMap.h"
#include "RoboroboMain/roborobo.h"
#include "EDQD/include/EDQDSharedData.h"
#include "Utilities/Graphics.h"
#include "Utilities/Misc.h"
#include "World/PhysicalObject.h"
#include "World/PhysicalObjectGroup.h"


EDQDWorldObserver::EDQDWorldObserver( World* world ) : WorldObserver( world )
{
    _world = world;
    

    // ==== create specific "lite" logger file
    
    std::string litelogFullFilename = gLogDirectoryname + "/lite_" + gLogFilename;
    gLitelogFile.open(litelogFullFilename.c_str());
        
    if(!gLitelogFile) {
        std::cout << "[CRITICAL] Cannot open \"lite\" log file " << litelogFullFilename << "." << std::endl << std::endl;
        exit(-1);
    }
        
    gLitelogManager = new LogManager();
    gLitelogManager->setLogFile(gLitelogFile);
    gLitelogManager->write("# lite logger\n");
    gLitelogManager->write("# generation,iteration,populationSize,minFitness,maxFitness,avgFitness");
    for (int i = 0; i < gNbOfPhysicalObjectGroups; i++ ) {
    	gLitelogManager->write(",t" + std::to_string(i));
	}
    gLitelogManager->flush();


    // ==== create specific "eog" logger file

    std::string eogLogFullFilename = gLogDirectoryname + "/eog_"
			+ gStartTime + "_" + getpidAsReadableString() + ".csv";
    EDQDSharedData::gEOGLogFile.open(eogLogFullFilename.c_str());

	if(!EDQDSharedData::gEOGLogFile) {
		std::cout << "[CRITICAL] Cannot open \"eog\" log file " << eogLogFullFilename << "." << std::endl << std::endl;
		exit(-1);
	}

	EDQDSharedData::gEOGLogManager = new LogManager();
    EDQDSharedData::gEOGLogManager->setLogFile(EDQDSharedData::gEOGLogFile);
    EDQDSharedData::gEOGLogManager->write("generation,iteration,robot,birthday,ancestor_id,ancestor_birthday,cell_id,age,energy,sigma,maps,genomes,fitness,cells_in_map,dist_max,dist_pot_max,dist_total,com_rx,com_tx");
    for (int i = 0; i < gNbOfPhysicalObjectGroups; i++) {
    	EDQDSharedData::gEOGLogManager->write(std::string(",obj_group" + std::to_string(i)));
    }
    EDQDSharedData::gEOGLogManager->write(std::string("\n"));
    EDQDSharedData::gEOGLogManager->flush();


    // ==== create specific "maps" logger file

    std::string mapsLogFullFilename = gLogDirectoryname + "/maps_"
			+ gStartTime + "_" + getpidAsReadableString() + ".csv";
	EDQDSharedData::gMapsLogFile.open(mapsLogFullFilename.c_str());

	if(!EDQDSharedData::gMapsLogFile) {
		std::cout << "[CRITICAL] Cannot open \"maps\" log file " << mapsLogFullFilename << "." << std::endl << std::endl;
		exit(-1);
	}

	EDQDSharedData::gMapsLogManager = new LogManager();
	EDQDSharedData::gMapsLogManager->setLogFile(EDQDSharedData::gMapsLogFile);
	EDQDSharedData::gMapsLogManager->write("generation,map_index");

	std::vector<std::string> _behav_dim_names = {"token_ratio","explore_dist"};
	for (int i = 0; i < EDQDSharedData::gEDQDNbOfDimensions; i++) {
		EDQDSharedData::gMapsLogManager->write(std::string("," + _behav_dim_names[i]));
	}
	EDQDSharedData::gMapsLogManager->write(",min_fitness,max_fitness,mean_fitness,median_fitness,sd,se,var,n,ancestors");
	EDQDSharedData::gMapsLogManager->write(std::string("\n"));
	EDQDSharedData::gMapsLogManager->flush();


    // ==== loading project-specific properties

    gProperties.checkAndGetPropertyValue("gSigmaRef",&EDQDSharedData::gSigmaRef,true);
    gProperties.checkAndGetPropertyValue("gSigmaMin",&EDQDSharedData::gSigmaMin,true);
    gProperties.checkAndGetPropertyValue("gSigmaMax",&EDQDSharedData::gSigmaMax,true);
    
    gProperties.checkAndGetPropertyValue("gProbaMutation",&EDQDSharedData::gProbaMutation,true);
    gProperties.checkAndGetPropertyValue("gUpdateSigmaStep",&EDQDSharedData::gUpdateSigmaStep,true);
    gProperties.checkAndGetPropertyValue("gEvaluationTime",&EDQDSharedData::gEvaluationTime,true);
    gProperties.checkAndGetPropertyValue("gSynchronization",&EDQDSharedData::gSynchronization,true);
    
    gProperties.checkAndGetPropertyValue("gEnergyRequestOutput",&EDQDSharedData::gEnergyRequestOutput,false);
    
    gProperties.checkAndGetPropertyValue("gMonitorPositions",&EDQDSharedData::gMonitorPositions,true);
    
    gProperties.checkAndGetPropertyValue("gNbHiddenLayers",&EDQDSharedData::gNbHiddenLayers,true);
    gProperties.checkAndGetPropertyValue("gNbNeuronsPerHiddenLayer",&EDQDSharedData::gNbNeuronsPerHiddenLayer,true);
    gProperties.checkAndGetPropertyValue("gNeuronWeightRange",&EDQDSharedData::gNeuronWeightRange,true);
    
    gProperties.checkAndGetPropertyValue("gSnapshots",&EDQDSharedData::gSnapshots,false);
    gProperties.checkAndGetPropertyValue("gSnapshotsFrequency",&EDQDSharedData::gSnapshotsFrequency,false);
    
    gProperties.checkAndGetPropertyValue("gControllerType",&EDQDSharedData::gControllerType,true);
    
    gProperties.checkAndGetPropertyValue("gMaxNbGenomeTransmission",&EDQDSharedData::gMaxNbGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gLimitGenomeTransmission",&EDQDSharedData::gLimitGenomeTransmission,true);
    gProperties.checkAndGetPropertyValue("gSelectionMethod",&EDQDSharedData::gSelectionMethod,true);
    
    gProperties.checkAndGetPropertyValue("gNotListeningStateDelay",&EDQDSharedData::gNotListeningStateDelay,true);
    gProperties.checkAndGetPropertyValue("gListeningStateDelay",&EDQDSharedData::gListeningStateDelay,true);
    
    gProperties.checkAndGetPropertyValue("gLogGenome",&EDQDSharedData::gLogGenome,false);
    gProperties.checkAndGetPropertyValue("gLogGenomeSnapshot",&EDQDSharedData::gLogGenomeSnapshot,false);
    
    gProperties.checkAndGetPropertyValue("gIndividualMutationRate",&EDQDSharedData::gIndividualMutationRate,false);

    gProperties.checkAndGetPropertyValue("gMutationOperator",&EDQDSharedData::gMutationOperator,false);
    
    gProperties.checkAndGetPropertyValue("gSigma",&EDQDSharedData::gSigma,false);
    
    // ==== loading EDQD-specific properties
//	gProperties.checkAndGetPropertyValue("gMapElitesNbOfDimensions",&EDQDSharedData::gMapElitesNbOfDimensions,false);
//	gProperties.checkAndGetPropertyValue("gMapElitesNbOfIntervals",&EDQDSharedData::gMapElitesNbOfIntervals,false);
	gProperties.checkAndGetPropertyValue("gEDQDFitEpsilon",&EDQDSharedData::gEDQDFitEpsilon,false);
	gProperties.checkAndGetPropertyValue("gEDQDMapSelection",&EDQDSharedData::gEDQDMapSelection,false);
	gProperties.checkAndGetPropertyValue("gEDQDOnlyKeepMapsForGeneration",&EDQDSharedData::gEDQDOnlyKeepMapsForGeneration,false);
	gProperties.checkAndGetPropertyValue("gEDQDIncludeLocalMapForSelectionMerge",&EDQDSharedData::gEDQDIncludeLocalMapForSelectionMerge,false);
	gProperties.checkAndGetPropertyValue("gEDQDMergeAllMapsIntoLocalMap",&EDQDSharedData::gEDQDMergeAllMapsIntoLocalMap,false);

	// ==== loading project-specific properties
	gProperties.checkAndGetPropertyValue("gPhysicalObjectRepositioning",&EDQDSharedData::gPhysicalObjectRepositioning,false);
	gProperties.checkAndGetPropertyValue("gPhysicalObjectRepositionStartTime",&EDQDSharedData::gPhysicalObjectRepositionStartTime,false);
	gProperties.checkAndGetPropertyValue("gPhysicalObjectRepositionInterval",&EDQDSharedData::gPhysicalObjectRepositionInterval,false);

	std::cout << "[INFO] gPhysicalObjectRepositioning: " << EDQDSharedData::gPhysicalObjectRepositioning << std::endl;
	std::cout << "[INFO] gPhysicalObjectRepositionStartTime: " << EDQDSharedData::gPhysicalObjectRepositionStartTime << std::endl;
	std::cout << "[INFO] gPhysicalObjectRepositionInterval: " << EDQDSharedData::gPhysicalObjectRepositionInterval << std::endl;


	EDQDSharedData::gArenaRadius = ( (gScreenWidth + gScreenHeight) / 4.0 );

    // ====
    
    if ( !gRadioNetwork)
    {
        std::cout << "Error : gRadioNetwork must be true." << std::endl;
        exit(-1);
    }
    
    // * iteration and generation counters
    
    _generationItCount = -1;
    _generationCount = -1;

}

EDQDWorldObserver::~EDQDWorldObserver()
{
    gLitelogFile.close();
    EDQDSharedData::gEOGLogFile.close();
    EDQDSharedData::gMapsLogFile.close();
}

void EDQDWorldObserver::initPre()
{
    // nothing to do.
}

void EDQDWorldObserver::initPost()
{
	int _n = gWorld->getNbOfRobots();
	for (int i = 0 ; i < _n ; i++ )
	{
		Robot* _r = gWorld->getRobot(i);
		( dynamic_cast<EDQDAgentObserver*>( _r->getObserver() ) )->init( _r );
	}
}

void EDQDWorldObserver::stepPre()
{
    _generationItCount++;
    
    if( _generationItCount == EDQDSharedData::gEvaluationTime+1 ) // switch to next generation.
    {
        // update iterations and generations counters
        _generationItCount = 0;
        _generationCount++;
    }
    
    updateMonitoring();
    
    updateEnvironment();
    
}

void EDQDWorldObserver::stepPost()
{
	if( gWorld->getIterations() > 0 &&
//			gWorld->getIterations() % EDQDSharedData::gEvaluationTime == EDQDSharedData::gEvaluationTime-1
			_generationItCount == EDQDSharedData::gEvaluationTime - 1
	)
	{
		bool _update = false;
		for ( int i = 0 ; i != gNbOfRobots ; i++ ) {
			EDQDMap* __rMap = static_cast<EDQDController*>(gWorld->getRobot(i)->getController())->getMap();
			for ( EDQD::map_index_t j = 0; j < EDQDSharedData::gEDQDNbOfIntervals; j++ ) {
				for ( EDQD::map_index_t k = 0; k < EDQDSharedData::gEDQDNbOfIntervals; k++ ) {
//					std::cout << _bestMap(j,k) << " " << (*__rMap)(j,k) << std::endl;
					if ( _bestMap(j,k) < (*__rMap)(j,k) )
					{
						_bestMap(j,k) = (*__rMap)(j,k);
						_update = true;
					}
				}
			}
		}
		if ( _update )
		{
			std::cout << "[INFO] Global best map updated:\n"
					<< _bestMap << std::endl;
			std::string fname = gLogDirectoryname + "/"
					+ "bestmap_"
					+ gStartTime + "_" + getpidAsReadableString();

			EDQD::writeMapToFile(_bestMap.getMap(),
					fname + "_"
						+ boost::lexical_cast<std::string>(gWorld->getIterations() / EDQDSharedData::gEvaluationTime )
						+ std::string(".csv"),
					"plain text bestMap");

			EDQD::writeMapBin(_bestMap,std::string(fname + ".dat"));

//			_bestMap = EDQD::loadMap(std::string(fname + ".dat"));
		}
	}
}

void EDQDWorldObserver::updateEnvironment()
{
    // example: moving landmarks
    /*
    if ( gWorld->getIterations() % 2000 == 0 )
        for ( int i = 0 ; i != gNbOfLandmarks ; i++ )
        {
            Point2d* position = new Point2d( 200+randint()%(gAreaWidth-400) , 200+randint()%(gAreaHeight-400) );
            gLandmarks[i]->setPosition(*position);
        }
    */
	// moving PhysicalObjectGroups
	if ( EDQDSharedData::gPhysicalObjectRepositioning
			&& gWorld->getIterations() > EDQDSharedData::gPhysicalObjectRepositionStartTime
			&& gWorld->getIterations() % EDQDSharedData::gPhysicalObjectRepositionInterval == 0 )
	{
		for ( int i = 0 ; i != gNbOfPhysicalObjectGroups ; i++ ) {
			PhysicalObjectGroup *g = gPhysicalObjectGroups[i];
			double angle = M_PI/6;
			int xOrigin = (int) gAreaWidth/2,
				yOrigin = (int) gAreaHeight/2,
				x = g->getInitAreaX(),
				y = g->getInitAreaY();

			/* https://stackoverflow.com/a/32161713 */
			int _x, _y;
			_x = ((x - xOrigin) * cos(angle)) - ((y - yOrigin) * sin(angle)) + xOrigin;
			_y = ((x - xOrigin) * sin(angle)) + ((y - yOrigin) * cos(angle)) + yOrigin;

			g->setInitAreaX(_x);
			g->setInitAreaY(_y);

			if ( gVerbose )
				std::cout << "[ INFO: new position for group " << g->getId() << " is ( " << _x << ", " << _y << ") ]" << std::endl;
		}

		for (std::vector<PhysicalObjectGroup*>::iterator itG = gPhysicalObjectGroups.begin(); itG < gPhysicalObjectGroups.end() ; itG++ ) {
			std::vector<PhysicalObject*> _g= (*itG)->getGroupObjects();
			for ( std::vector<PhysicalObject*>::iterator itO = _g.begin(); itO < _g.end() ; itO++  ) {
				(*itO)->relocateObject();
			}
		}
	}
}

void EDQDWorldObserver::updateMonitoring()
{
    // * Log at end of each generation

    //if( gWorld->getIterations() % EDQDSharedData::gEvaluationTime == 1 || gWorld->getIterations() % EDQDSharedData::gEvaluationTime == EDQDSharedData::gEvaluationTime-1 ) // beginning(+1) *and* end of generation. ("==1" is required to monitor the outcome of the first iteration)
    // log at end of generation.
    if( gWorld->getIterations() % EDQDSharedData::gEvaluationTime == EDQDSharedData::gEvaluationTime-1 )
    {
        monitorPopulation();
    }
    
    // * Every N generations, take a video (duration: one generation time)
    
    if ( EDQDSharedData::gSnapshots )
    {
        if ( ( gWorld->getIterations() ) % ( EDQDSharedData::gEvaluationTime * EDQDSharedData::gSnapshotsFrequency ) == 0 )
        {
            if ( gVerbose )
                std::cout << "[START] Video recording: generation #" << (gWorld->getIterations() / EDQDSharedData::gEvaluationTime ) << ".\n";
            gTrajectoryMonitorMode = 0;
            initTrajectoriesMonitor();
        }
        else
            if ( ( gWorld->getIterations() ) % ( EDQDSharedData::gEvaluationTime * EDQDSharedData::gSnapshotsFrequency ) == EDQDSharedData::gEvaluationTime - 1 )
            {
                if ( gVerbose )
                    std::cout << "[STOP]  Video recording: generation #" << (gWorld->getIterations() / EDQDSharedData::gEvaluationTime ) << ".\n";
                saveTrajectoryImage();
            }
    }    
}

void EDQDWorldObserver::monitorPopulation( bool localVerbose )
{
    // * monitoring: count number of active agents.
    int generation = gWorld->getIterations() / EDQDSharedData::gEvaluationTime;
    
    int activeCount = 0;
    double sumOfFitnesses = 0;
    double minFitness = DBL_MAX;
    double maxFitness = -DBL_MAX;

    int sumOfToken = 0;
    std::map<int,int> tokens;
//    double tokenRatio = 0.0;

    std::vector<EDQDMap> maps(gNbOfRobots);

    for ( auto &og : gPhysicalObjectGroups )
	{
		tokens[og->getId()] = 0;
	}

    
    for ( int i = 0 ; i != gNbOfRobots ; i++ ) {
        EDQDController *ctl = static_cast<EDQDController*>(gWorld->getRobot(i)->getController());
        
        if ( ctl->getWorldModel()->isAlive() == true ) {
            activeCount++;
            sumOfFitnesses += ctl->getFitness() ;
            if ( ctl->getFitness() < minFitness )
                minFitness = ctl->getFitness();
            if ( ctl->getFitness() > maxFitness )
                maxFitness = ctl->getFitness();
        }

        for ( auto &oc : ctl->getObjectCounters() ) {
        	tokens[oc.first] += oc.second;
        	sumOfToken += oc.second;
        }

		ctl->updateCellId();

        ctl->writeEOGEntry(EDQDSharedData::gEOGLogManager);

        maps.push_back( *(ctl->getMap()) );

//        std::string __map =
//        		EDQD::mapToString(
//        				ctl->getMap()->getMap(),
//						std::string(
//								std::to_string(generation) + "," +
//								std::to_string(ctl->getWorldModel()->getId()) + ","
//								));
//		EDQDSharedData::gMapsLogManager->write(__map);
    }
    EDQDSharedData::gEOGLogManager->flush();
//    EDQDSharedData::gMapsLogManager->flush();
    

    if (maps.size() > 0) {
		std::vector<double> fit;
		std::map<std::pair<int,int>,int> ancestors;
		double f = 0.0;
		int sum = 0;
		int min_fit = 0;
		int max_fit = 0;
		double mean_fit = 0;
		int median_fit = 0;
		double se = 0.0;
		double sd = 0.0;
		double var = 0.0;
		int N = 0;

		EDQD::behav_index_t _index;

		size_t behav_dim = EDQDSharedData::gEDQDNbOfDimensions;
	//	size_t behav_interval = EDQDSharedData::gEDQDNbOfIntervals;
		size_t num_elements = maps.begin()->getMap().num_elements();
	//	EDQDMap* last;

		for(size_t c = 0; c < num_elements; c++ ) {
			// initialise
			sum = 0;
			fit.clear();
			ancestors.clear();
			f = 0.0;
			min_fit = 0;
			max_fit = 0;
			mean_fit = 0;
			median_fit = 0;
			se = 0.0;
			sd = 0.0;
			var = 0.0;
			N = 0;


			// iterate through maps
			for (std::vector<EDQDMap>::iterator m = maps.begin(); m != maps.end(); m++ ) {
				f = m->getMap().data()[c].fitness;
				if (f >= 0) {
					N++;
					fit.push_back(f);
					if (f < min_fit) min_fit = f;
					if (f > max_fit) max_fit = f;
	//				last = &(*m);
					//will add new entry to map if ancestor is not yet in
					// => size of ancestors is used as approximation for number of unique genomes
					ancestors[m->getMap().data()[c].id];
				}
			}

			// calculate values
			if (N > 0) {
				size_t n = fit.size() / 2;
				nth_element(fit.begin(), fit.begin()+n, fit.end());
				if (fit.size() % 2 == 0) {
					nth_element(fit.begin(), fit.begin()+n-1, fit.end());
					median_fit = (fit[n-1] + fit[n]) / 2;
				} else {
					median_fit = fit[n];
				}
				mean_fit = sum / (double)N;
				for( int i = 0; i < N; i++ ) {
				  var += (fit[i] - mean_fit) * (fit[i] - mean_fit);
				}
				var /= N;
				sd = sqrt(var);
				se = sd / sqrt(N);
			}

			// get index values
			for (size_t dim = 0; dim < behav_dim; dim++ ) {
			  _index[dim] = (c / maps.begin()->getMap().strides()[dim] % maps.begin()->getMap().shape()[dim] +  maps.begin()->getMap().index_bases()[dim]);
			}

			// output
			std::string ofs = std::to_string(generation) + ","
					+ std::to_string(c) + ",";
			for (size_t dim = 0; dim < behav_dim; ++dim) {
				ofs += std::to_string(_index[dim] / (float)maps.begin()->getMap().shape()[dim]) + ",";
			}
			ofs += std::to_string(min_fit) + "," + std::to_string(max_fit) + ","
					+ std::to_string(mean_fit) + "," + std::to_string(median_fit) + ","
					+ std::to_string(sd) + "," + std::to_string(se) + ","
					+ std::to_string(var) + "," + std::to_string(N);
			ofs += "," + std::to_string(ancestors.size());
			ofs += "\n";

			EDQDSharedData::gMapsLogManager->write(std::string(ofs));
			ofs.clear();
			ofs = "";
		}
		EDQDSharedData::gMapsLogManager->flush();


    }

    if ( gVerbose && localVerbose ) {
        std::cout << "[ gen:" << (gWorld->getIterations()/EDQDSharedData::gEvaluationTime)
					<< "\tit:" << gWorld->getIterations()
					<< "\tpop:" << activeCount
					<< "\tminFitness:" << minFitness
					<< "\tmaxFitness:" << maxFitness
					<< "\tavgFitness:" << sumOfFitnesses/activeCount;

        for (auto &t : tokens ) {
        	std::cout << "\tt" << t.first << ": " << t.second << "(" << (double)t.second / sumOfToken << ")";
        }

        std::cout << " ]\n";
    }
    
    // display lightweight logs for easy-parsing
    std::string sLitelog =
        "log:"
        + std::to_string(gWorld->getIterations()/EDQDSharedData::gEvaluationTime)
        + ","
        + std::to_string(gWorld->getIterations())
        + ","
        + std::to_string(activeCount)
        + ","
        + std::to_string(minFitness)
        + ","
        + std::to_string(maxFitness)
        + ","
        + std::to_string(sumOfFitnesses/activeCount);
    for (auto &t : tokens ) {
		sLitelog += "," + t.second;
	}
    gLitelogManager->write(sLitelog);
    gLitelogManager->flush();  // flush internal buffer to file
    gLitelogFile << std::endl; // flush file output (+ "\n")
    
    // Logging, population-level: alive
    std::string sLog = std::string("") + std::to_string(gWorld->getIterations()) + ",pop,alive," + std::to_string(activeCount) + "\n";
    gLogManager->write(sLog);
    gLogManager->flush();
}
