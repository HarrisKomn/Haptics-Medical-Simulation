/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sstream>
using std::ostringstream ;
#include <fstream>

#include <string>
using std::string;

#include <vector>
using std::vector;


#include <sofa/helper/ArgumentParser.h>
#include <SofaSimulationCommon/common.h>
#include <sofa/simulation/Node.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/simulation/config.h> // #defines SOFA_HAVE_DAG (or not)
#include <SofaSimulationCommon/init.h>
#ifdef SOFA_HAVE_DAG
#include <SofaSimulationGraph/init.h>
#include <SofaSimulationGraph/DAGSimulation.h>
#endif
#include <SofaSimulationTree/init.h>
#include <SofaSimulationTree/TreeSimulation.h>
using sofa::simulation::Node;

#include <SofaComponentCommon/initComponentCommon.h>
#include <SofaComponentBase/initComponentBase.h>
#include <SofaComponentGeneral/initComponentGeneral.h>
#include <SofaComponentAdvanced/initComponentAdvanced.h>
#include <SofaComponentMisc/initComponentMisc.h>

#include <SofaGeneralLoader/ReadState.h>
#include <SofaValidation/CompareState.h>
#include <sofa/helper/Factory.h>
#include <sofa/helper/cast.h>
#include <sofa/helper/BackTrace.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/Utils.h>
#include <sofa/gui/GUIManager.h>
using sofa::gui::GUIManager;

#include <sofa/gui/Main.h>
#include <sofa/gui/BatchGUI.h>  // For the default number of iterations
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/atomic.h>

using sofa::core::ExecParams ;

#include <sofa/helper/system/console.h>
using sofa::helper::Utils;

using sofa::component::misc::CompareStateCreator;
using sofa::component::misc::ReadStateActivator;
using sofa::simulation::tree::TreeSimulation;
using sofa::simulation::graph::DAGSimulation;
using sofa::helper::system::SetDirectory;
using sofa::core::objectmodel::BaseNode ;
using sofa::gui::BatchGUI;
using sofa::gui::BaseGUI;

#include <sofa/helper/logging/Messaging.h>

#include <sofa/helper/logging/ConsoleMessageHandler.h>
using sofa::helper::logging::ConsoleMessageHandler ;

#include <sofa/core/logging/RichConsoleStyleMessageFormatter.h>
using  sofa::helper::logging::RichConsoleStyleMessageFormatter ;

#include <sofa/core/logging/PerComponentLoggingMessageHandler.h>
using  sofa::helper::logging::MainPerComponentLoggingMessageHandler ;

#ifdef WIN32
#include <windows.h>
#endif

#include <sofa/gui/GuiDataRepository.h>
using sofa::gui::GuiDataRepository ;

using sofa::helper::system::DataRepository;
using sofa::helper::system::PluginRepository;
using sofa::helper::system::PluginManager;

#include <sofa/helper/logging/MessageDispatcher.h>
using sofa::helper::logging::MessageDispatcher ;

#include <sofa/helper/logging/ClangMessageHandler.h>
using sofa::helper::logging::ClangMessageHandler ;

#include <sofa/helper/logging/ExceptionMessageHandler.h>
using sofa::helper::logging::ExceptionMessageHandler;

#include <sofa/defaulttype/Vec3Types.h>

using sofa::defaulttype::Vec3Types;
using sofa::defaulttype::Vector3;


//typedef typename DataTypes::Real          Real;
typedef sofa::defaulttype::Vec<3, double>   Coord3;
typedef sofa::helper::vector <Coord3>    VecCoord3;
typedef sofa::helper::vector <sofa::defaulttype::RigidDeriv<3, double>> VecCoord3Rigid;
typedef sofa::defaulttype::RigidCoord<3, double> Coord;
typedef sofa::helper::vector<sofa::defaulttype::Vec3d> VCoord3;
typedef sofa::defaulttype::Vec3Types::Deriv Deriv3;
typedef sofa::defaulttype::Vec<6, double> Vec6;
typedef sofa::defaulttype::Vec<15, double> Vec30;



#include <SofaOpenglVisual/OglModel.h>
using sofa::component::visualmodel::OglModel;

#include <SofaBaseVisual/VisualStyle.h>

#include <SofaBaseCollision/SphereModel.h>

#include <SofaBaseCollision/DefaultPipeline.h>

#include <SofaBaseCollision/BruteForceDetection.h>

#include <SofaBaseCollision/MinProximityIntersection.h>
#include <SofaConstraint/LocalMinDistance.h>


#include <SofaBaseCollision/DefaultContactManager.h>

#include <SofaImplicitOdeSolver/EulerImplicitSolver.h>
using sofa::component::odesolver::EulerImplicitSolver;

#include <SofaBaseLinearSolver/CGLinearSolver.h>
typedef sofa::component::linearsolver::CGLinearSolver < sofa::component::linearsolver::GraphScatteredMatrix,sofa::component::linearsolver::GraphScatteredVector>   CGLinearSolver3;

#include <SofaGeneralLoader/MeshGmshLoader.h>

//#include <SofaBaseTopology/MeshTopology.h>
//#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>


#include <SofaBaseMechanics/MechanicalObject.h>
typedef sofa::component::container::MechanicalObject<Vec3Types> MechanicalObject3;
typedef sofa::component::container::MechanicalObject < sofa::defaulttype::StdRigidTypes <3, double> > MechanicalObjectRigid3;


#include <SofaBaseMechanics/UniformMass.h>
typedef sofa::component::mass::UniformMass<sofa::defaulttype::Vec3Types, SReal> UniformMass3;
typedef sofa::component::mass::UniformMass<sofa::defaulttype::Rigid3Types, sofa::defaulttype::Rigid3Mass> UniformMassRigid3;

//#include <SofaSimpleFem/TetrahedronFEMForceField.h>
//typedef sofa::component::forcefield::TetrahedronFEMForceField<Vec3Types>          TetrahedronFEMForceField3;
#include <SofaGeneralSimpleFEM/TetrahedralCorotationalFEMForceField.h>

#include <SofaEngine/BoxROI.h>
#include <SofaBoundaryCondition/FixedConstraint.h>
typedef sofa::component::projectiveconstraintset::FixedConstraint<sofa::defaulttype::Vec3Types> FixedConstraint3;
//sofa::defaulttype::
//typedef StdVectorTypes<sofa::defaulttype::Vec6d, sofa::defaulttype::Vec6d, double> Vec6dTypes;

#include <SofaBaseMechanics/BarycentricMapping.h>
typedef sofa::component::mapping::BarycentricMapping<sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes> BarycentricMapping_Vec3d_to_ExteVec3f;

#include <SofaGeneralLoader/SphereLoader.h>

#include <plugins/SofaCarving/CarvingManager.h>
#include <plugins/Geomagic/src/GeomagicDriver.h>


#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetGeometryAlgorithms.h>
#include <SofaBaseTopology/TetrahedronSetTopologyModifier.h>
#include <SofaBaseTopology/TetrahedronSetTopologyAlgorithms.h>

#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyModifier.h>
#include <SofaBaseTopology/TriangleSetGeometryAlgorithms.h>
#include <SofaBaseTopology/TriangleSetTopologyAlgorithms.h>
#include <SofaTopologyMapping/Tetra2TriangleTopologicalMapping.h>

#include <SofaMeshCollision/TriangleModel.h>
typedef sofa::component::collision::TTriangleModel<sofa::defaulttype::Vec3dTypes> TTriangle_3;


#include <SofaMeshCollision/LineModel.h>
typedef sofa::component::collision::TLineModel<sofa::defaulttype::Vec3dTypes> TLine_3;


#include <SofaMeshCollision/PointModel.h>
typedef sofa::component::collision::TPointModel<sofa::defaulttype::Vec3dTypes> TPoint_3;

#include <sofa/core/CollisionModel.h>


typedef sofa::core::CollisionModel ToolModel;
typedef sofa::core::CollisionModel SurfaceModel;

//#include <SofaBaseMechanics/IdentityMapping.h>
#include <SofaConstraint/LCPConstraintSolver.h>
#include <SofaConstraint/PrecomputedConstraintCorrection.h>

#include <SofaUserInteraction/MechanicalStateController.h>

typedef sofa::component::mapping::RigidMapping<sofa::defaulttype::StdRigidTypes <3, double>, sofa::defaulttype::ExtVec3fTypes> RigidMapping_Rigid_to_Extevec3F;
typedef sofa::component::mapping::RigidMapping<sofa::defaulttype::StdRigidTypes <3, double>, sofa::defaulttype::Vec3dTypes> RigidMapping_Rigid_to_Vec3d;
//typedef sofa::component::mapping::RigidMapping<sofa::defaulttype::, sofa::defaulttype::ExtVec3fTypes> RigidMapping_Rigidnew_to_Extevec3F;
#include <SofaLoader/MeshObjLoader.h>
#include <SofaBaseTopology/MeshTopology.h>

#include <SofaHaptics/LCPForceFeedback.h>
#include <SofaConstraint/UncoupledConstraintCorrection.h>


//#include <SofaMeshCollision/LineModel.h>
//typedef sofa::component::collision::TLineModel<sofa::defaulttype::Vec3dTypes> TLine_3;

#include <SofaGeneralDeformable/VectorSpringForceField.h>
#include <SofaConstraint/FreeMotionAnimationLoop.h>
#include <SofaBaseMechanics/DiagonalMass.h>

#include <SceneCreator/SceneCreator.h>

#include <plugins\SofaMiscCollision\DefaultCollisionGroupManager.h>
#include <SofaBoundaryCondition\LinearMovementConstraint.h>




#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)


void loadVerificationData(string& directory, string& filename, Node* node)
{
    msg_info("") << "loadVerificationData from " << directory << " and file " << filename ;

    string refFile;

    refFile += directory;
    refFile += '/';
    refFile += SetDirectory::GetFileName(filename.c_str());

    msg_info("") << "loadVerificationData " << refFile ;

    CompareStateCreator compareVisitor(ExecParams::defaultInstance());
    compareVisitor.setCreateInMapping(true);
    compareVisitor.setSceneName(refFile);
    compareVisitor.execute(node);

    ReadStateActivator v_read(ExecParams::defaultInstance(), true);
    v_read.execute(node);
}

void addGUIParameters(ArgumentParser* argumentParser)
{
    GUIManager::RegisterParameters(argumentParser);
}

// ---------------------------------------------------------------------
// ---
// ---------------------------------------------------------------------
int main(int argc, char** argv)
{
    // Add resources dir to GuiDataRepository
    const std::string sofaIniFilePath = Utils::getSofaPathPrefix() + "/etc/runSofa.ini";
    std::map<std::string, std::string> iniFileValues = Utils::readBasicIniFile(sofaIniFilePath);
    if (iniFileValues.find("RESOURCES_DIR") != iniFileValues.end())
    {
        GuiDataRepository.addFirstPath( iniFileValues["RESOURCES_DIR"] );
    }

    sofa::helper::BackTrace::autodump();

    ExecParams::defaultInstance()->setAspectID(0);

#ifdef WIN32
    {
        HANDLE hStdout = GetStdHandle(STD_OUTPUT_HANDLE);
        COORD s;
        s.X = 160; s.Y = 10000;
        SetConsoleScreenBufferSize(hStdout, s);
        CONSOLE_SCREEN_BUFFER_INFO csbi;
        if (GetConsoleScreenBufferInfo(hStdout, &csbi))
        {
            SMALL_RECT winfo;
            winfo = csbi.srWindow;
            //winfo.Top = 0;
            winfo.Left = 0;
            //winfo.Bottom = csbi.dwSize.Y-1;
            winfo.Right = csbi.dwMaximumWindowSize.X-1;
            SetConsoleWindowInfo(hStdout, TRUE, &winfo);
        }

    }
#endif

    sofa::gui::initMain();

	// change scene
	bool scene = false;
	bool sceneHaptics = false;
	bool sceneNOHaptics = true;
    string fileName ;
	string fileName2;
    bool        startAnim = false;
    bool        showHelp = false;
    bool        printFactory = false;
    bool        loadRecent = false;
    bool        temporaryFile = false;
    bool        testMode = false;
    bool        noAutoloadPlugins = false;
    unsigned int nbMSSASamples = 1;
    bool computationTimeAtBegin = false;
    unsigned int computationTimeSampling=0; ///< Frequency of display of the computation time statistics, in number of animation steps. 0 means never.
    string    computationTimeOutputType="stdout";

    string gui = "";
    string verif = "";
	
#if defined(SOFA_HAVE_DAG)
    string simulationType = "dag";
#else
    string simulationType = "tree";
#endif

    vector<string> plugins;

	//ADD PLUGINS: SofaCarving , Geomagic

	plugins.push_back("SofaCarving");
	plugins.push_back("Geomagic");
	plugins.push_back("CImgPlugin");
	plugins.push_back("SofaMiscCollision");

	vector<string> files;
#ifdef SOFA_SMP
    string nProcs="";
    bool        disableStealing = false;
    bool        affinity = false;
#endif
    string colorsStatus = "unset";
    string messageHandler = "auto";
    bool enableInteraction = false ;
    int width = 800;
    int height = 600;

    string gui_help = "choose the UI (";
    gui_help += GUIManager::ListSupportedGUI('|');
    gui_help += ")";

    ArgumentParser* argParser = new ArgumentParser(argc, argv);
    argParser->addArgument(po::value<bool>(&showHelp)->default_value(false)->implicit_value(true),                  "help,h", "Display this help message");
    argParser->addArgument(po::value<bool>(&startAnim)->default_value(false)->implicit_value(true),                 "start,a", "start the animation loop");
    argParser->addArgument(po::value<bool>(&computationTimeAtBegin)->default_value(false)->implicit_value(true),    "computationTimeAtBegin,b", "Output computation time statistics of the init (at the begin of the simulation)");
    argParser->addArgument(po::value<unsigned int>(&computationTimeSampling)->default_value(0),                     "computationTimeSampling", "Frequency of display of the computation time statistics, in number of animation steps. 0 means never.");
    argParser->addArgument(po::value<std::string>(&computationTimeOutputType)->default_value("stdout"),             "computationTimeOutputType,o", "Output type for the computation time statistics: either stdout, json or ljson");
    argParser->addArgument(po::value<std::string>(&gui)->default_value(""),                                         "gui,g", gui_help.c_str());
    argParser->addArgument(po::value<std::vector<std::string>>(&plugins),                                           "load,l", "load given plugins");
    argParser->addArgument(po::value<bool>(&noAutoloadPlugins)->default_value(false)->implicit_value(true),         "noautoload", "disable plugins autoloading");

    // example of an option using lambda function which ensure the value passed is > 0
    argParser->addArgument(po::value<unsigned int>(&nbMSSASamples)->default_value(1)->notifier([](unsigned int value)
    {
        if (value < 1) {
            std::cerr << "msaa sample cannot be lower than 1" << std::endl;
            exit( EXIT_FAILURE );
        }
    }),                                                                                                             "msaa,m", "number of samples for MSAA (Multi Sampling Anti Aliasing ; value < 2 means disabled");

    argParser->addArgument(po::value<bool>(&printFactory)->default_value(false)->implicit_value(true),              "factory,p", "print factory logs");
    argParser->addArgument(po::value<bool>(&loadRecent)->default_value(false)->implicit_value(true),                "recent,r", "load most recently opened file");
    argParser->addArgument(po::value<std::string>(&simulationType),                                                 "simu,s", "select the type of simulation (bgl, dag, tree)");
    argParser->addArgument(po::value<bool>(&temporaryFile)->default_value(false)->implicit_value(true),             "tmp", "the loaded scene won't appear in history of opened files");
    argParser->addArgument(po::value<bool>(&testMode)->default_value(false)->implicit_value(true),                  "test", "select test mode with xml output after N iteration");
    argParser->addArgument(po::value<std::string>(&verif)->default_value(""), "verification,v",                     "load verification data for the scene");
    argParser->addArgument(po::value<std::string>(&colorsStatus)->default_value("unset", "auto")->implicit_value("yes"),     "colors,c", "use colors on stdout and stderr (yes, no, auto)");
    argParser->addArgument(po::value<std::string>(&messageHandler)->default_value("auto"), "formatting,f",          "select the message formatting to use (auto, clang, sofa, rich, test)");
    argParser->addArgument(po::value<bool>(&enableInteraction)->default_value(false)->implicit_value(true),         "interactive,i", "enable interactive mode for the GUI which includes idle and mouse events (EXPERIMENTAL)");
    argParser->addArgument(po::value<std::vector<std::string> >()->multitoken(), "argv",                            "forward extra args to the python interpreter");

#ifdef SOFA_SMP
    argParser->addArgument(po::value<bool>(&disableStealing)->default_value(false)->implicit_value(true),           "disableStealing,w", "Disable Work Stealing")
    argParser->addArgument(po::value<std::string>(&nProcs)->default_value(""),                                      "nprocs", "Number of processor")
    argParser->addArgument(po::value<bool>(&affinity)->default_value(false)->implicit_value(true),                  "affinity", "Enable aFfinity base Work Stealing")
#endif

    addGUIParameters(argParser);
    argParser->parse();
    files = argParser->getInputFileList();

    if(showHelp)
    {
        argParser->showHelp();
        exit( EXIT_SUCCESS );
    }

    // Note that initializations must be done after ArgumentParser that can exit the application (without cleanup)
    // even if everything is ok e.g. asking for help
    sofa::simulation::tree::init();
#ifdef SOFA_HAVE_DAG
    sofa::simulation::graph::init();
#endif
    sofa::component::initComponentBase();
    sofa::component::initComponentCommon();
    sofa::component::initComponentGeneral();
    sofa::component::initComponentAdvanced();
    sofa::component::initComponentMisc();

#ifdef SOFA_HAVE_DAG
    if (simulationType == "tree")
        sofa::simulation::setSimulation(new TreeSimulation());
    else
        sofa::simulation::setSimulation(new DAGSimulation());
#else //SOFA_HAVE_DAG
    sofa::simulation::setSimulation(new TreeSimulation());
#endif

    if (colorsStatus == "unset") {
        // If the parameter is unset, check the environment variable
        const char * colorStatusEnvironment = std::getenv("SOFA_COLOR_TERMINAL");
        if (colorStatusEnvironment != nullptr) {
            const std::string status (colorStatusEnvironment);
            if (status == "yes" || status == "on" || status == "always")
                sofa::helper::console::setStatus(sofa::helper::console::Status::On);
            else if (status == "no" || status == "off" || status == "never")
                sofa::helper::console::setStatus(sofa::helper::console::Status::Off);
            else
                sofa::helper::console::setStatus(sofa::helper::console::Status::Auto);
        }
    } else if (colorsStatus == "auto")
        sofa::helper::console::setStatus(sofa::helper::console::Status::Auto);
    else if (colorsStatus == "yes")
        sofa::helper::console::setStatus(sofa::helper::console::Status::On);
    else if (colorsStatus == "no")
        sofa::helper::console::setStatus(sofa::helper::console::Status::Off);

    //TODO(dmarchal): Use smart pointer there to avoid memory leaks !!
    if (messageHandler == "auto" )
    {
        MessageDispatcher::clearHandlers() ;
        MessageDispatcher::addHandler( new ConsoleMessageHandler() ) ;
    }
    else if (messageHandler == "clang")
    {
        MessageDispatcher::clearHandlers() ;
        MessageDispatcher::addHandler( new ClangMessageHandler() ) ;
    }
    else if (messageHandler == "sofa")
    {
        MessageDispatcher::clearHandlers() ;
        MessageDispatcher::addHandler( new ConsoleMessageHandler() ) ;
    }
    else if (messageHandler == "rich")
    {
        MessageDispatcher::clearHandlers() ;
        MessageDispatcher::addHandler( new ConsoleMessageHandler(&RichConsoleStyleMessageFormatter::getInstance()) ) ;
    }
    else if (messageHandler == "test"){
        MessageDispatcher::addHandler( new ExceptionMessageHandler() ) ;
    }
    else{
        msg_warning("") << "Invalid argument '" << messageHandler << "' for '--formatting'";
    }
    MessageDispatcher::addHandler(&MainPerComponentLoggingMessageHandler::getInstance()) ;


    // Add the plugin directory to PluginRepository
    const std::string& pluginDir = Utils::getPluginDirectory();
    PluginRepository.addFirstPath(pluginDir);

    // Initialise paths
    BaseGUI::setConfigDirectoryPath(Utils::getSofaPathPrefix() + "/config", true);
    BaseGUI::setScreenshotDirectoryPath(Utils::getSofaPathPrefix() + "/screenshots", true);

    if (!files.empty())
        fileName = files[0];
	//sofa::helper::system::PluginRepository.addFirstPath(QCoreApplication::applicationDirPath().toStdString() + "/../lib"); // Your plugin compiled with your application


    for (unsigned int i=0; i<plugins.size(); i++)
        PluginManager::getInstance().loadPlugin(plugins[i]);

    std::string configPluginPath = pluginDir + "/" + TOSTRING(CONFIG_PLUGIN_FILENAME);
    std::string defaultConfigPluginPath = pluginDir + "/" + TOSTRING(DEFAULT_CONFIG_PLUGIN_FILENAME);
	
    if (!noAutoloadPlugins)
    {
        if (PluginRepository.findFile(configPluginPath))
        {
            msg_info("runSofa") << "Loading automatically plugin list in " << configPluginPath;
            PluginManager::getInstance().readFromIniFile(configPluginPath);
        }
        else if (PluginRepository.findFile(defaultConfigPluginPath))
        {
            msg_info("runSofa") << "Loading automatically plugin list in " << defaultConfigPluginPath;
            PluginManager::getInstance().readFromIniFile(defaultConfigPluginPath);
        }
        else
            msg_info("runSofa") << "No plugin list found. No plugin will be automatically loaded.";
    }
    else
        msg_info("runSofa") << "Automatic plugin loading disabled.";

    PluginManager::getInstance().init();

    if (int err = GUIManager::Init(argv[0],gui.c_str()))
        return err;

    if (fileName.empty())
    {
        if (loadRecent) // try to reload the latest scene
        {
            string scenes = BaseGUI::getConfigDirectoryPath() + "/runSofa.ini";
            std::ifstream mrulist(scenes.c_str());
            std::getline(mrulist,fileName);
            mrulist.close();
        }
		else
		{
			fileName = "Demos/MyGeomagic.scn";
			fileName2 = "Demos/OmniInstr.scn";
		}

        fileName = DataRepository.getFile(fileName);
		fileName2= DataRepository.getFile(fileName2);
    }


    if (int err=GUIManager::createGUI(NULL))
        return err;

    //To set a specific resolution for the viewer, use the component ViewerSetting in you scene graph

	//Before first change , change the default scene

	/*________________________________________________Diplwmatiki_________________________________________________________________________________________________*/
	/*____________________________________________________________________________________________________________________________________________________________*/
	/*____________________________________________________________________________________________________________________________________________________________*/



	if (sceneHaptics) {

		GUIManager::SetDimension(width, height);

		Node::SPtr groot = sofa::simulation::getSimulation()->createNewGraph("Root");
		groot->setGravity(sofa::defaulttype::Vector3(0, 0, 0));
		groot->setDt(0.05);

		sofa::component::visualmodel::VisualStyle::SPtr style = sofa::core::objectmodel::New<sofa::component::visualmodel::VisualStyle>();
		sofa::core::visual::DisplayFlags& flags = *style->displayFlags.beginEdit();
		flags.setShowCollisionModels(true);
		flags.setShowVisualModels(false);
		style->displayFlags.endEdit();
		groot->addObject(style);

		sofa::component::collision::DefaultPipeline::SPtr pipeline = sofa::core::objectmodel::New<sofa::component::collision::DefaultPipeline>();
		pipeline->setName("CollisionPipeline");
		pipeline->d_depth.setValue(6);
		groot->addObject(pipeline);

		sofa::component::collision::BruteForceDetection::SPtr bruteforce = sofa::core::objectmodel::New<sofa::component::collision::BruteForceDetection>();
		bruteforce->setName("N2");
		groot->addObject(bruteforce);

		//CollisionResponse
		sofa::component::collision::DefaultContactManager::SPtr response = sofa::core::objectmodel::New<sofa::component::collision::DefaultContactManager>();
		response->setName("Collision Response");
		response->setDefaultResponseType("FrictionContact");
		groot->addObject(response);

		//LocalMinDistance
		sofa::component::collision::LocalMinDistance::SPtr localMinDist = sofa::core::objectmodel::New<sofa::component::collision::LocalMinDistance>();
		localMinDist->setName("Proximity");
		localMinDist->alarmDistance.setValue(1.5);
		localMinDist->contactDistance.setValue(0.05);
		groot->addObject(localMinDist);

		//FreeMotionAnimationLoop
		sofa::component::animationloop::FreeMotionAnimationLoop::SPtr freemot = sofa::core::objectmodel::New<sofa::component::animationloop::FreeMotionAnimationLoop>(groot.get());
		freemot->setName("FreeMotionLoop");
		groot->addObject(freemot);

		//LCPConstraintSolver
		sofa::component::constraintset::LCPConstraintSolver::SPtr lcpSolver = sofa::core::objectmodel::New<sofa::component::constraintset::LCPConstraintSolver>();
		lcpSolver->tol.setValue(0.001);
		lcpSolver->maxIt.setValue(1000);
		groot->addObject(lcpSolver);

		
		/*sofa::core::collision::CollisionGroupManager::SPtr colMan= sofa::core::objectmodel::New<sofa::core::collision::CollisionGroupManager>(colMan);
		colMan->setName("ColManager");
		groot->addObject(colMan);*/
		

		/*sofa::component::collision::DefaultCollisionGroupManager::SPtr collisionGroupManager = sofa::core::objectmodel::New<sofa::component::collision::DefaultCollisionGroupManager>(groot.get());
		collisionGroupManager->setName("Collision Group Manager");
		groot->addObject(collisionGroupManager);*/

		//GeomagicDriver
		/*sofa::component::controller::GeomagicDriver::SPtr geomDriver = sofa::core::objectmodel::New<sofa::component::controller::GeomagicDriver>();
		geomDriver->setName("GeomagicDevice");
		geomDriver->d_deviceName.setValue("Default Device");
		geomDriver->d_scale.setValue(1);
		geomDriver->d_frameVisu.setValue(true);
		geomDriver->d_positionBase.setValue(sofa::defaulttype::Vector3(0, -3, 0));
		geomDriver->d_orientationBase.setValue(sofa::defaulttype::Quat(0, 0.707, 0, -0.707));
		groot->addObject(geomDriver);*/


		//------------------------------------------------------------------------------------------------------ //
		//-------------------------------------Thorac-----------------------------------------------------------//
	
		/*sofa::simulation::Node::SPtr thoracNode = groot.get()->createChild("Thorac");

		//Visual
		sofa::simulation::Node::SPtr thoracVisu = thoracNode.get()->createChild("Thorac Visu");

		OglModel::SPtr visualThoracModel = sofa::core::objectmodel::New<OglModel>();
		visualThoracModel->setName("Visual thorac");
		visualThoracModel->load(sofa::helper::system::DataRepository.getFile("mesh/diploma/cylinder.obj"), "", "");
		visualThoracModel->setColor("white");
		thoracVisu->addObject(visualThoracModel);
		
		
		//Collision
		sofa::simulation::Node::SPtr thoracsurf = thoracNode.get()->createChild("Thorac Surf");

		//ObjLoader (coarser)
		sofa::component::loader::MeshObjLoader::SPtr thoracLoader = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		thoracLoader->setFilename(sofa::helper::system::DataRepository.getFile("mesh/diploma/kala/thorac_rot.obj"));
		thoracLoader->load();
		thoracsurf->addObject(thoracLoader);

		//Topology
		sofa::component::topology::MeshTopology::SPtr thoracTopology = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		thoracTopology->setSrc("", thoracLoader.get());
		thoracsurf->addObject(thoracTopology);

		//Mechanical Obj
		MechanicalObject3::SPtr thoracMech = sofa::core::objectmodel::New<MechanicalObject3>();
		thoracMech->setName("instrumentCollisionState");
		thoracMech->setSrc("", thoracLoader.get());
		thoracMech->setTranslation(0, 10, 0);
		thoracsurf->addObject(thoracMech);

		//Triangle
		TTriangle_3::SPtr thoracTrian = sofa::core::objectmodel::New<TTriangle_3>();
		thoracTrian->setName("Triangles");
		thoracTrian->setContactStiffness(100);
		thoracTrian->setContactFriction(0.01);
		thoracTrian->setSimulated(false);
		thoracTrian->setMoving(false);
		thoracsurf->addObject(thoracTrian);

		//Line
		TLine_3::SPtr thoracLines = sofa::core::objectmodel::New<TLine_3>();
		thoracLines->setName("Lines");
		thoracLines->setContactStiffness(100);
		thoracLines->setSimulated(false);
		thoracLines->setContactFriction(0.01);
		thoracLines->setMoving(false);
		thoracsurf->addObject(thoracLines);

		//Point
		TPoint_3::SPtr thoracPoints = sofa::core::objectmodel::New<TPoint_3>();
		thoracPoints->setName("Points");
		thoracPoints->setContactStiffness(100);
		thoracPoints->setContactFriction(0.01);

		thoracPoints->setSimulated(false);
		thoracPoints->setMoving(false);
		thoracsurf->addObject(thoracPoints);
		*/




		//Lungs
		sofa::simulation::Node::SPtr lungsFEM = groot.get()->createChild("LungsFEM");
		
		EulerImplicitSolver::SPtr solverLungs = sofa::core::objectmodel::New<EulerImplicitSolver>();
		solverLungs->setName("EulerImplicitSolver");
		groot->addObject(solverLungs);

		CGLinearSolver3::SPtr linearSolverLungs = sofa::core::objectmodel::New<CGLinearSolver3>();
		linearSolverLungs->setName("CGLinearSolver");
		linearSolverLungs->f_maxIter.setValue(25);
		linearSolverLungs->f_tolerance.setValue(1e-9);
		linearSolverLungs->f_smallDenominatorThreshold.setValue(1e-9);
		groot->addObject(linearSolverLungs);

		sofa::component::loader::MeshGmshLoader::SPtr meshLoaderLungs = sofa::core::objectmodel::New<sofa::component::loader::MeshGmshLoader>();
		meshLoaderLungs->setFilename(sofa::helper::system::DataRepository.getFile("Diploma_Objects/lungs/exp/2/lungs_rot_right_2.msh"));
		meshLoaderLungs->load();
		meshLoaderLungs->setScale(2, 2, 2);
		lungsFEM->addObject(meshLoaderLungs);

		sofa::component::topology::TetrahedronSetTopologyContainer::SPtr containerLungs = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyContainer>();
		containerLungs->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(containerLungs);

		
		MechanicalObject3::SPtr dofsLungs = sofa::core::objectmodel::New<MechanicalObject3>();
		dofsLungs->setName("Lungs Dofs");
		dofsLungs->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(dofsLungs);

		sofa::component::topology::TetrahedronSetTopologyContainer::SPtr container = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyContainer>();
		container->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(container);

		sofa::component::topology::TetrahedronSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>::SPtr topoalgo = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>>();
		topoalgo->setName("Topo_Algo");
		topoalgo->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(topoalgo);

		sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr geomalgo = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>();
		geomalgo->setName("Geom_Algo");
		geomalgo->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(geomalgo);

		sofa::component::topology::TetrahedronSetTopologyModifier::SPtr modifier = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyModifier>();
		modifier->setName("Modifier");
		modifier->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(modifier);

		//3)BoxROI
		sofa::helper::vector<sofa::defaulttype::Vec6d> val;
		val[0] = sofa::defaulttype::Vec6d(-1, -1, -1, 1, 1, 1);
		sofa::component::engine::BoxROI<sofa::defaulttype::Vec6dTypes>::SPtr boxr = sofa::core::objectmodel::New<sofa::component::engine::BoxROI<sofa::defaulttype::Vec6dTypes>>();
		boxr->init();
		boxr->setName("Box1");
		boxr->d_drawBoxes.setValue(true);
		boxr->d_alignedBoxes.setValue(val);
		lungsFEM->addObject(boxr);
		//4)Fixed Constraint

		sofa::component::mass::DiagonalMass<sofa::defaulttype::Vec3dTypes, double>::SPtr lartriumdiagMass = sofa::core::objectmodel::New<sofa::component::mass::DiagonalMass<sofa::defaulttype::Vec3dTypes, double>>();
		lartriumdiagMass->setName("Computed using mass");
		lartriumdiagMass->setMassDensity(1);
		lungsFEM->addObject(lartriumdiagMass);

		sofa::component::forcefield::TetrahedralCorotationalFEMForceField<sofa::defaulttype::Vec3Types>::SPtr coorotFEMLungs = sofa::core::objectmodel::New<sofa::component::forcefield::TetrahedralCorotationalFEMForceField<sofa::defaulttype::Vec3Types>>();
		coorotFEMLungs->setName("Co-rotational Forcefield");
		coorotFEMLungs->f_method.setValue("large");
		coorotFEMLungs->setPoissonRatio(0.35);
		coorotFEMLungs->setYoungModulus(5000);
		lungsFEM->addObject(coorotFEMLungs);

		sofa::component::constraintset::PrecomputedConstraintCorrection<sofa::defaulttype::Vec3dTypes>::SPtr preComputed = sofa::core::objectmodel::New<sofa::component::constraintset::PrecomputedConstraintCorrection<sofa::defaulttype::Vec3dTypes>>();
		preComputed->recompute.setValue(false);
		//preComputed->f_fileCompliance.setValue("Right Lung Coarse Patient 2-3567-0.005.comp");
		preComputed->f_fileCompliance.setValue("Diploma_Objects/lungs/exp/Compliance/RightLung_2.comp");
		lungsFEM->addObject(preComputed);

		
		//Box Roi
		//sofa::component::engine::BoxROI<sofa::defaulttype::Vec<6, SReal>>::SPtr Box;
		/*sofa::component::projectiveconstraintset::FixedConstraint<DataTypes>* fc;
		*/
		//sofa::component::engine::BoxROI<sofa::defaulttype::Vec3Types>::SPtr BoxRoi= sofa::core::objectmodel::New<sofa::component::engine::BoxROI<sofa::defaulttype::Vec3Types>>(sofa::defaulttype::Vec3Types);
		//BoxRoi->setName("ROI1");
		//BoxRoi->setSrc("-10 -0.5 -6 10 1.5 4");
		//BoxRoi->boxes.setValue(sofa::helper::vector<sofa::defaulttype::Vec<6, SReal>> ((sofa::defaulttype::Vec<6, SReal>(-1, -1, -1, -1, -1, -1)));
		//lungsFEM->addObject(BoxRoi);
		
		//sofa::component::engine::BoxROI<sofa::defaulttype::Vec<6,SReal>>::SPtr boxr= sofa::core::objectmodel::New<sofa::component::engine::BoxROI<sofa::component::engine::BoxROI<sofa::defaulttype::Vec<6, SReal>>>>();
		
		/*FixedConstraint3::SPtr constraint = sofa::core::objectmodel::New<FixedConstraint3>();
		
		constraint->addConstraint(3);
		constraint->addConstraint(39);
		BoxROI<defaulttype::Vec6dTypes>;
		liverFEM->addObject(constraint);*/
		

	/*	FixedConstraint3::SPtr constraint = sofa::core::objectmodel::New<FixedConstraint3>();

		constraint->add
		

		lungsFEM->addObject(constraint);8?



		//Visual Model
		sofa::simulation::Node::SPtr lungsvisu = lungsFEM.get()->createChild("LiverVisu");

		OglModel::SPtr visualLungs = sofa::core::objectmodel::New<OglModel>();
		visualLungs->setName("Visual");
		visualLungs->load(sofa::helper::system::DataRepository.getFile("mesh/diploma/kala/rotated/lungs_rot_right.obj"), "","textures/diploma/lungs.png" );
		visualLungs->putOnlyTexCoords.setValue(true);
		visualLungs->setScale(2, 2, 2);
		lungsvisu->addObject(visualLungs);

		BarycentricMapping_Vec3d_to_ExteVec3f::SPtr barmappingLungs = sofa::core::objectmodel::New<BarycentricMapping_Vec3d_to_ExteVec3f>();
		barmappingLungs->setName("Visual Mapping");
		barmappingLungs->setModels(dofsLungs.get(), visualLungs.get());
		lungsvisu->addObject(barmappingLungs);

		//Collision Model
		sofa::simulation::Node::SPtr lungssurf = lungsFEM.get()->createChild("LungsSurf");
		
		sofa::component::topology::TriangleSetTopologyContainer::SPtr container2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetTopologyContainer>();
		lungssurf->addObject(container2);


		sofa::component::topology::TriangleSetTopologyModifier::SPtr mod2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetTopologyModifier>();
		lungssurf->addObject(mod2);

		sofa::component::topology::TriangleSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>::SPtr topalgo2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>>();
		lungssurf->addObject(topalgo2);

		sofa::component::topology::TriangleSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr geomalgo2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>();
		lungssurf->addObject(geomalgo2);

		sofa::component::topology::Tetra2TriangleTopologicalMapping::SPtr tetra2trian = sofa::core::objectmodel::New<sofa::component::topology::Tetra2TriangleTopologicalMapping>();
		tetra2trian->setTopologies(container->toBaseMeshTopology(), container2->toBaseMeshTopology());
		lungssurf->addObject(tetra2trian);

		//Triangle
		TTriangle_3::SPtr triangles_col_lungs = sofa::core::objectmodel::New<TTriangle_3>();
		triangles_col_lungs->setName("Triangles");
		lungssurf->addObject(triangles_col_lungs);

		/*----------------------Omni-------------------------------------------------------*/

		sofa::simulation::Node::SPtr omni = groot.get()->createChild("Omni");

		MechanicalObjectRigid3::SPtr dofRigid = sofa::core::objectmodel::New<MechanicalObjectRigid3>();
		dofRigid->setName("DofsRigid");
		//dofRigid->x.setParent(geomDriver->d_posDevice.getData());
		omni->addObject(dofRigid);

		//MechanicalStateController
		sofa::component::controller::MechanicalStateController<sofa::defaulttype::Rigid3dTypes>::SPtr controller = sofa::core::objectmodel::New<sofa::component::controller::MechanicalStateController<sofa::defaulttype::Rigid3dTypes>>();
		controller->f_listening.setValue(true);
		controller->setMainDirection(sofa::defaulttype::Vec3d(-1, 0, 0));
		controller->handleEventTriggersUpdate.setValue(true);
		omni->addObject(controller);


		sofa::simulation::Node::SPtr visuAvatar = omni.get()->createChild("VisuAvatar");
		visuAvatar->setActive(true);

		OglModel::SPtr visual1 = sofa::core::objectmodel::New<OglModel>();
		visual1->setName("Visual");
		visual1->load(sofa::helper::system::DataRepository.getFile("mesh/sphere.obj"), "", "");
		visual1->setColor("gray");
		visual1->setScale(0.1,0.1, 0.1);
		visuAvatar->addObject(visual1);

		//RigidMapping
		RigidMapping_Rigid_to_Extevec3F::SPtr rigidMap2 = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Extevec3F>();
		rigidMap2->setName("VM Mapping");
		rigidMap2->setModels(dofRigid.get(), visual1.get());
		rigidMap2->index.setValue(0);
		visuAvatar->addObject(rigidMap2);



		sofa::simulation::Node::SPtr refModel = omni.get()->createChild("RefModel");

		sofa::component::loader::MeshObjLoader::SPtr loaderFixed = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderFixed->setFilename(sofa::helper::system::DataRepository.getFile("mesh/dental_instrument_centerline.obj"));
		loaderFixed->load();
		refModel->addObject(loaderFixed);

		sofa::component::topology::MeshTopology::SPtr meshTorusFEM = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshTorusFEM->setSrc("", loaderFixed.get());
		refModel->addObject(meshTorusFEM);

		MechanicalObject3::SPtr instrumentColState = sofa::core::objectmodel::New<MechanicalObject3>();
		instrumentColState->setName("instrumentCollisionState");
		instrumentColState->setSrc("", loaderFixed.get());
		instrumentColState->setRotation(0, -180, -90);
		instrumentColState->setTranslation(-0.3, 0, 3.5);
		refModel->addObject(instrumentColState);



		RigidMapping_Rigid_to_Vec3d::SPtr rigidMap3 = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMap3->setModels(dofRigid.get(), instrumentColState.get());
		rigidMap3->index.setValue(0);
		refModel->addObject(rigidMap3);


		sofa::simulation::Node::SPtr refModelRight = omni.get()->createChild("RefModelRight");

		sofa::component::loader::MeshObjLoader::SPtr loaderFixedright = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderFixedright->setFilename(sofa::helper::system::DataRepository.getFile("mesh/dental_instrument_centerline.obj"));
		loaderFixedright->load();
		refModelRight->addObject(loaderFixedright);

		sofa::component::topology::MeshTopology::SPtr meshTorusFEMRight = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshTorusFEMRight->setSrc("", loaderFixedright.get());
		refModelRight->addObject(meshTorusFEMRight);

		MechanicalObject3::SPtr instrumentColStateRight = sofa::core::objectmodel::New<MechanicalObject3>();
		instrumentColStateRight->setName("instrumentCollisionState");
		instrumentColStateRight->setSrc("", loaderFixedright.get());
		instrumentColStateRight->setRotation(0, -180, -90);
		instrumentColStateRight->setTranslation(-0.3, 0.5, 3.5);
		refModelRight->addObject(instrumentColStateRight);

		RigidMapping_Rigid_to_Vec3d::SPtr rigidMap3right = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMap3right->setModels(dofRigid.get(), instrumentColStateRight.get());

		refModelRight->addObject(rigidMap3right);

		sofa::simulation::Node::SPtr refModelleft = omni.get()->createChild("RefModelLeft");

		sofa::component::loader::MeshObjLoader::SPtr loaderFixedleft = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderFixedleft->setFilename(sofa::helper::system::DataRepository.getFile("mesh/dental_instrument_centerline.obj"));
		loaderFixedleft->load();
		refModelleft->addObject(loaderFixedleft);

		sofa::component::topology::MeshTopology::SPtr meshTorusFEMleft = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshTorusFEMleft->setSrc("", loaderFixedleft.get());
		refModelleft->addObject(meshTorusFEMleft);

		MechanicalObject3::SPtr instrumentColStateleft = sofa::core::objectmodel::New<MechanicalObject3>();
		instrumentColStateleft->setName("instrumentCollisionState");
		instrumentColStateleft->setSrc("", loaderFixedleft.get());
		instrumentColStateleft->setRotation(0, -180, -90);
		instrumentColStateleft->setTranslation(-0.3, -0.5, 3.5);
		refModelleft->addObject(instrumentColStateleft);

		RigidMapping_Rigid_to_Vec3d::SPtr rigidMap3left = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMap3left->setModels(dofRigid.get(), instrumentColStateleft.get());
		refModelleft->addObject(rigidMap3left);

		/*----------------------Instrument-------------------------------------------------------*/
		sofa::simulation::Node::SPtr instrum = groot.get()->createChild("Instrument");

		EulerImplicitSolver::SPtr solverInstrum = sofa::core::objectmodel::New<EulerImplicitSolver>();
		solverInstrum->setName("ODE solver");
		solverInstrum->f_rayleighStiffness.setValue(0.05);
		solverInstrum->f_rayleighMass.setValue(1.0);
		instrum->addObject(solverInstrum);


		CGLinearSolver3::SPtr linearSolverIntrum = sofa::core::objectmodel::New<CGLinearSolver3>();
		linearSolverIntrum->setName("linear solver");
		linearSolverIntrum->f_maxIter.setValue(25);
		linearSolverIntrum->f_tolerance.setValue(1e-10);
		linearSolverIntrum->f_smallDenominatorThreshold.setValue(10e-10);
		instrum->addObject(linearSolverIntrum);

		MechanicalObjectRigid3::SPtr dofRigidInstrum = sofa::core::objectmodel::New<MechanicalObjectRigid3>();
		dofRigidInstrum->setName("instrumentState");
		//dofRigidInstrum->x.setParent(geomDriver->d_posDevice.getData());
		/*
		//to 5 einai to position
		//std::cout<<dofRigidInstrum->getDataFields()[0]->getName();
		//for (int i = 0; i < dofRigidInstrum->getDataFields().size(); i++) {
		//	std::cout << "\n" << i;
		//	std::cout<<dofRigidInstrum->getDataFields()[i]->getName();
		//}
		//sofa::modeling::setDataLink(geomDriver->d_posDevice.getData(), dofRigidInstrum->getDataFields()[5]); */
		instrum->addObject(dofRigidInstrum);

		UniformMassRigid3::SPtr uniMassCarvInstrum = sofa::core::objectmodel::New<UniformMassRigid3>();
		uniMassCarvInstrum->setTotalMass(0.05);
		uniMassCarvInstrum->setName("Mass");
		instrum->addObject(uniMassCarvInstrum);

		sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Rigid3dTypes>::SPtr lcpContr = sofa::core::objectmodel::New<sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Rigid3dTypes>>();
		lcpContr->f_activate.setValue(true);
		lcpContr->forceCoef.setValue(0.5);
		instrum->addObject(lcpContr);

		sofa::component::constraintset::UncoupledConstraintCorrection<sofa::defaulttype::Rigid3dTypes>::SPtr constrUncoup=
		sofa::core::objectmodel::New<sofa::component::constraintset::UncoupledConstraintCorrection<sofa::defaulttype::Rigid3dTypes>>();
		instrum->addObject(constrUncoup);


		sofa::simulation::Node::SPtr visuModelInstrument = instrum.get()->createChild("VisualModel");

		OglModel::SPtr visualInstr = sofa::core::objectmodel::New<OglModel>();
		visualInstr->setName("InstrumentVisualModel");
		visualInstr->load(sofa::helper::system::DataRepository.getFile("Demos/Dentistry/data/mesh/dental_instrument.obj"), "", "");
		visualInstr->setColor(1.f,0.2f,0.2f,1.f);
		visualInstr->m_translation.setValue(sofa::defaulttype::Vec3f(-0.3f, 0.f, 3.5f));
		visualInstr->m_rotation.setValue(sofa::defaulttype::Vec3f(0.f, -180.f, -90.f));
		visuModelInstrument->addObject(visualInstr);

		//RigidMapping
		RigidMapping_Rigid_to_Extevec3F::SPtr rigidMapInstr = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Extevec3F>();
		rigidMapInstr->setName("VM Mapping");
		rigidMapInstr->setModels(dofRigidInstrum.get(), visualInstr.get());
		visuModelInstrument->addObject(rigidMapInstr);

		

		sofa::simulation::Node::SPtr collModelInstrument = instrum.get()->createChild("CollisionModel");


		sofa::component::loader::MeshObjLoader::SPtr loaderColl = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderColl->setName("loader");
		loaderColl->setFilename(sofa::helper::system::DataRepository.getFile("Demos/Dentistry/data/mesh/dental_instrument_centerline.obj"));
		loaderColl->load();
		collModelInstrument->addObject(loaderColl);

		sofa::component::topology::MeshTopology::SPtr meshColl = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshColl->setSrc("", loaderColl.get());
		collModelInstrument->addObject(meshColl);

		MechanicalObject3::SPtr mechColl = sofa::core::objectmodel::New<MechanicalObject3>();
		mechColl->setName("instrumentCollisionState");
		mechColl->setSrc("", loaderColl.get());
		mechColl->setRotation(0, -180, -90);
		mechColl->setTranslation(-0.3, 0, 3.5);
		collModelInstrument->addObject(mechColl);

		//Lines
		TLine_3::SPtr line_coll_instr = sofa::core::objectmodel::New<TLine_3>();
		line_coll_instr->setName("Lines");
		line_coll_instr->setContactStiffness(10.0);
		line_coll_instr->setContactFriction(0.01);
		line_coll_instr->addTag(sofa::core::objectmodel::Tag("CarvingTool"));
		collModelInstrument->addObject(line_coll_instr);

		//Points
		TPoint_3::SPtr points_col_instr = sofa::core::objectmodel::New<TPoint_3>();
		points_col_instr->setName("Points");
		points_col_instr->setContactStiffness(10.0);
		points_col_instr->setContactFriction(0.01);
		points_col_instr->addTag(sofa::core::objectmodel::Tag("CarvingTool"));
		collModelInstrument->addObject(points_col_instr);

		//RigidMapping
		RigidMapping_Rigid_to_Vec3d::SPtr rigidMapInstrCol = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMapInstrCol->setName("VM Mapping");
		rigidMapInstrCol->setModels(dofRigidInstrum.get(), mechColl.get());
		collModelInstrument->addObject(rigidMapInstrCol);

		
		sofa::helper::vector<double> vec;
		vec.push_back(0);
		vec.push_back(0);
		vec.push_back(0);

		sofa::component::collision::SphereModel::SPtr sphereModelCarv = sofa::core::objectmodel::New<sofa::component::collision::SphereModel>();
		sphereModelCarv->setName("CollisionModel");
		sphereModelCarv->defaultRadius.setValue(0.02);
		sphereModelCarv->radius.setValue(vec);
		collModelInstrument->addObject(sphereModelCarv);

		///////////////////////////////////////*/
		//Right

		sofa::simulation::Node::SPtr collModelInstrumentRight = instrum.get()->createChild("RefModelRight");


		sofa::component::loader::MeshObjLoader::SPtr loaderCollRight = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderCollRight->setFilename(sofa::helper::system::DataRepository.getFile("Demos/Dentistry/data/mesh/dental_instrument_centerline.obj"));
		loaderCollRight->load();
		collModelInstrumentRight->addObject(loaderCollRight);

		sofa::component::topology::MeshTopology::SPtr meshCollRight = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshCollRight->setSrc("", loaderCollRight.get());
		collModelInstrumentRight->addObject(meshCollRight);

		MechanicalObject3::SPtr mechCollRight = sofa::core::objectmodel::New<MechanicalObject3>();
		mechCollRight->setName("instrumentCollisionState");
		mechCollRight->setSrc("", loaderCollRight.get());
		mechCollRight->setRotation(0, -180, -90);
		mechCollRight->setTranslation(-0.3, 0.5, 3.5);
		collModelInstrumentRight->addObject(mechCollRight);



		//RigidMapping
		RigidMapping_Rigid_to_Vec3d::SPtr rigidMapInstrColRight = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMapInstrColRight->setName("VM Mapping");
		rigidMapInstrColRight->setModels(dofRigidInstrum.get(), mechCollRight.get());
		collModelInstrumentRight->addObject(rigidMapInstrColRight);

		//Left

		sofa::simulation::Node::SPtr collModelInstrumentLeft = instrum.get()->createChild("RefModelLeft");


		sofa::component::loader::MeshObjLoader::SPtr loaderCollLeft = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderCollLeft->setFilename(sofa::helper::system::DataRepository.getFile("Demos/Dentistry/data/mesh/dental_instrument_centerline.obj"));
		loaderCollLeft->load();
		collModelInstrumentLeft->addObject(loaderCollLeft);

		sofa::component::topology::MeshTopology::SPtr meshCollLeft = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshCollLeft->setSrc("", loaderCollLeft.get());
		collModelInstrumentLeft->addObject(meshCollLeft);

		MechanicalObject3::SPtr mechCollLeft = sofa::core::objectmodel::New<MechanicalObject3>();
		mechCollLeft->setName("instrumentCollisionState");
		mechCollLeft->setSrc("", loaderCollLeft.get());
		mechCollLeft->setRotation(0, -180, -90);
		mechCollLeft->setTranslation(-0.3, -0.5, 3.5);
		collModelInstrumentLeft->addObject(mechCollLeft);



		//RigidMapping
		RigidMapping_Rigid_to_Vec3d::SPtr rigidMapInstrColLeft = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMapInstrColLeft->setName("VM Mapping");
		rigidMapInstrColLeft->setModels(dofRigidInstrum.get(), mechCollLeft.get());
		collModelInstrumentLeft->addObject(rigidMapInstrColLeft);



		//VectorSpringForceField
		sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>::SPtr vSpringForc =
		sofa::core::objectmodel::New<sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>>();
		vSpringForc->setPathObject1("@Omni/RefModel/instrumentCollisionState");
		vSpringForc->setPathObject2("@Instrument/CollisionModel/instrumentCollisionState");
		vSpringForc->m_stiffness.setValue(10);
		vSpringForc->m_viscosity.setValue(0);
		instrum->addObject(vSpringForc);

		sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>::SPtr vSpringForc2 =
		sofa::core::objectmodel::New<sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>>();

		vSpringForc2->setPathObject1("@Omni/RefModelRight/instrumentCollisionState");
		vSpringForc2->setPathObject2("@Instrument/RefModelRight/instrumentCollisionState");
		vSpringForc2->m_stiffness.setValue(10);
		vSpringForc2->m_viscosity.setValue(0);
		instrum->addObject(vSpringForc2);

		sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>::SPtr vSpringForc3 =
		sofa::core::objectmodel::New<sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>>();

		vSpringForc3->setPathObject1("@Omni/RefModelLeft/instrumentCollisionState");
		vSpringForc3->setPathObject2("@Instrument/RefModelLeft/instrumentCollisionState");
		vSpringForc3->m_stiffness.setValue(10);
		vSpringForc3->m_viscosity.setValue(0);
		instrum->addObject(vSpringForc3);
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/*______Continue RunSofa______________*/



		if (computationTimeAtBegin)
		{
			sofa::helper::AdvancedTimer::setEnabled("Init", true);
			sofa::helper::AdvancedTimer::setInterval("Init", 1);
			sofa::helper::AdvancedTimer::setOutputType("Init", computationTimeOutputType);
			sofa::helper::AdvancedTimer::begin("Init");
		}

		sofa::simulation::getSimulation()->init(groot.get());
		if (computationTimeAtBegin)
		{
			msg_info("") << sofa::helper::AdvancedTimer::end("Init", groot.get());
		}

		GUIManager::SetScene(groot);

		if (startAnim)
			groot->setAnimate(true);

		if (printFactory)
		{
			msg_info("") << "////////// FACTORY //////////";
			sofa::helper::printFactoryLog();
			msg_info("") << "//////// END FACTORY ////////";
		}

		if (computationTimeSampling > 0)
		{
			sofa::helper::AdvancedTimer::setEnabled("Animate", true);
			sofa::helper::AdvancedTimer::setInterval("Animate", computationTimeSampling);
			sofa::helper::AdvancedTimer::setOutputType("Animate", computationTimeOutputType);
		}
		if (int err = GUIManager::MainLoop(groot))
			return err;
		groot = dynamic_cast<Node*>(GUIManager::CurrentSimulation());


		if (groot != NULL)
			sofa::simulation::getSimulation()->unload(groot);



		GUIManager::closeGUI();

		sofa::simulation::common::cleanup();
		sofa::simulation::tree::cleanup();
#ifdef SOFA_HAVE_DAG
		sofa::simulation::graph::cleanup();
#endif
		return 0;
	}

	



	else if (sceneNOHaptics) {

		GUIManager::SetDimension(width, height);

		Node::SPtr groot = sofa::simulation::getSimulation()->createNewGraph("Root");
		groot->setGravity(sofa::defaulttype::Vector3(0, 0, 0));
		groot->setDt(0.05);

		sofa::component::visualmodel::VisualStyle::SPtr style = sofa::core::objectmodel::New<sofa::component::visualmodel::VisualStyle>();
		sofa::core::visual::DisplayFlags& flags = *style->displayFlags.beginEdit();
		flags.setShowCollisionModels(false);
		flags.setShowVisualModels(true);
		style->displayFlags.endEdit();
		groot->addObject(style);

		sofa::component::collision::DefaultPipeline::SPtr pipeline = sofa::core::objectmodel::New<sofa::component::collision::DefaultPipeline>();
		pipeline->setName("CollisionPipeline");
		pipeline->d_depth.setValue(6);
		groot->addObject(pipeline);

		sofa::component::collision::BruteForceDetection::SPtr bruteforce = sofa::core::objectmodel::New<sofa::component::collision::BruteForceDetection>();
		bruteforce->setName("N2");
		groot->addObject(bruteforce);

		//CollisionResponse
		sofa::component::collision::DefaultContactManager::SPtr response = sofa::core::objectmodel::New<sofa::component::collision::DefaultContactManager>();
		response->setName("Collision Response");
		response->setDefaultResponseType("FrictionContact");
		groot->addObject(response);

		//LocalMinDistance
		sofa::component::collision::LocalMinDistance::SPtr localMinDist = sofa::core::objectmodel::New<sofa::component::collision::LocalMinDistance>();
		localMinDist->setName("Proximity");
		localMinDist->alarmDistance.setValue(1.5);
		localMinDist->contactDistance.setValue(0.05);
		groot->addObject(localMinDist);

		//FreeMotionAnimationLoop
		sofa::component::animationloop::FreeMotionAnimationLoop::SPtr freemot = sofa::core::objectmodel::New<sofa::component::animationloop::FreeMotionAnimationLoop>(groot.get());
		freemot->setName("FreeMotionLoop");
		groot->addObject(freemot);

		//LCPConstraintSolver
		sofa::component::constraintset::LCPConstraintSolver::SPtr lcpSolver = sofa::core::objectmodel::New<sofa::component::constraintset::LCPConstraintSolver>();
		lcpSolver->tol.setValue(0.001);
		lcpSolver->maxIt.setValue(1000);
		groot->addObject(lcpSolver);



		//Lungs
		sofa::simulation::Node::SPtr lungsFEM = groot.get()->createChild("LungsFEM");

		EulerImplicitSolver::SPtr solverLungs = sofa::core::objectmodel::New<EulerImplicitSolver>();
		solverLungs->setName("EulerImplicitSolver");
		groot->addObject(solverLungs);

		CGLinearSolver3::SPtr linearSolverLungs = sofa::core::objectmodel::New<CGLinearSolver3>();
		linearSolverLungs->setName("CGLinearSolver");
		linearSolverLungs->f_maxIter.setValue(25);
		linearSolverLungs->f_tolerance.setValue(1e-9);
		linearSolverLungs->f_smallDenominatorThreshold.setValue(1e-9);
		groot->addObject(linearSolverLungs);

		sofa::component::loader::MeshGmshLoader::SPtr meshLoaderLungs = sofa::core::objectmodel::New<sofa::component::loader::MeshGmshLoader>();
		meshLoaderLungs->setFilename(sofa::helper::system::DataRepository.getFile("Diploma_Objects/lungs/exp/6/lungs_rot_right_6.msh"));
		meshLoaderLungs->load();
		meshLoaderLungs->setScale(2, 2, 2);
		lungsFEM->addObject(meshLoaderLungs);

		sofa::component::topology::TetrahedronSetTopologyContainer::SPtr containerLungs = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyContainer>();
		containerLungs->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(containerLungs);


		MechanicalObject3::SPtr dofsLungs = sofa::core::objectmodel::New<MechanicalObject3>();
		dofsLungs->setName("Lungs Dofs");
		dofsLungs->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(dofsLungs);

		sofa::component::topology::TetrahedronSetTopologyContainer::SPtr container = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyContainer>();
		container->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(container);

		sofa::component::topology::TetrahedronSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>::SPtr topoalgo = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>>();
		topoalgo->setName("Topo_Algo");
		topoalgo->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(topoalgo);

		sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr geomalgo = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>();
		geomalgo->setName("Geom_Algo");
		geomalgo->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(geomalgo);

		sofa::component::topology::TetrahedronSetTopologyModifier::SPtr modifier = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyModifier>();
		modifier->setName("Modifier");
		modifier->setSrc("", meshLoaderLungs.get());
		lungsFEM->addObject(modifier);

		//3)BoxROI
		//4)Fixed Constraint

		sofa::component::mass::DiagonalMass<sofa::defaulttype::Vec3dTypes, double>::SPtr lartriumdiagMass = sofa::core::objectmodel::New<sofa::component::mass::DiagonalMass<sofa::defaulttype::Vec3dTypes, double>>();
		lartriumdiagMass->setName("Computed using mass");
		lartriumdiagMass->setMassDensity(1);
		lungsFEM->addObject(lartriumdiagMass);

		sofa::component::forcefield::TetrahedralCorotationalFEMForceField<sofa::defaulttype::Vec3Types>::SPtr coorotFEMLungs = sofa::core::objectmodel::New<sofa::component::forcefield::TetrahedralCorotationalFEMForceField<sofa::defaulttype::Vec3Types>>();
		coorotFEMLungs->setName("Co-rotational Forcefield");
		coorotFEMLungs->f_method.setValue("large");
		coorotFEMLungs->setPoissonRatio(0.35);
		coorotFEMLungs->setYoungModulus(5000);
		lungsFEM->addObject(coorotFEMLungs);

		sofa::component::constraintset::PrecomputedConstraintCorrection<sofa::defaulttype::Vec3dTypes>::SPtr preComputed = sofa::core::objectmodel::New<sofa::component::constraintset::PrecomputedConstraintCorrection<sofa::defaulttype::Vec3dTypes>>();
		preComputed->recompute.setValue(false);
		preComputed->f_fileCompliance.setValue("Right Lung Coarse Patient 2-3567-0.005.comp");
		lungsFEM->addObject(preComputed);


		//Box Roi
		//sofa::component::engine::BoxROI<sofa::defaulttype::Vec<6, SReal>>::SPtr Box;
		/*sofa::component::projectiveconstraintset::FixedConstraint<DataTypes>* fc;
		*/
		/*sofa::component::engine::BoxROI<sofa::defaulttype::Vec<6, double>>::SPtr BoxRoi= sofa::core::objectmodel::New<sofa::component::engine::BoxROI<sofa::defaulttype::Vec<6, double>>>();
		BoxRoi->setName("ROI1");
		BoxRoi->setSrc("-10 -0.5 -6 10 1.5 4");
		//BoxRoi->boxes.setValue(sofa::helper::vector<sofa::defaulttype::Vec<6, SReal>> ((sofa::defaulttype::Vec<6, SReal>(-1, -1, -1, -1, -1, -1)));
		lungsFEM->addObject(BoxRoi);
		*/


		/*FixedConstraint3::SPtr constraint = sofa::core::objectmodel::New<FixedConstraint3>();

		constraint->addConstraint(3);
		constraint->addConstraint(39);

		liverFEM->addObject(constraint);*/
		sofa::helper::vector<sofa::defaulttype::Vec6d> val;
		val.push_back( sofa::defaulttype::Vec6d(-1., -1., -1., 1., 1., 1.));
		sofa::component::engine::BoxROI<sofa::defaulttype::Vec6dTypes>::SPtr boxr = sofa::core::objectmodel::New<sofa::component::engine::BoxROI<sofa::defaulttype::Vec6dTypes>>();
		//boxr->init();
		boxr->setName("Box1");
		boxr->d_drawBoxes.setValue(true);
		boxr->d_alignedBoxes.setValue(val);
		lungsFEM->addObject(boxr);

		FixedConstraint3::SPtr constraint = sofa::core::objectmodel::New<FixedConstraint3>();
		constraint->d_indices.setParent(boxr->d_indices.getData());
		lungsFEM->addObject(constraint);

		//Visual Model
		sofa::simulation::Node::SPtr lungsvisu = lungsFEM.get()->createChild("LiverVisu");

		OglModel::SPtr visualLungs = sofa::core::objectmodel::New<OglModel>();
		visualLungs->setName("Visual");
		visualLungs->load(sofa::helper::system::DataRepository.getFile("mesh/diploma/kala/rotated/lungs_rot_right.obj"), "", "textures/diploma/lungs.png");
		visualLungs->putOnlyTexCoords.setValue(true);
		visualLungs->setScale(2, 2, 2);
		lungsvisu->addObject(visualLungs);

		BarycentricMapping_Vec3d_to_ExteVec3f::SPtr barmappingLungs = sofa::core::objectmodel::New<BarycentricMapping_Vec3d_to_ExteVec3f>();
		barmappingLungs->setName("Visual Mapping");
		barmappingLungs->setModels(dofsLungs.get(), visualLungs.get());
		lungsvisu->addObject(barmappingLungs);

		//Collision Model
		sofa::simulation::Node::SPtr lungssurf = lungsFEM.get()->createChild("LungsSurf");

		sofa::component::topology::TriangleSetTopologyContainer::SPtr container2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetTopologyContainer>();
		lungssurf->addObject(container2);


		sofa::component::topology::TriangleSetTopologyModifier::SPtr mod2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetTopologyModifier>();
		lungssurf->addObject(mod2);

		sofa::component::topology::TriangleSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>::SPtr topalgo2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>>();
		lungssurf->addObject(topalgo2);

		sofa::component::topology::TriangleSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr geomalgo2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>();
		lungssurf->addObject(geomalgo2);

		sofa::component::topology::Tetra2TriangleTopologicalMapping::SPtr tetra2trian = sofa::core::objectmodel::New<sofa::component::topology::Tetra2TriangleTopologicalMapping>();
		tetra2trian->setTopologies(container->toBaseMeshTopology(), container2->toBaseMeshTopology());
		tetra2trian->flipNormals.setValue(true);
		lungssurf->addObject(tetra2trian);

		//Triangle
		TTriangle_3::SPtr triangles_col_lungs = sofa::core::objectmodel::New<TTriangle_3>();
		triangles_col_lungs->setName("Triangles");
		//triangles_col_lungs->addGroup(0);
		lungssurf->addObject(triangles_col_lungs);



		/*----------------------Instrument-------------------------------------------------------*/
		sofa::simulation::Node::SPtr instrum = groot.get()->createChild("Instrument");

		EulerImplicitSolver::SPtr solverInstrum = sofa::core::objectmodel::New<EulerImplicitSolver>();
		solverInstrum->setName("ODE solver");
		solverInstrum->f_rayleighStiffness.setValue(0.05);
		solverInstrum->f_rayleighMass.setValue(1.0);
		instrum->addObject(solverInstrum);


		CGLinearSolver3::SPtr linearSolverIntrum = sofa::core::objectmodel::New<CGLinearSolver3>();
		linearSolverIntrum->setName("linear solver");
		linearSolverIntrum->f_maxIter.setValue(25);
		linearSolverIntrum->f_tolerance.setValue(1e-10);
		linearSolverIntrum->f_smallDenominatorThreshold.setValue(10e-10);
		instrum->addObject(linearSolverIntrum);

		sofa::component::loader::MeshObjLoader::SPtr loaderin = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderin->setName("loader");
		loaderin->setFilename(sofa::helper::system::DataRepository.getFile("Demos/Dentistry/data/mesh/dental_instrument_centerline.obj"));
		loaderin->load();
		instrum->addObject( loaderin);

		sofa::component::topology::MeshTopology::SPtr meshColl = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshColl->setSrc("", loaderin.get());
		instrum->addObject(meshColl);

		MechanicalObjectRigid3::SPtr dofRigidInstrum = sofa::core::objectmodel::New<MechanicalObjectRigid3>();
		dofRigidInstrum->setName("instrumentState");
		dofRigidInstrum->setRotation(0, 90, 0);
		dofRigidInstrum->setTranslation(7, 2.3, -0.5);
		//dofRigidInstrum->x.setParent(geomDriver->d_posDevice.getData());
		/*
		//to 5 einai to position
		//std::cout<<dofRigidInstrum->getDataFields()[0]->getName();
		//for (int i = 0; i < dofRigidInstrum->getDataFields().size(); i++) {
		//	std::cout << "\n" << i;
		//	std::cout<<dofRigidInstrum->getDataFields()[i]->getName();
		//}
		//sofa::modeling::setDataLink(geomDriver->d_posDevice.getData(), dofRigidInstrum->getDataFields()[5]); */
		instrum->addObject(dofRigidInstrum);

		UniformMassRigid3::SPtr uniMassCarvInstrum = sofa::core::objectmodel::New<UniformMassRigid3>();
		uniMassCarvInstrum->setTotalMass(0.05);
		uniMassCarvInstrum->setName("Mass");
		instrum->addObject(uniMassCarvInstrum);
		
		sofa::helper::vector<double> vectimes;
		vectimes.push_back(0);
		vectimes.push_back(100);

		//typedef sofa::defaulttype::RigidDeriv<3, double> rigidmoves;
		//sofa::defaulttype::RigidDeriv<3, double>(sofa::defaulttype::Vec3d(0, 0, 0), sofa::defaulttype::Vec3d(0, 0, 0));
		sofa::helper::vector<sofa::defaulttype::RigidDeriv<3, double>> vecmoves;
		vecmoves.push_back(sofa::defaulttype::RigidDeriv<3, double>(sofa::defaulttype::Vec3d(0, 0, 0), sofa::defaulttype::Vec3d(0, 0, 0)));
		vecmoves.push_back(sofa::defaulttype::RigidDeriv<3, double>(sofa::defaulttype::Vec3d(-10, 0, 0), sofa::defaulttype::Vec3d(0, 0, 0)));

		sofa::component::projectiveconstraintset::LinearMovementConstraint<sofa::defaulttype::Rigid3dTypes>::SPtr linearmotion= sofa::core::objectmodel::New<sofa::component::projectiveconstraintset::LinearMovementConstraint<sofa::defaulttype::Rigid3dTypes>>();
		linearmotion->m_keyTimes.setValue(vectimes);
		linearmotion->m_keyMovements.setValue(vecmoves);

		instrum-> addObject(linearmotion);
		
		sofa::simulation::Node::SPtr visuModelInstrument = instrum.get()->createChild("VisualModel");
		
		OglModel::SPtr visualInstr = sofa::core::objectmodel::New<OglModel>();
		visualInstr->setName("InstrumentVisualModel");
		visualInstr->load(sofa::helper::system::DataRepository.getFile("Demos/Dentistry/data/mesh/dental_instrument.obj"), "", "");
		visualInstr->setColor(1.f, 0.2f, 0.2f, 1.f);
		visualInstr->m_translation.setValue(sofa::defaulttype::Vec3f(-0.3f, 0.f, 3.5f));
		visualInstr->m_rotation.setValue(sofa::defaulttype::Vec3f(0.f, -180.f, -90.f));
		visuModelInstrument->addObject(visualInstr);

		//RigidMapping
		RigidMapping_Rigid_to_Extevec3F::SPtr rigidMapInstr = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Extevec3F>();
		rigidMapInstr->setName("VM Mapping");
		rigidMapInstr->setModels(dofRigidInstrum.get(), visualInstr.get());
		visuModelInstrument->addObject(rigidMapInstr);



		sofa::simulation::Node::SPtr collModelInstrument = instrum.get()->createChild("CollisionModel");


		sofa::component::loader::MeshObjLoader::SPtr loaderColl = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderColl->setName("loader");
		loaderColl->setFilename(sofa::helper::system::DataRepository.getFile("Demos/Dentistry/data/mesh/dental_instrument_centerline.obj"));
		loaderColl->load();
		collModelInstrument->addObject(loaderColl);

		sofa::component::topology::MeshTopology::SPtr topcol = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		topcol->setSrc("", loaderColl.get());
		collModelInstrument->addObject(topcol);

		MechanicalObject3::SPtr mechanColl = sofa::core::objectmodel::New<MechanicalObject3>();
		mechanColl->setName("instrumentCollisionState");
		mechanColl->setSrc("", loaderColl.get());
		mechanColl->setRotation(0, -180, -90);
		mechanColl->setTranslation(-0.3, 0, 3.5);
		collModelInstrument->addObject(mechanColl);

		//Lines
		TLine_3::SPtr line_coll_instr = sofa::core::objectmodel::New<TLine_3>();
		line_coll_instr->setName("Lines");
		line_coll_instr->setContactStiffness(10.0);
		line_coll_instr->setContactFriction(0.01);
		//line_coll_instr->addGroup(0);
		collModelInstrument->addObject(line_coll_instr);

		//Points
		TPoint_3::SPtr points_col_instr = sofa::core::objectmodel::New<TPoint_3>();
		points_col_instr->setName("Points");
		points_col_instr->setContactStiffness(10.0);
		points_col_instr->setContactFriction(0.01);	
		//points_col_instr->addGroup(0);
		collModelInstrument->addObject(points_col_instr);

		sofa::helper::vector<double> vec;
		vec.push_back(0);
		vec.push_back(0);
		vec.push_back(0);

		sofa::component::collision::SphereModel::SPtr sphereModelCarv = sofa::core::objectmodel::New<sofa::component::collision::SphereModel>();
		sphereModelCarv->setName("CollisionModel");
		sphereModelCarv->defaultRadius.setValue(0.02);
		sphereModelCarv->radius.setValue(vec);
		//sphereModelCarv->addGroup(0);
		collModelInstrument->addObject(sphereModelCarv);

		//RigidMapping
		RigidMapping_Rigid_to_Vec3d::SPtr rigidMapInstrCol = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMapInstrCol->setName("VM Mapping");
		rigidMapInstrCol->setModels(dofRigidInstrum.get(), mechanColl.get());
		collModelInstrument->addObject(rigidMapInstrCol);
		

	


		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/*______Continue RunSofa______________*/



		if (computationTimeAtBegin)
		{
			sofa::helper::AdvancedTimer::setEnabled("Init", true);
			sofa::helper::AdvancedTimer::setInterval("Init", 1);
			sofa::helper::AdvancedTimer::setOutputType("Init", computationTimeOutputType);
			sofa::helper::AdvancedTimer::begin("Init");
		}

		sofa::simulation::getSimulation()->init(groot.get());
		if (computationTimeAtBegin)
		{
			msg_info("") << sofa::helper::AdvancedTimer::end("Init", groot.get());
		}

		GUIManager::SetScene(groot);

		if (startAnim)
			groot->setAnimate(true);

		if (printFactory)
		{
			msg_info("") << "////////// FACTORY //////////";
			sofa::helper::printFactoryLog();
			msg_info("") << "//////// END FACTORY ////////";
		}

		if (computationTimeSampling > 0)
		{
			sofa::helper::AdvancedTimer::setEnabled("Animate", true);
			sofa::helper::AdvancedTimer::setInterval("Animate", computationTimeSampling);
			sofa::helper::AdvancedTimer::setOutputType("Animate", computationTimeOutputType);
		}
		if (int err = GUIManager::MainLoop(groot))
			return err;
		groot = dynamic_cast<Node*>(GUIManager::CurrentSimulation());


		if (groot != NULL)
			sofa::simulation::getSimulation()->unload(groot);



		GUIManager::closeGUI();

		sofa::simulation::common::cleanup();
		sofa::simulation::tree::cleanup();
#ifdef SOFA_HAVE_DAG
		sofa::simulation::graph::cleanup();
#endif
		return 0;
	}



	else if (scene) {

		GUIManager::SetDimension(width, height);

		Node::SPtr groot = sofa::simulation::getSimulation()->createNewGraph("Root");
		groot->setGravity(sofa::defaulttype::Vector3(0, 0,-10.9));
		groot->setDt(0.01);

		sofa::component::visualmodel::VisualStyle::SPtr style = sofa::core::objectmodel::New<sofa::component::visualmodel::VisualStyle>();
		sofa::core::visual::DisplayFlags& flags = *style->displayFlags.beginEdit();
		flags.setShowCollisionModels(true);
		flags.setShowVisualModels(false);
		style->displayFlags.endEdit();
		groot->addObject(style);

		sofa::component::collision::DefaultPipeline::SPtr pipeline = sofa::core::objectmodel::New<sofa::component::collision::DefaultPipeline>();
		pipeline->setName("CollisionPipeline");
		pipeline->d_depth.setValue(6);
		groot->addObject(pipeline);

		sofa::component::collision::BruteForceDetection::SPtr bruteforce = sofa::core::objectmodel::New<sofa::component::collision::BruteForceDetection>();
		bruteforce->setName("N2");
		groot->addObject(bruteforce);

		//CollisionResponse
		sofa::component::collision::DefaultContactManager::SPtr response = sofa::core::objectmodel::New<sofa::component::collision::DefaultContactManager>();
		response->setName("Collision Response");
		response->setDefaultResponseType("FrictionContact");
		groot->addObject(response);

		/*sofa::component::collision::MinProximityIntersection::SPtr proximity = sofa::modeling::addNew<sofa::component::collision::MinProximityIntersection>(groot);
		proximity->setName("Proximity");
		proximity->alarmDistance.setValue(0.5);
		proximity->contactDistance.setValue(0.1);
		groot->addObject(proximity);*/

		//LocalMinDistance
		sofa::component::collision::LocalMinDistance::SPtr localMinDist = sofa::core::objectmodel::New<sofa::component::collision::LocalMinDistance>();
		localMinDist->setName("Proximity");
		localMinDist->alarmDistance.setValue(0.15);
		localMinDist->contactDistance.setValue(0.05);
		localMinDist->angleCone.setValue(0.0);
		groot->addObject(localMinDist);
		

		sofa::component::collision::CarvingManager::SPtr carv= sofa::core::objectmodel::New<sofa::component::collision::CarvingManager>();
		carv->d_active.setValue(true);
		carv->d_carvingDistance.setValue(-0.09);
		carv->setName("carving");
		groot->addObject(carv);
		
		//FreeMotionAnimationLoop
		sofa::component::animationloop::FreeMotionAnimationLoop::SPtr freemot= sofa::core::objectmodel::New<sofa::component::animationloop::FreeMotionAnimationLoop>(groot.get());
		freemot->setName("Loop");
		groot->addObject(freemot);

		//LCPConstraintSolver
		sofa::component::constraintset::LCPConstraintSolver::SPtr lcpSolver = sofa::core::objectmodel::New<sofa::component::constraintset::LCPConstraintSolver>();
		lcpSolver->tol.setValue(0.001);
		lcpSolver->maxIt.setValue(1000);
		groot->addObject(lcpSolver);

		/*sofa::core::collision::CollisionGroupManager::SPtr colMan= sofa::core::objectmodel::New<sofa::core::collision::CollisionGroupManager>(*groot->toCollisionGroupManager());
		colMan->setName("ColManager");
		groot->addObject(colMan);*/
		
		//GeomagicDriver
		sofa::component::controller::GeomagicDriver::SPtr geomDriver = sofa::core::objectmodel::New<sofa::component::controller::GeomagicDriver>();
		geomDriver->setName("GeomagicDevice");
		geomDriver->d_deviceName.setValue("Default Device");
		geomDriver->d_scale.setValue(1);
		geomDriver->d_frameVisu.setValue(true);
		geomDriver->d_positionBase.setValue(sofa::defaulttype::Vector3(0, 0, 0));	
		geomDriver->d_orientationBase.setValue(sofa::defaulttype::Quat(0, 0.707, 0, - 0.707));
		/*sofa::component::controller::GeomagicDriver::Coord & posDevice = *geomDriver->d_posDevice.beginEdit();
		posDevice.getCenter() = geomDriver->d_positionBase.getValue();
		posDevice.getOrientation() = geomDriver->d_orientationBase.getValue();
		geomDriver->d_posDevice.endEdit();*/
		groot->addObject(geomDriver);

		//-------------------------------------Skin-----------------------------------------------------------//


		//------------------------------------------------------------------------------------------------------ //
		//-------------------------------------Thorac-----------------------------------------------------------//
		//---------------------------------------------------------------------------------------------------- //
/*		sofa::simulation::Node::SPtr thoracNode = groot.get()->createChild("Thorac");

		//Visual
		/*sofa::simulation::Node::SPtr thoracVisu = thoracNode.get()->createChild("Thorac Visu");

		OglModel::SPtr visualThoracModel = sofa::core::objectmodel::New<OglModel>();
		visualThoracModel->setName("Visual thorac");
		visualThoracModel->load(sofa::helper::system::DataRepository.getFile("mesh/diploma/cylinder.obj"), "", "");
		visualThoracModel->setColor("white");
		thoracVisu->addObject(visualThoracModel);
		*/
		//Collision
/*		sofa::simulation::Node::SPtr thoracsurf = thoracNode.get()->createChild("Thorac Surf");

		//ObjLoader (coarser)
		sofa::component::loader::MeshObjLoader::SPtr thoracLoader = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		thoracLoader->setFilename(sofa::helper::system::DataRepository.getFile("mesh/diploma/kala/thorac_rot.obj"));
		thoracLoader->load();
		thoracsurf->addObject(thoracLoader);

		//Topology
		sofa::component::topology::MeshTopology::SPtr thoracTopology = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		thoracTopology->setSrc("", thoracLoader.get());
		thoracsurf->addObject(thoracTopology);

		//Mechanical Obj
		MechanicalObject3::SPtr thoracMech = sofa::core::objectmodel::New<MechanicalObject3>();
		thoracMech->setName("instrumentCollisionState");
		thoracMech->setSrc("", thoracLoader.get());
		thoracMech->setTranslation(0, 10, 0);
		thoracsurf->addObject(thoracMech);

		//Triangle
		TTriangle_3::SPtr thoracTrian = sofa::core::objectmodel::New<TTriangle_3>();
		thoracTrian->setName("Triangles");
		thoracTrian->setContactStiffness(100);
		thoracTrian->setContactFriction(0.01);
		thoracTrian->setSimulated(false);
		thoracTrian->setMoving(false);
		thoracsurf->addObject(thoracTrian);

		//Line 
		TLine_3::SPtr thoracLines = sofa::core::objectmodel::New<TLine_3>();
		thoracLines->setName("Lines");
		thoracLines->setContactStiffness(100);
		thoracLines->setSimulated(false);
		thoracLines->setContactFriction(0.01);
		thoracLines->setMoving(false);
		thoracsurf->addObject(thoracLines);

		//Point
		TPoint_3::SPtr thoracPoints = sofa::core::objectmodel::New<TPoint_3>();
		thoracPoints->setName("Points");
		thoracPoints->setContactStiffness(100);
		thoracPoints->setContactFriction(0.01);

		thoracPoints->setSimulated(false);
		thoracPoints->setMoving(false);
		thoracsurf->addObject(thoracPoints);
*/
		//------------------------------------------------------------------------------------------------------//
		//-------------------------------------Lungs-----------------------------------------------------------//
		//----------------------------------------------------------------------------------------------------//
/*		sofa::simulation::Node::SPtr lungsNode = groot.get()->createChild("Lungs");

		//Visual
		sofa::simulation::Node::SPtr lungsVisu = lungsNode.get()->createChild("Lungs Visu");

		OglModel::SPtr visualLungsModel = sofa::core::objectmodel::New<OglModel>();
		visualLungsModel->setName("Visual lungs");
		visualLungsModel->load(sofa::helper::system::DataRepository.getFile("mesh/diploma/lungs_meshlab.obj"), "", "");
		visualLungsModel->setColor("white");
		lungsVisu->addObject(visualLungsModel);

		//Collision
		sofa::simulation::Node::SPtr lungssurf = lungsNode.get()->createChild("Lungs Surf");

		//ObjLoader (coarser)
		sofa::component::loader::MeshObjLoader::SPtr lungsLoader = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		lungsLoader->setFilename(sofa::helper::system::DataRepository.getFile("mesh/diploma/lungs_meshlab.obj"));
		lungsLoader->load();
		lungssurf->addObject(lungsLoader);

		//Topology
		sofa::component::topology::MeshTopology::SPtr lungsTopology = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		lungsTopology->setSrc("", lungsLoader.get());
		lungssurf->addObject(lungsTopology);

		//Mechanical Obj
		MechanicalObject3::SPtr lungsMech = sofa::core::objectmodel::New<MechanicalObject3>();
		lungsMech->setName("instrumentCollisionState");
		lungsMech->setSrc("", lungsLoader.get());
		lungssurf->addObject(lungsMech);

		//Triangle
		TTriangle_3::SPtr lungsTriang = sofa::core::objectmodel::New<TTriangle_3>();
		lungsTriang->setName("Triangles");
		lungsTriang->setContactStiffness(100);
		lungsTriang->setSimulated(0);
		lungsTriang->setMoving(0);
		lungssurf->addObject(lungsTriang);

		//Line 
		TLine_3::SPtr lungsLines = sofa::core::objectmodel::New<TLine_3>();
		lungsLines->setName("Lines");
		lungsLines->setContactStiffness(100);
		lungsLines->setSimulated(0);
		lungsLines->setMoving(0);
		lungssurf->addObject(lungsLines);

		//Point
		TPoint_3::SPtr lungsPoints = sofa::core::objectmodel::New<TPoint_3>();
		lungsPoints->setName("Points");
		lungsPoints->setContactStiffness(100);
		lungsPoints->setSimulated(0);
		lungsPoints->setMoving(0);
		lungssurf->addObject(lungsPoints);

		
		//-------------------------------------Heart----------------------------------------------------------//

		sofa::simulation::Node::SPtr heartNode = groot.get()->createChild("Heart");

		//Visual
		sofa::simulation::Node::SPtr heartVisu = heartNode.get()->createChild("Heart Visu");

		OglModel::SPtr visualheartModel = sofa::core::objectmodel::New<OglModel>();
		visualheartModel->setName("Visual heart");
		visualheartModel->load(sofa::helper::system::DataRepository.getFile("mesh/diploma/heart_other_modified2.obj"), "", "");
		visualheartModel->setColor("white");
		heartVisu->addObject(visualheartModel);

		//Collision
		sofa::simulation::Node::SPtr heartsurf = lungsNode.get()->createChild("heart Surf");

		//ObjLoader (coarser)
		sofa::component::loader::MeshObjLoader::SPtr heartLoader = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		heartLoader->setFilename(sofa::helper::system::DataRepository.getFile("mesh/diploma/heart_other_modified2.obj"));
		heartLoader->load();
		heartsurf->addObject(heartLoader);

		//Topology
		sofa::component::topology::MeshTopology::SPtr heartTopology = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		heartTopology->setSrc("", heartLoader.get());
		heartsurf->addObject(heartTopology);

		//Mechanical Obj
		MechanicalObject3::SPtr heartMech = sofa::core::objectmodel::New<MechanicalObject3>();
		heartMech->setName("instrumentCollisionState");
		heartMech->setSrc("", heartLoader.get());
		heartsurf->addObject(heartMech);

		//Triangle
		TTriangle_3::SPtr heartTriang = sofa::core::objectmodel::New<TTriangle_3>();
		heartTriang->setName("Triangles");
		heartTriang->setContactStiffness(100);
		heartTriang->setSimulated(0);
		heartTriang->setMoving(0);
		heartsurf->addObject(heartTriang);

		//Line 
		TLine_3::SPtr heartLines = sofa::core::objectmodel::New<TLine_3>();
		heartLines->setName("Lines");
		heartLines->setContactStiffness(100);
		heartLines->setSimulated(0);
		heartLines->setMoving(0);
		heartsurf->addObject(heartLines);

		//Point
		TPoint_3::SPtr heartPoints = sofa::core::objectmodel::New<TPoint_3>();
		heartPoints->setName("Points");
		heartPoints->setContactStiffness(100);
		heartPoints->setSimulated(0);
		heartPoints->setMoving(0);
		heartsurf->addObject(heartPoints);



		//-----------------------------------Left Artrium----------------------------------------------------//


		sofa::simulation::Node::SPtr lartriumFEM = groot.get()->createChild("LeftArtirumFEM");

		EulerImplicitSolver::SPtr lartriumSolver = sofa::core::objectmodel::New<EulerImplicitSolver>();
		lartriumSolver->setName("default_19");
		lartriumFEM->addObject(lartriumSolver);


		CGLinearSolver3::SPtr lartriumlinearSolver = sofa::core::objectmodel::New<CGLinearSolver3>();
		lartriumlinearSolver->setName("linear solver");
		lartriumlinearSolver->f_maxIter.setValue(25);
		lartriumlinearSolver->f_tolerance.setValue(1e-9);
		lartriumlinearSolver->f_smallDenominatorThreshold.setValue(1e-9);
		lartriumFEM->addObject(lartriumlinearSolver);

		sofa::component::loader::MeshGmshLoader::SPtr lartriummeshloader = sofa::core::objectmodel::New<sofa::component::loader::MeshGmshLoader>();
		lartriummeshloader->setName("loaderkalo");
		lartriummeshloader->setFilename(sofa::helper::system::DataRepository.getFile("mesh/diploma/l_art_modified.msh"));
		lartriummeshloader->load();
		lartriumFEM->addObject(lartriummeshloader);

		sofa::component::topology::TetrahedronSetTopologyContainer::SPtr lartriumcontainer = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyContainer>();
		lartriumcontainer->setSrc("", lartriummeshloader.get());
		lartriumFEM->addObject(lartriumcontainer);

		MechanicalObject3::SPtr lartriumdofs = sofa::core::objectmodel::New<MechanicalObject3>();
		lartriumdofs->setName("Dofs");
		lartriumdofs->setSrc("", lartriummeshloader.get());
		lartriumFEM->addObject(lartriumdofs);

		sofa::component::mass::DiagonalMass<sofa::defaulttype::Vec3dTypes, double>::SPtr lartriumdiagMass = sofa::core::objectmodel::New<sofa::component::mass::DiagonalMass<sofa::defaulttype::Vec3dTypes, double>>();
		lartriumdiagMass->setName("Computed using mass");
		lartriumdiagMass->setMassDensity(1);
		lartriumFEM->addObject(lartriumdiagMass);

		sofa::component::forcefield::TetrahedralCorotationalFEMForceField<sofa::defaulttype::Vec3Types>::SPtr lartriumcoorotFEM = sofa::core::objectmodel::New<sofa::component::forcefield::TetrahedralCorotationalFEMForceField<sofa::defaulttype::Vec3Types>>();
		lartriumcoorotFEM->setName("Co-rotational Forcefield");
		lartriumcoorotFEM->f_method.setValue("large");
		lartriumcoorotFEM->setPoissonRatio(0.3);
		lartriumcoorotFEM->setYoungModulus(100);
		lartriumcoorotFEM->setComputeGlobalMatrix(false);
		lartriumFEM->addObject(lartriumcoorotFEM);


		//Visual 


		//Collision


*/

/*		// Lungs

		/*	sofa::simulation::Node::SPtr lungsFEM = groot.get()->createChild("LungsFEM");
			lungsFEM->setGravity(sofa::defaulttype::Vector3(0, -9.81, 0));


			EulerImplicitSolver::SPtr solverLungs = sofa::core::objectmodel::New<EulerImplicitSolver>();
			solverLungs->setName("default_19");
			solverLungs->f_rayleighStiffness.setValue(0.1);
			solverLungs->f_rayleighMass.setValue(0.1);
			groot->addObject(solverLungs);

			CGLinearSolver3::SPtr linearSolverLungs = sofa::core::objectmodel::New<CGLinearSolver3>();
			linearSolverLungs->setName("linear solver");
			linearSolverLungs->f_maxIter.setValue(25);
			linearSolverLungs->f_tolerance.setValue(1e-9);
			linearSolverLungs->f_smallDenominatorThreshold.setValue(1e-9);
			groot->addObject(linearSolverLungs);

			sofa::component::loader::MeshGmshLoader::SPtr meshLoaderLungs = sofa::core::objectmodel::New<sofa::component::loader::MeshGmshLoader>();
			meshLoaderLungs->setFilename(sofa::helper::system::DataRepository.getFile("mesh/heart_fill5"));
			meshLoaderLungs->load();
			lungsFEM->addObject(meshLoaderLungs);

			sofa::component::topology::TetrahedronSetTopologyContainer::SPtr containerLungs = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyContainer>();
			containerLungs->setSrc("", meshLoaderLungs.get());
			lungsFEM->addObject(containerLungs);


			MechanicalObject3::SPtr dofsLungs = sofa::core::objectmodel::New<MechanicalObject3>();
			dofsLungs->setName("Dofs");
			dofsLungs->setSrc("", meshLoaderLungs.get());
			dofsLungs->setTranslation(-10, -5, 0);
			dofsLungs->setScale(10, 2, 2);
			lungsFEM->addObject(dofsLungs);


			UniformMass3::SPtr uniMassFEMLungs = sofa::core::objectmodel::New<UniformMass3>();
			uniMassFEMLungs->setTotalMass(1);
			uniMassFEMLungs->setName("Mass");
			lungsFEM->addObject(uniMassFEMLungs);

			sofa::component::forcefield::TetrahedralCorotationalFEMForceField<sofa::defaulttype::Vec3Types>::SPtr coorotFEMLungs = sofa::core::objectmodel::New<sofa::component::forcefield::TetrahedralCorotationalFEMForceField<sofa::defaulttype::Vec3Types>>();
			coorotFEMLungs->setName("Co-rotational Forcefield");
			coorotFEMLungs->f_method.setValue("large");
			coorotFEMLungs->setPoissonRatio(0.3);
			coorotFEMLungs->setYoungModulus(6000);
			coorotFEMLungs->setComputeGlobalMatrix(false);
			lungsFEM->addObject(coorotFEMLungs);

			//Visu
			sofa::simulation::Node::SPtr lungsvisu = lungsFEM.get()->createChild("LiverVisu");

			OglModel::SPtr visualLungs = sofa::core::objectmodel::New<OglModel>();
			visualLungs->setName("Visual");
			visualLungs->load(sofa::helper::system::DataRepository.getFile("mesh/heart_fill.stl"), "","textures/greeny.bmp" );
			visualLungs->setTranslation(-0.5, -5, 0);
			visualLungs->setScale(10, 2, 2);
			lungsvisu->addObject(visualLungs);

			BarycentricMapping_Vec3d_to_ExteVec3f::SPtr barmappingLungs = sofa::core::objectmodel::New<BarycentricMapping_Vec3d_to_ExteVec3f>();
			barmappingLungs->setName("Visual Mapping");
			barmappingLungs->setModels(dofsLungs.get(), visualLungs.get());
			lungsvisu->addObject(barmappingLungs);

			//Surf
			sofa::simulation::Node::SPtr lungssurf = lungsFEM.get()->createChild("LungsSurf");



			//Triangle
			TTriangle_3::SPtr triangles_col_lungs = sofa::core::objectmodel::New<TTriangle_3>();
			triangles_col_lungs->setName("Triangles");
			//triangles_col->addTag(sofa::core::objectmodel::Tag("CarvingSurface"));
			lungssurf->addObject(triangles_col_lungs);

			//Point
			TPoint_3::SPtr points_col_lungs = sofa::core::objectmodel::New<TPoint_3>();
			points_col_lungs->setName("Points");
			//points_col->addTag(sofa::core::objectmodel::Tag("CarvingSurface"));

			lungssurf->addObject(points_col_lungs);
			*/

		//--------------------------Liver---------------------------------------------------------------//
		sofa::simulation::Node::SPtr liverFEM = groot.get()->createChild("LiverTetraFEM");
		liverFEM->setGravity(sofa::defaulttype::Vector3(0, -9.81, 0));

		EulerImplicitSolver::SPtr solver = sofa::core::objectmodel::New<EulerImplicitSolver>();
		solver->setName("default_19");
		solver->f_rayleighStiffness.setValue(0.1);
		solver->f_rayleighMass.setValue(0.1);
		liverFEM->addObject(solver);


		CGLinearSolver3::SPtr linearSolver = sofa::core::objectmodel::New<CGLinearSolver3>();
		linearSolver->setName("linear solver");
		linearSolver->f_maxIter.setValue(25);
		linearSolver->f_tolerance.setValue(1e-9);
		linearSolver->f_smallDenominatorThreshold.setValue(1e-9);
		liverFEM->addObject(linearSolver);

		sofa::component::loader::MeshGmshLoader::SPtr meshloader = sofa::core::objectmodel::New<sofa::component::loader::MeshGmshLoader>();
		meshloader->setName("loaderkalo");
		meshloader->setFilename(sofa::helper::system::DataRepository.getFile("mesh/liver.msh"));
		meshloader->load();
		liverFEM->addObject(meshloader);

		sofa::component::topology::TetrahedronSetTopologyContainer::SPtr container = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyContainer>();
		container->setSrc("", meshloader.get());
		liverFEM->addObject(container);

		/*sofa::component::topology::TetrahedronSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>::SPtr topoalgo = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>>();
		topoalgo->setName("Topo_Algo");
		topoalgo->setSrc("", meshloader.get());
		liverFEM->addObject(topoalgo);

		sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr geomalgo = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>();
		geomalgo->setName("Geom_Algo");
		geomalgo->setSrc("", meshloader.get());
		liverFEM->addObject(geomalgo);

		sofa::component::topology::TetrahedronSetTopologyModifier::SPtr modifier = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetTopologyModifier>();
		modifier->setName("Modifier");
		modifier->setSrc("", meshloader.get());
		liverFEM->addObject(modifier);

		/*sofa::component::topology::MeshTopology::SPtr mesh2 = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		mesh2->setName("mesh");
		mesh2->setSrc("", meshloader.get());
		liverFEM->addObject(mesh2);*/

		MechanicalObject3::SPtr dofs = sofa::core::objectmodel::New<MechanicalObject3>();
		dofs->setName("Dofs");
		dofs->setSrc("", meshloader.get());
		liverFEM->addObject(dofs);

		sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr alogirthms = sofa::core::objectmodel::New<sofa::component::topology::TetrahedronSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>();
		alogirthms->setName("GeomAlgo");
		liverFEM->addObject(alogirthms);

		//Diagonal Mass
		sofa::component::mass::DiagonalMass<sofa::defaulttype::Vec3dTypes, double>::SPtr diagMass = sofa::core::objectmodel::New<sofa::component::mass::DiagonalMass<sofa::defaulttype::Vec3dTypes, double>>();
		diagMass->setName("Computed using mass");
		diagMass->setMassDensity(0.01);
		liverFEM->addObject(diagMass);


		/*UniformMass3::SPtr uniMassFEM = sofa::core::objectmodel::New<UniformMass3>();
		uniMassFEM->setTotalMass(1);
		uniMassFEM->setName("Mass");
		liverFEM->addObject(uniMassFEM);*/

		/*sofa::component::forcefield::TetrahedronFEMForceField<sofa::defaulttype::Vec3Types>::SPtr coorot = sofa::core::objectmodel::New<sofa::component::forcefield::TetrahedronFEMForceField<sofa::defaulttype::Vec3Types>>();
		coorot->setName("FEM");
		coorot->f_method.setValue("large");
		coorot->setPoissonRatio(0.45);
		coorot->setYoungModulus(50);
		//coorot->setComputeGlobalMatrix(0);
		liverFEM->addObject(coorot);
		*/

		//Co-rotational Forcefield
		sofa::component::forcefield::TetrahedralCorotationalFEMForceField<sofa::defaulttype::Vec3Types>::SPtr coorotFEM = sofa::core::objectmodel::New<sofa::component::forcefield::TetrahedralCorotationalFEMForceField<sofa::defaulttype::Vec3Types>>();
		coorotFEM->setName("Co-rotational Forcefield");
		coorotFEM->f_method.setValue("large");
		coorotFEM->setPoissonRatio(0.3);
		coorotFEM->setYoungModulus(6000);
		coorotFEM->setComputeGlobalMatrix(false);
		liverFEM->addObject(coorotFEM);

		//*************PrecomputedConstraintCorrection*******************/
		sofa::component::constraintset::PrecomputedConstraintCorrection<sofa::defaulttype::Vec3dTypes>::SPtr preComputed= sofa::core::objectmodel::New<sofa::component::constraintset::PrecomputedConstraintCorrection<sofa::defaulttype::Vec3dTypes>>();
		preComputed->recompute.setValue(true);
		liverFEM->addObject(preComputed);

		//Box Roi
		/*sofa::component::engine::BoxROI<sofa::defaulttype::Vec<6, SReal>>::SPtr BoxRoi= sofa::core::objectmodel::New<sofa::component::engine::BoxROI<sofa::defaulttype::Vec<6, SReal>>>();
		BoxRoi->setName("ROI1");
		BoxRoi->p_drawBoxes.setValue(true);
		BoxRoi->boxes.setValue(sofa::helper::vector<sofa::defaulttype::Vec<6, SReal>> ((sofa::defaulttype::Vec<6, SReal>(-1, -1, -1, -1, -1, -1)));
		*/


		FixedConstraint3::SPtr constraint = sofa::core::objectmodel::New<FixedConstraint3>();
		//for (int i = 0; i < 100; i++) {
			constraint->addConstraint(3);
			constraint->addConstraint(39);
			constraint->addConstraint(64);

		//}
		liverFEM->addObject(constraint);


		//Surf
		sofa::simulation::Node::SPtr liversurf = liverFEM.get()->createChild("LiverSurf");
		liversurf->setGravity(sofa::defaulttype::Vector3(0, 0, -9));

		/*sofa::component::topology::TriangleSetTopologyContainer::SPtr container2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetTopologyContainer>();
		liversurf->addObject(container2);


		sofa::component::topology::TriangleSetTopologyModifier::SPtr mod2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetTopologyModifier>();
		liversurf->addObject(mod2);

		sofa::component::topology::TriangleSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>::SPtr topalgo2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetTopologyAlgorithms<sofa::defaulttype::Vec3Types>>();
		liversurf->addObject(topalgo2);

		sofa::component::topology::TriangleSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>::SPtr geomalgo2 = sofa::core::objectmodel::New<sofa::component::topology::TriangleSetGeometryAlgorithms<sofa::defaulttype::Vec3Types>>();
		liversurf->addObject(geomalgo2);


		sofa::component::topology::Tetra2TriangleTopologicalMapping::SPtr tetra2trian= sofa::core::objectmodel::New<sofa::component::topology::Tetra2TriangleTopologicalMapping>();
		tetra2trian->setTopologies(container->toBaseMeshTopology(), container2->toBaseMeshTopology());
		liversurf->addObject(tetra2trian);

		*/
		//Triangle
		TTriangle_3::SPtr triangles_col = sofa::core::objectmodel::New<TTriangle_3>();
		triangles_col->setName("Triangles");
		//triangles_col->addTag(sofa::core::objectmodel::Tag("CarvingSurface"));
		liversurf->addObject(triangles_col);

		//Point
		TPoint_3::SPtr points_col = sofa::core::objectmodel::New<TPoint_3>();
		points_col->setName("Points");
		//points_col->addTag(sofa::core::objectmodel::Tag("CarvingSurface"));

		liversurf->addObject(points_col);

			//carv->m_surfaceCollisionModels.push_back(triangles_col->toCollisionModel());

			//carv->m_surfaceCollisionModels.push_back(points_col->toCollisionModel());

		//groot->addObject(carv);

/*		sofa::simulation::Node::SPtr instrum = groot.get()->createChild("Instrument");

		

		MechanicalObjectRigid3::SPtr dofRigidInstrum = sofa::core::objectmodel::New<MechanicalObjectRigid3>();
		dofRigidInstrum->setName("instrumentState");
		dofRigidInstrum->x.setParent(geomDriver->d_posDevice.getData());

		instrum->addObject(dofRigidInstrum);

		UniformMassRigid3::SPtr uniMassCarvInstrum = sofa::core::objectmodel::New<UniformMassRigid3>();
		uniMassCarvInstrum->setTotalMass(0.05);
		uniMassCarvInstrum->setName("Mass");
		instrum->addObject(uniMassCarvInstrum);

		/*		sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Rigid3dTypes>::SPtr lcpContr = sofa::core::objectmodel::New<sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Rigid3dTypes>>();
		lcpContr->f_activate.setValue(true);
		lcpContr->forceCoef.setValue(0.001);
		instrum->addObject(lcpContr);

		sofa::component::constraintset::UncoupledConstraintCorrection<sofa::defaulttype::Rigid3dTypes>::SPtr constrUncoup=
		sofa::core::objectmodel::New<sofa::component::constraintset::UncoupledConstraintCorrection<sofa::defaulttype::Rigid3dTypes>>();

		instrum->addObject(constrUncoup);*/

		/*sofa::simulation::Node::SPtr collModelInstrument = instrum.get()->createChild("CollisionModel");


		sofa::component::loader::MeshObjLoader::SPtr loaderColl = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderColl->setName("loader");
		loaderColl->setFilename(sofa::helper::system::DataRepository.getFile("mesh/dental_instrument_centerline.obj"));
		loaderColl->load();
		collModelInstrument->addObject(loaderColl);

		sofa::component::topology::MeshTopology::SPtr meshColl = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshColl->setSrc("", loaderColl.get());
		collModelInstrument->addObject(meshColl);

		MechanicalObject3::SPtr mechColl = sofa::core::objectmodel::New<MechanicalObject3>();
		mechColl->setName("instrumentCollisionState");
		mechColl->setSrc("", loaderColl.get());
		mechColl->setRotation(0, -180, -90);
		mechColl->setTranslation(-0.3, 5, 3.5);
		collModelInstrument->addObject(mechColl);

		//Lines
		TLine_3::SPtr line_coll_instr = sofa::core::objectmodel::New<TLine_3>();
		line_coll_instr->setName("Lines");
		line_coll_instr->setContactStiffness(10.0);
		line_coll_instr->setContactFriction(0.01);
		//line_coll_instr->addTag(sofa::core::objectmodel::Tag("CarvingTool"));
		collModelInstrument->addObject(line_coll_instr);

		//Points
		TPoint_3::SPtr points_col_instr = sofa::core::objectmodel::New<TPoint_3>();
		points_col_instr->setName("Points");
		points_col_instr->setContactStiffness(10.0);
		points_col_instr->setContactFriction(0.01);
		//points_col_instr->addTag(sofa::core::objectmodel::Tag("CarvingTool"));
		collModelInstrument->addObject(points_col_instr);

		//RigidMapping
		RigidMapping_Rigid_to_Vec3d::SPtr rigidMapInstrCol = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMapInstrCol->setName("VM Mapping");
		rigidMapInstrCol->setModels(dofRigidInstrum.get(), mechColl.get());
		collModelInstrument->addObject(rigidMapInstrCol);*/

/*		sofa::simulation::Node::SPtr modl = groot.get()->createChild("CarveLEM");

		MechanicalObject3::SPtr sphm= sofa::core::objectmodel::New<MechanicalObject3>();
		sphm->setName("particles");
		sphm->x.setParent(geomDriver->d_posDevice.getData());
		modl->addObject(sphm);

		UniformMass3::SPtr uniMasssph = sofa::core::objectmodel::New<UniformMass3>();
		uniMasssph->setTotalMass(1);
		uniMasssph->setName("Mass");
		modl->addObject(uniMasssph);

		//geomagic test	
		sofa::component::collision::SphereModel::SPtr sphereModelCarv = sofa::core::objectmodel::New<sofa::component::collision::SphereModel>();
		sphereModelCarv->setName("CollisionModel");
		sphereModelCarv->defaultRadius.setValue(0.02);
		sphereModelCarv->addTag(sofa::core::objectmodel::Tag("CarvingTool"));
		modl->addObject(sphereModelCarv);

		//carv->m_surfaceCollisionModels.push_back(triangles_col.get());
		//carv->m_surfaceCollisionModels.push_back(points_col.get());
		//carv->d_surfaceModelPath.setValue("@LiverTetraFEM/LiverSurf/Triangles");
		//groot->addObject(carv); 

		/*carv->f_modelTool.setValue(particle->getName());
		carv->f_modelSurface.setValue(dofs->getName());
		groot->addObject(carv);*/

			//Visu
/*		sofa::simulation::Node::SPtr livervisu = liverFEM.get()->createChild("LiverVisu");

		OglModel::SPtr visual = sofa::core::objectmodel::New<OglModel>();
		visual->setName("Visual");
		/*sofa::helper::types::Material material;
		material.ambient = sofa::helper::types::RGBAColor(1, 1, 1, 1.0);
		material.diffuse = sofa::helper::types::RGBAColor(0, 1, 0, 0.75);
		material.specular = sofa::helper::types::RGBAColor(1, 1, 0, 1.0);
		material.emissive = sofa::helper::types::RGBAColor(1, 1, 0.0, 1);
		material.shininess = 100;

		visual->material.setValue(material);*/
/*		visual->load(sofa::helper::system::DataRepository.getFile("mesh/liver-smooth.obj"), "", "textures/liver-texture-square.png");

		livervisu->addObject(visual);

		BarycentricMapping_Vec3d_to_ExteVec3f::SPtr barmapping = sofa::core::objectmodel::New<BarycentricMapping_Vec3d_to_ExteVec3f>();
		barmapping->setName("Visual Mapping");
		barmapping->setModels(dofs.get(), visual.get());
		livervisu->addObject(barmapping);




		/*sofa::component::mapping::IdentityMapping
			<  sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes>::SPtr identmap
			= sofa::core::objectmodel::New<sofa::component::mapping::IdentityMapping  < sofa::defaulttype::Vec3dTypes, sofa::defaulttype::ExtVec3fTypes>>();
		identmap->setModels(dofs.get(), visual.get());

		livervisu->addObject(identmap);*/

		/*----------------------Omni-------------------------------------------------------*/

/*		sofa::simulation::Node::SPtr carvingElement = groot.get()->createChild("Omni");

		MechanicalObjectRigid3::SPtr dofRigid = sofa::core::objectmodel::New<MechanicalObjectRigid3>();
		dofRigid->setName("DofsRigid");
		dofRigid->x.setParent(geomDriver->d_posDevice.getData());
		carvingElement->addObject(dofRigid);

		//MechanicalStateController
		sofa::component::controller::MechanicalStateController<sofa::defaulttype::Rigid3dTypes>::SPtr controller = sofa::core::objectmodel::New<sofa::component::controller::MechanicalStateController<sofa::defaulttype::Rigid3dTypes>>();
		controller->f_listening.setValue(true);
		controller->setMainDirection(sofa::defaulttype::Vec3d(-1, 0, 0));
		controller->handleEventTriggersUpdate.setValue(true);
		carvingElement->addObject(controller);


		sofa::simulation::Node::SPtr visuAvatar = carvingElement.get()->createChild("VisuAvatar");
		visuAvatar->setActive(true);
		OglModel::SPtr visual1 = sofa::core::objectmodel::New<OglModel>();
		visual1->setName("Visual");
		visual1->load(sofa::helper::system::DataRepository.getFile("mesh/sphere.obj"), "", "");
		visual1->setColor("gray");
		visual1->setScale(0.1, 0.1, 0.1);
		visuAvatar->addObject(visual1);

		//RigidMapping
		RigidMapping_Rigid_to_Extevec3F::SPtr rigidMap2 = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Extevec3F>();
		rigidMap2->setName("VM Mapping");
		rigidMap2->setModels(dofRigid.get(), visual1.get());
		rigidMap2->index.setValue(0);
		visuAvatar->addObject(rigidMap2);



		sofa::simulation::Node::SPtr refModel = carvingElement.get()->createChild("RefModel");

		sofa::component::loader::MeshObjLoader::SPtr loaderFixed = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderFixed->setFilename(sofa::helper::system::DataRepository.getFile("mesh/dental_instrument_centerline.obj"));
		loaderFixed->load();
		refModel->addObject(loaderFixed);

		sofa::component::topology::MeshTopology::SPtr meshTorusFEM = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshTorusFEM->setSrc("", loaderFixed.get());
		refModel->addObject(meshTorusFEM);

		MechanicalObject3::SPtr instrumentColState = sofa::core::objectmodel::New<MechanicalObject3>();
		instrumentColState->setName("instrumentCollisionState");
		instrumentColState->setSrc("", loaderFixed.get());
		instrumentColState->setRotation(0, -180, -90);
		instrumentColState->setTranslation(-0.3, 0, 3.5);
		refModel->addObject(instrumentColState);

	

		RigidMapping_Rigid_to_Vec3d::SPtr rigidMap3 = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMap3->setModels(dofRigid.get(), instrumentColState.get());
		rigidMap3->index.setValue(0);
		refModel->addObject(rigidMap3);


		sofa::simulation::Node::SPtr refModelRight = carvingElement.get()->createChild("RefModelRight");

		sofa::component::loader::MeshObjLoader::SPtr loaderFixedright = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderFixedright->setFilename(sofa::helper::system::DataRepository.getFile("mesh/dental_instrument_centerline.obj"));
		loaderFixedright->load();
		refModelRight->addObject(loaderFixedright);

		sofa::component::topology::MeshTopology::SPtr meshTorusFEMRight = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshTorusFEMRight->setSrc("", loaderFixedright.get());
		refModelRight->addObject(meshTorusFEMRight);

		MechanicalObject3::SPtr instrumentColStateRight = sofa::core::objectmodel::New<MechanicalObject3>();
		instrumentColStateRight->setName("instrumentCollisionState");
		instrumentColStateRight->setSrc("", loaderFixedright.get());
		instrumentColStateRight->setRotation(0, -180, -90);
		instrumentColStateRight->setTranslation(-0.3, 0.5, 3.5);
		refModelRight->addObject(instrumentColStateRight);

		RigidMapping_Rigid_to_Vec3d::SPtr rigidMap3right = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMap3right->setModels(dofRigid.get(), instrumentColStateRight.get());

		refModelRight->addObject(rigidMap3right);

		sofa::simulation::Node::SPtr refModelleft = carvingElement.get()->createChild("RefModelLeft");

		sofa::component::loader::MeshObjLoader::SPtr loaderFixedleft = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderFixedleft->setFilename(sofa::helper::system::DataRepository.getFile("mesh/dental_instrument_centerline.obj"));
		loaderFixedleft->load();
		refModelleft->addObject(loaderFixedleft);

		sofa::component::topology::MeshTopology::SPtr meshTorusFEMleft = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshTorusFEMleft->setSrc("", loaderFixedleft.get());
		refModelleft->addObject(meshTorusFEMleft);

		MechanicalObject3::SPtr instrumentColStateleft = sofa::core::objectmodel::New<MechanicalObject3>();
		instrumentColStateleft->setName("instrumentCollisionState");
		instrumentColStateleft->setSrc("", loaderFixedleft.get());
		instrumentColStateleft->setRotation(0, -180, -90);
		instrumentColStateleft->setTranslation(-0.3, -0.5, 3.5);
		refModelleft->addObject(instrumentColStateleft);

		RigidMapping_Rigid_to_Vec3d::SPtr rigidMap3left = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMap3left->setModels(dofRigid.get(), instrumentColStateleft.get());
		refModelleft->addObject(rigidMap3left);

	
		/*----------------------Instrument-------------------------------------------------------*/
/*		sofa::simulation::Node::SPtr instrum = groot.get()->createChild("Instrument");
		
		EulerImplicitSolver::SPtr solverInstrum = sofa::core::objectmodel::New<EulerImplicitSolver>();
		solverInstrum->setName("ODE solver");
		solverInstrum->f_rayleighStiffness.setValue(0.05);
		solverInstrum->f_rayleighMass.setValue(1.0);
		instrum->addObject(solverInstrum);
	

		CGLinearSolver3::SPtr linearSolverIntrum = sofa::core::objectmodel::New<CGLinearSolver3>();
		linearSolverIntrum->setName("linear solver");
		linearSolverIntrum->f_maxIter.setValue(25);
		linearSolverIntrum->f_tolerance.setValue(1e-10);
		linearSolverIntrum->f_smallDenominatorThreshold.setValue(10e-10);
		instrum->addObject(linearSolverIntrum);

		MechanicalObjectRigid3::SPtr dofRigidInstrum = sofa::core::objectmodel::New<MechanicalObjectRigid3>();
		dofRigidInstrum->setName("instrumentState");
		dofRigidInstrum->x.setParent(geomDriver->d_posDevice.getData());
		/*
		//to 5 einai to position
		//std::cout<<dofRigidInstrum->getDataFields()[0]->getName();
		//for (int i = 0; i < dofRigidInstrum->getDataFields().size(); i++) {
		//	std::cout << "\n" << i;
		//	std::cout<<dofRigidInstrum->getDataFields()[i]->getName();
		//}
		//sofa::modeling::setDataLink(geomDriver->d_posDevice.getData(), dofRigidInstrum->getDataFields()[5]); */
/*		instrum->addObject(dofRigidInstrum);

		UniformMassRigid3::SPtr uniMassCarvInstrum = sofa::core::objectmodel::New<UniformMassRigid3>();
		uniMassCarvInstrum->setTotalMass(0.05);
		uniMassCarvInstrum->setName("Mass");
		instrum->addObject(uniMassCarvInstrum);

/*		sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Rigid3dTypes>::SPtr lcpContr = sofa::core::objectmodel::New<sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Rigid3dTypes>>();
		lcpContr->f_activate.setValue(true);
		lcpContr->forceCoef.setValue(0.001);
		instrum->addObject(lcpContr);

		sofa::component::constraintset::UncoupledConstraintCorrection<sofa::defaulttype::Rigid3dTypes>::SPtr constrUncoup=
			sofa::core::objectmodel::New<sofa::component::constraintset::UncoupledConstraintCorrection<sofa::defaulttype::Rigid3dTypes>>();
			
		instrum->addObject(constrUncoup);*/
		

/*		sofa::simulation::Node::SPtr visuModelInstrument = instrum.get()->createChild("VisualModel");
		
		OglModel::SPtr visualInstr = sofa::core::objectmodel::New<OglModel>();
		visualInstr->setName("InstrumentVisualModel");
		visualInstr->load(sofa::helper::system::DataRepository.getFile("mesh/dental_instrument.obj"), "", "");
		visualInstr->setColor(1.f,0.2f,0.2f,1.f);
		visualInstr->m_translation.setValue(sofa::defaulttype::Vec3f(-0.3f, 0.f, 3.5f));
		visualInstr->m_rotation.setValue(sofa::defaulttype::Vec3f(0.f, -180.f, -90.f));

		visuModelInstrument->addObject(visualInstr);
		
		//RigidMapping
		RigidMapping_Rigid_to_Extevec3F::SPtr rigidMapInstr = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Extevec3F>();
		rigidMapInstr->setName("VM Mapping");
		rigidMapInstr->setModels(dofRigidInstrum.get(), visualInstr.get());
		visuModelInstrument->addObject(rigidMapInstr);

*/

/*		sofa::simulation::Node::SPtr collModelInstrument = instrum.get()->createChild("CollisionModel");


		sofa::component::loader::MeshObjLoader::SPtr loaderColl = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderColl->setName("loader");
		loaderColl->setFilename(sofa::helper::system::DataRepository.getFile("mesh/dental_instrument_centerline.obj"));
		loaderColl->load();
		collModelInstrument->addObject(loaderColl);

		sofa::component::topology::MeshTopology::SPtr meshColl = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshColl->setSrc("", loaderColl.get());
		collModelInstrument->addObject(meshColl);

		MechanicalObject3::SPtr mechColl = sofa::core::objectmodel::New<MechanicalObject3>();
		mechColl->setName("instrumentCollisionState");
		mechColl->setSrc("", loaderColl.get());
		mechColl->setRotation(0, -180, -90);
		mechColl->setTranslation(-0.3, 0, 3.5);
		collModelInstrument->addObject(mechColl);

		//Lines
		TLine_3::SPtr line_coll_instr = sofa::core::objectmodel::New<TLine_3>();
		line_coll_instr->setName("Lines");
		line_coll_instr->setContactStiffness(10.0);
		line_coll_instr->setContactFriction(0.01);
		line_coll_instr->addTag(sofa::core::objectmodel::Tag("CarvingTool"));
		collModelInstrument->addObject(line_coll_instr);

		//Points
		TPoint_3::SPtr points_col_instr = sofa::core::objectmodel::New<TPoint_3>();
		points_col_instr->setName("Points");
		points_col_instr->setContactStiffness(10.0);
		points_col_instr->setContactFriction(0.01);
		points_col_instr->addTag(sofa::core::objectmodel::Tag("CarvingTool"));
		collModelInstrument->addObject(points_col_instr);

		//RigidMapping
		RigidMapping_Rigid_to_Vec3d::SPtr rigidMapInstrCol = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMapInstrCol->setName("VM Mapping");
		rigidMapInstrCol->setModels(dofRigidInstrum.get(), mechColl.get());
		collModelInstrument->addObject(rigidMapInstrCol);

	/*	//geomagic test	
		sofa::component::collision::SphereModel::SPtr sphereModelCarv = sofa::core::objectmodel::New<sofa::component::collision::SphereModel>();
		sphereModelCarv->setName("CollisionModel");
		sphereModelCarv->defaultRadius.setValue(0.02);
		//sphereModelCarv->addTag(sofa::core::objectmodel::Tag("CarvingTool"));
		groot->addObject(sphereModelCarv);
		///////////////////////////////////////*/
		//Right

/*		sofa::simulation::Node::SPtr collModelInstrumentRight = instrum.get()->createChild("RefModelRight");


		sofa::component::loader::MeshObjLoader::SPtr loaderCollRight = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderCollRight->setFilename(sofa::helper::system::DataRepository.getFile("mesh/dental_instrument_centerline.obj"));
		loaderCollRight->load();
		collModelInstrumentRight->addObject(loaderCollRight);

		sofa::component::topology::MeshTopology::SPtr meshCollRight = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshCollRight->setSrc("", loaderCollRight.get());
		collModelInstrumentRight->addObject(meshCollRight);

		MechanicalObject3::SPtr mechCollRight = sofa::core::objectmodel::New<MechanicalObject3>();
		mechCollRight->setName("instrumentCollisionState");
		mechCollRight->setSrc("", loaderCollRight.get());
		mechCollRight->setRotation(0, -180, -90);
		mechCollRight->setTranslation(-0.3, 0.5, 3.5);
		collModelInstrumentRight->addObject(mechCollRight);



		//RigidMapping
		RigidMapping_Rigid_to_Vec3d::SPtr rigidMapInstrColRight = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMapInstrColRight->setName("VM Mapping");
		rigidMapInstrColRight->setModels(dofRigidInstrum.get(), mechCollRight.get());
		collModelInstrumentRight->addObject(rigidMapInstrColRight);

		//Left 

		sofa::simulation::Node::SPtr collModelInstrumentLeft = instrum.get()->createChild("RefModelLeft");


		sofa::component::loader::MeshObjLoader::SPtr loaderCollLeft = sofa::core::objectmodel::New<sofa::component::loader::MeshObjLoader>();
		loaderCollLeft->setFilename(sofa::helper::system::DataRepository.getFile("mesh/dental_instrument_centerline.obj"));
		loaderCollLeft->load();
		collModelInstrumentLeft->addObject(loaderCollLeft);

		sofa::component::topology::MeshTopology::SPtr meshCollLeft = sofa::core::objectmodel::New<sofa::component::topology::MeshTopology>();
		meshCollLeft->setSrc("", loaderCollLeft.get());
		collModelInstrumentLeft->addObject(meshCollLeft);

		MechanicalObject3::SPtr mechCollLeft = sofa::core::objectmodel::New<MechanicalObject3>();
		mechCollLeft->setName("instrumentCollisionState");
		mechCollLeft->setSrc("", loaderCollLeft.get());
		mechCollLeft->setRotation(0, -180, -90);
		mechCollLeft->setTranslation(-0.3, -0.5, 3.5);
		collModelInstrumentLeft->addObject(mechCollLeft);



		//RigidMapping
		RigidMapping_Rigid_to_Vec3d::SPtr rigidMapInstrColLeft = sofa::core::objectmodel::New<RigidMapping_Rigid_to_Vec3d>();
		rigidMapInstrColLeft->setName("VM Mapping");
		rigidMapInstrColLeft->setModels(dofRigidInstrum.get(), mechCollLeft.get());
		collModelInstrumentLeft->addObject(rigidMapInstrColLeft);



		//VectorSpringForceField
		sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>::SPtr vSpringForc =
			sofa::core::objectmodel::New<sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>>();
		vSpringForc->setPathObject1("@Omni/RefModel/instrumentCollisionState");
		vSpringForc->setPathObject2("@Instrument/CollisionModel/instrumentCollisionState");
		vSpringForc->m_stiffness.setValue(10);
		vSpringForc->m_viscosity.setValue(0);
		instrum->addObject(vSpringForc);

		sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>::SPtr vSpringForc2 =
			sofa::core::objectmodel::New<sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>>();

		vSpringForc2->setPathObject1("@Omni/RefModelRight/instrumentCollisionState");
		vSpringForc2->setPathObject2("@Instrument/RefModelRight/instrumentCollisionState");
		vSpringForc2->m_stiffness.setValue(10);
		vSpringForc2->m_viscosity.setValue(0);
		instrum->addObject(vSpringForc2);

		sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>::SPtr vSpringForc3 =
			sofa::core::objectmodel::New<sofa::component::interactionforcefield::VectorSpringForceField<sofa::defaulttype::Vec3dTypes>>();

		vSpringForc3->setPathObject1("@Omni/RefModelLeft/instrumentCollisionState");
		vSpringForc3->setPathObject2("@Instrument/RefModelLeft/instrumentCollisionState");
		vSpringForc3->m_stiffness.setValue(10);
		vSpringForc3->m_viscosity.setValue(0);
		instrum->addObject(vSpringForc3);
	


		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/*______Continue RunSofa______________*/



		if (computationTimeAtBegin)
		{
			sofa::helper::AdvancedTimer::setEnabled("Init", true);
			sofa::helper::AdvancedTimer::setInterval("Init", 1);
			sofa::helper::AdvancedTimer::setOutputType("Init", computationTimeOutputType);
			sofa::helper::AdvancedTimer::begin("Init");
		}

		sofa::simulation::getSimulation()->init(groot.get());
		if (computationTimeAtBegin)
		{
			msg_info("") << sofa::helper::AdvancedTimer::end("Init", groot.get());
		}

		GUIManager::SetScene(groot);

		if (startAnim)
			groot->setAnimate(true);

		if (printFactory)
		{
			msg_info("") << "////////// FACTORY //////////";
			sofa::helper::printFactoryLog();
			msg_info("") << "//////// END FACTORY ////////";
		}

		if (computationTimeSampling > 0)
		{
			sofa::helper::AdvancedTimer::setEnabled("Animate", true);
			sofa::helper::AdvancedTimer::setInterval("Animate", computationTimeSampling);
			sofa::helper::AdvancedTimer::setOutputType("Animate", computationTimeOutputType);
		}
		if (int err = GUIManager::MainLoop(groot))
			return err;
		groot = dynamic_cast<Node*>(GUIManager::CurrentSimulation());


		if (groot != NULL)
			sofa::simulation::getSimulation()->unload(groot);



		GUIManager::closeGUI();

		sofa::simulation::common::cleanup();
		sofa::simulation::tree::cleanup();
#ifdef SOFA_HAVE_DAG
		sofa::simulation::graph::cleanup();
#endif
		return 0;
	}

	else {
		GUIManager::SetDimension(width, height);

		
		Node::SPtr groot = sofa::simulation::getSimulation()->load(fileName.c_str());

		if (!groot)
			groot = sofa::simulation::getSimulation()->createNewGraph("");

		if (!verif.empty())
		{
			loadVerificationData(verif, fileName, groot.get());
		}

		if (computationTimeAtBegin)
		{
			sofa::helper::AdvancedTimer::setEnabled("Init", true);
			sofa::helper::AdvancedTimer::setInterval("Init", 1);
			sofa::helper::AdvancedTimer::setOutputType("Init", computationTimeOutputType);
			sofa::helper::AdvancedTimer::begin("Init");
		}

		sofa::simulation::getSimulation()->init(groot.get());
		if (computationTimeAtBegin)
		{
			msg_info("") << sofa::helper::AdvancedTimer::end("Init", groot.get());
		}
		GUIManager::SetScene(groot, fileName.c_str(), temporaryFile);


		//=======================================
		//Apply Options

		if (startAnim)
			groot->setAnimate(true);
		if (printFactory)
		{
			msg_info("") << "////////// FACTORY //////////";
			sofa::helper::printFactoryLog();
			msg_info("") << "//////// END FACTORY ////////";
		}

		if (computationTimeSampling > 0)
		{
			sofa::helper::AdvancedTimer::setEnabled("Animate", true);
			sofa::helper::AdvancedTimer::setInterval("Animate", computationTimeSampling);
			sofa::helper::AdvancedTimer::setOutputType("Animate", computationTimeOutputType);
		}

		//=======================================
		// Run the main loop
		if (int err = GUIManager::MainLoop(groot, fileName.c_str()))
			return err;
		groot = dynamic_cast<Node*>(GUIManager::CurrentSimulation());

		if (testMode)
		{
			string xmlname = fileName.substr(0, fileName.length() - 4) + "-scene.scn";
			msg_info("") << "Exporting to XML " << xmlname;
			sofa::simulation::getSimulation()->exportXML(groot.get(), xmlname.c_str());
		}

		if (groot != NULL)
			sofa::simulation::getSimulation()->unload(groot);


		GUIManager::closeGUI();

		sofa::simulation::common::cleanup();
		sofa::simulation::tree::cleanup();
#ifdef SOFA_HAVE_DAG
		sofa::simulation::graph::cleanup();
#endif
		return 0;
	}
}
