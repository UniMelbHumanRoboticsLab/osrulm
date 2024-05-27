//// last change on 2024/4/11
////V1.4
///// update note: scaletool is all set


//All of OpenSim and Simbody's classes available.
#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>
#include <fstream>
#include <string>

// "Use" the OpenSim namespace and certain SimTK symbols to shorten
// code ("using namespace SimTK" would cause conflicts for Body).
using namespace OpenSim;
using namespace SimTK;
using SimTK::Vec3;
using SimTK::Inertia;
using SimTK::Pi;
using SimTK::convertDegreesToRadians;
using namespace std;

float** readTable(std::string file, int& row, int& col) {

	ifstream inputFile(file);
	//check if file is opened or not
	if (!inputFile) {
		cerr << "Error opening file!" << endl;
		return nullptr;
	}

	string skip;
	for (int i = 0; i < 8; ++i) {
		getline(inputFile, skip);
	}


	float** table = new float*[row];
	for (int i = 0; i < row; ++i) {
		table[i] = new float[col];
	}
	for (int i = 0; i < row; ++i) {
		for (int j = 0; j < col; ++j) {
			inputFile >> table[i][j];
		}
	}
	inputFile.close();
	return table;
}

void printTable(float** table, int rows, int cols) {
	// Print the table
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			cout << table[i][j] << "\t";
		}
		cout << endl;
	}
}

// Build an OpenSim model and save it to an OSIM file.
int main() {
	std::string model_path = "D:/D/unimelb/capstone/MobL_ARMS/MoBL-ARMS Upper/Benchmarking Simulations/4.1 Model with Millard/Geom/MOBL_ARMS_module2_4_allmuscles.osim";
    try {

		Model model(model_path);
		model.setName("RightArm");
		std::cout << model.getBodySet() << std::endl;
		std::cout << model.getNumBodies() << std::endl;
#ifdef VISUALIZE
		model.setUseVisualizer(true);
#endif
		
    }
	catch (const std::exception& ex) {
		std::cout << ex.what() << std::endl;
		return 1;
	}
	catch (...) {
		std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
		return 1;
	}
    std::cout << "OpenSim example completed successfully" << std::endl;
    // std::cout << "Press return to continue" << std::endl;
    // std::cin.get();
	

	///////////////////// scalling
	// load the model
	Model model(model_path);
	model.setName("RightArm");
	//check if model is scalled
	ScaleTool scaletool = ScaleTool(model_path);
	////////////////////////////////////////////optional set the gravity
	model.set_gravity(SimTK::Vec3(9.81, 0, 0));
	/*model.set_gravity(SimTK::Vec3(0, 0, 0));*/

	//bool run_scale = scaletool.run();
	// check if scaletool is running
	//if (run_scale) {
	//	std::cout << "The scaletool is actived."<< std::endl;
	//}
	

	//useing scaletool of model to get model scaler
	ModelScaler modelScaler = scaletool.getModelScaler();
	std::cout << "The pass is: " << scaletool.getPathToSubject() << std::endl;
	//proceess the mode in scaller
	//bool process_mode = modelscaler.processModel(model,scaletool.getPathToSubject(), -1);
	//scale set:
	ScaleSet scaleSet = modelScaler.getScaleSet();
	//get the initial state of model
	State initialState = model.initSystem();
	//check the mass:
	double orginMass = model.getTotalMass(initialState);
	std::cout << "The mass of model is: " << orginMass << std::endl;
	//scale the model
	bool scaleProcess = modelScaler.processModel(&model, scaletool.getPathToSubject(), 2);
	
	if (scaleProcess) {
		std::cout << "Scale done!" << std::endl;
	}
	
	// check if the current system is valid
	bool systemValid = model.isValidSystem();
	initialState = model.initSystem();
	Ground();
	double newnMass = model.getTotalMass(initialState);
	std::cout << "The mass of model is: " << newnMass << std::endl;

	// virsualization
	VisualizerUtilities visualizer;
	visualizer.showModel(model);

	// track the body and joint
	Vector nq = initialState.getQ();
	std::cout << "The geof coordinate of model is: " << nq << std::endl;

	BodySet BodySet = model.getBodySet(); // get the body set 

	JointSet JointSet = model.getJointSet(); // get the joint set

	std::cout << "The bodies of model are: " << BodySet << std::endl;

	std::cout << std::endl;

	std::cout << "The joints of the model are" << JointSet << std::endl;

	std::cout << std::endl;


	// example here, try to contrl elbow joint first
	OpenSim::Joint* elbow = &JointSet.get("elbow");
	OpenSim::Body* radius = &BodySet.get("radius");
	
	Vec3 radius_mc = radius->getMassCenter();
	std::cout << "The position of radius is: " << radius_mc << std::endl;
	
	initialState = model.initSystem();
	
	OpenSim::Coordinate jointCooridnate = elbow->get_coordinates(0); 
	//double value_rotation = jointCooridnate.getValue(initialState);
	double value_rotation = jointCooridnate.getDefaultValue();
	std::cout << "the rotation angle of elbow is :" << value_rotation << std::endl;

	//setting the controller to rotate elbow
	// check if there is any controller inside of model
	ControllerSet controllerset = model.getControllerSet();
	std::cout << "the controllers in model are:" << controllerset.getControlTable() << std::endl;
	// its empty, we add contoller by ourself
	//PrescribedController* brain = new PrescribedController();
	//brain->addActuator(*elbow);
	BodyActuator* robot_force = new BodyActuator();
	robot_force->setPoint(radius_mc);
	robot_force->setBody(BodySet.get("radius"));


	// read the inverse Kinematics file
	std::string ik_path = "D:/D/unimelb/capstone/UpAndStopAndDown0.5s_JointsKin.sto";

	// number of row and col
	int row = 604;
	int col = 21;
	float** ikTable = readTable(ik_path, row, col);
	if (ikTable != nullptr) {
		cout << "The table read from the file is:" << endl;
		printTable(ikTable, row, col);
	}

	// using moco
	MocoInverse inverse;
	inverse.setName("matLab_motion");
	inverse.setModel(ModelProcessor(model));




	//try to use moco to move the arm by using provided joint position. Two options use study or track
	
	MocoStudy study;
	study.setName("Matlab_example");
	MocoProblem problem = study.updProblem();
	// load the model in phase 0
	problem.setModel(std::make_unique<OpenSim::Model>(model_path));
	//set start time and end time range
	problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(0.5, 1.5));
	//set state infor. Use the path in model file
	problem.setStateInfo("/jointset/elbow", { 0, 3 }, 1.0);
	auto* jointControl= problem.addGoal<MocoStateTrackingGoal>();
	auto pathCon = MocoControlBoundConstraint();
	MocoSolution solution = study.solve();

	MocoStudy study_id;
	study_id.setName("I_dynamic");
	MocoProblem problem = study_id.updProblem();
	problem.setModel(std::make_unique<OpenSim::Model>(model_path));
	problem.setTimeBounds(MocoInitialBounds(0), MocoFinalBounds(0.5, 1.5));
	problem.setStateInfo("/jointset", { 0, 2*Pi});

	MocoInverse inverse;
	inverse.setName("I_dynamic");
	inverse.setModel(ModelProcessor("D:/D/unimelb/capstone/MobL_ARMS/MoBL-ARMS Upper/Benchmarking Simulations/4.1 Model with Millard/Geom/MOBL_ARMS_module2_4_allmuscles.osim") |
		ModOpAddExternalLoads("external_loads.xml") |
		ModOpAddReserves());
	/*
	//set a track
	MocoTrack track;
	track.setName("joint_track");
	//end of the code
	*/
    return 0;


}
