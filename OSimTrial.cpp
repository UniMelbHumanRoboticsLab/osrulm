/***************************************************************************
 *                                                                         *
 *                          PROJECT NAME: OSLRUM			               *
 *                                                                         *
 ***************************************************************************
 * Collaborators:                                                          *
 * -  KUN CHU															   *
 * -  KUNNATH SIVAPRASAD                                                   *
 * - Collaborator 3                                                        *
 * - Vincent Croacher													   *
 * Latest Modified: 2024-05-28                                             *
 *                                                                         *
 ***************************************************************************
 *                                                                         *
 * Status:                                                                 *
 *                                                                         *
 * - Start: 20024-03-25                                                    *
 * - In Progress: Yes													   *
 * - Completed: No														   *
 *                                                                         *
 ***************************************************************************
 *                                                                         *
 * Tasks:                                                                  *
 *                                                                         *
 * - To Start:                                                             *
 *   1. Task 1 Description                                                 *
 *   2. Task 2 Description                                                 *
 *                                                                         *
 * - In Progress:                                                          *
 *   1. MocoInverse for ID                                                 *
 *   2. Task 2 Description                                                 *
 *                                                                         *
 * - Completed:                                                            *
 *   1. Task 1 Description                                                 *
 *   2. Task 2 Description                                                 *
 *                                                                         *
 * - Debug:		                                                           *
 *   1. Ulna.vtp copied to the geometry folder, fixing gap in loading models*
 *   2. Task 2 Description                                                 *
 *                                                                         *
 ***************************************************************************/


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
	std::string model_path = "C:/Users/ksiva/OneDrive/Desktop/Semester 4/Capstone/Simulation Framework/OpenSim Model/MoBL-ARMS Upper Extremity Model/Benchmarking Simulations/4.1 Model with Millard/Geometry/MOBL_ARMS_module2_4_allmuscles.osim";
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
	bool scaleProcess = modelScaler.processModel(&model, scaletool.getPathToSubject(), 78);
	
	if (scaleProcess) {
		std::cout << "Scale done!" << std::endl;
	}
	
	// check if the current system is valid
	bool systemValid = model.isValidSystem();
	initialState = model.initSystem();
	Ground();
	double newnMass = model.getTotalMass(initialState);
	std::cout << "The mass of model is: " << newnMass << std::endl;


	bool test = 1;
	std::cout << "The step successful " << test << std::endl;

	
	// read the inverse Kinematics file
	std::string ik_path = "C:/Users/ksiva/OneDrive/Desktop/Semester 4/Capstone/Simulation Framework/ID/IK/UpAndStopAndDown0.5s_JointsKin.sto";

	std::string extFrc_path = "C:/Users/ksiva/OneDrive/Desktop/Semester 4/Capstone/Simulation Framework/ID/IK/ExtForce.xml";
	std::string extFrcsto_path = "C:/Users/ksiva/OneDrive/Desktop/Semester 4/Capstone/Simulation Framework/ID/IK/UpAndStopAndDown0.5s_ConstForceLoad.sto";
	// Verify the external force file has sufficient data points before proceeding
	// using moco
	try {
		MocoInverse inverse;
		inverse.setName("matLab_motion");

		ModelProcessor modelProcessor(model);
		modelProcessor.append(ModOpAddExternalLoads(extFrc_path));
		modelProcessor.append(ModOpAddReserves());
		inverse.setModel(modelProcessor);

		inverse.setKinematics(TableProcessor(ik_path));
		inverse.set_mesh_interval(0.002);
		inverse.set_initial_time(0);
		inverse.set_final_time(1.2);
		MocoInverseSolution solution = inverse.solve();
		solution.getMocoSolution().write("MocoInverse_solution.sto");

		bool test2 = 1;
		std::cout << "The step successful " << test2 << std::endl;
	}
	catch (const std::exception& ex) {
		std::cout << "Exception: " << ex.what() << std::endl;
		return 1;
	}
	catch (...) {
		std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
		return 1;
	}
	/*MocoInverseSolution solution = inverse.solve();
	solution.getMocoSolution().write("MocoInverse_solution.sto");*/
	/*bool test = solution.getMocoSolution().success();
	std::cout << "The inverse dynamics test was successful? :" << test << std::endl;*/
	
    return 0;


}
