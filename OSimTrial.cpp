//// last change on 2024/4/11
////V1.2
///// update note: scaletool is all set


//All of OpenSim and Simbody's classes available.
#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>

// "Use" the OpenSim namespace and certain SimTK symbols to shorten
// code ("using namespace SimTK" would cause conflicts for Body).
using namespace OpenSim;
using namespace SimTK;
using SimTK::Vec3;
using SimTK::Inertia;
using SimTK::Pi;
using SimTK::convertDegreesToRadians;

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

	BodySet BodySet = model.getBodySet();
	JointSet JointSet = model.getJointSet();
	std::cout << "The bodies of model are: " << BodySet << std::endl;

	std::cout << std::endl;

	std::cout << "The joints of the model are" << JointSet << std::endl;

	std::cout << std::endl;


	OpenSim::Joint* elbow = &JointSet.get("elbow");
	OpenSim::Body* radius = &BodySet.get("radius");
	
	Vec3 radius_mc = radius->getMassCenter();
	std::cout << "The position of radius is: " << radius_mc << std::endl;
	
	initialState = model.initSystem();
	
	OpenSim::Coordinate jointCooridnate = elbow->get_coordinates(0);
	//double value_rotation = jointCooridnate.getValue(initialState);
	double value_rotation = jointCooridnate.getDefaultValue();
	std::cout << "the rotation angle of elbow is :" << value_rotation << std::endl;
    return 0;


}
