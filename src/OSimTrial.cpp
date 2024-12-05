//// last change on 2024/4/11
//// V1.4
///// update note: scaletool is all set


//All of OpenSim and Simbody's classes available.
#include <OpenSim/OpenSim.h>
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>
#include <fstream>
#include <string>
#include <stdexcept>

// "Use" the OpenSim namespace and certain SimTK symbols to shorten
// code ("using namespace SimTK" would cause conflicts for Body).
using namespace OpenSim;
using namespace SimTK;
using SimTK::Vec3;
using SimTK::Inertia;
using SimTK::Pi;
using SimTK::convertDegreesToRadians;
using namespace std;

void model_scale(Model model, String model_path, double mass) {

	// print the origin mass of the model
	ScaleTool scaletool = ScaleTool(model_path);
	//useing scaletool of model to get model scaler
	ModelScaler modelScaler = scaletool.getModelScaler();
	//scale set:
	ScaleSet scaleSet = modelScaler.getScaleSet();
	//get the initial state of model
	State initialState = model.initSystem();
	//check the original model mass:
	double orginMass = model.getTotalMass(initialState);
	std::cout << "The original arm mass of model is: " << orginMass << std::endl;
	//scale the model
	double finalMasss = mass;

	bool scaleProcess = modelScaler.processModel(&model, scaletool.getPathToSubject(), finalMasss);

	if (scaleProcess) {
		std::cout << "Scale done!" << std::endl;
	}


	// check if the current system is valid
	bool systemValid = model.isValidSystem();
	initialState = model.initSystem();
	Ground();
	double newnMass = model.getTotalMass(initialState);
	std::cout << "The scaled arm mass of model is: " << newnMass << std::endl;
}

MocoSolution moco_inverse(Model model, String ex_file, String jointskin_file) {
	// find the number of muscle need to be replaced
	int numberMuscles = model.getMuscles().getSize();
		//replace muscle in the model
		for (int i = 0; i < numberMuscles; i++) {
			Muscle* mm = &model.getMuscles().get(i);
			Millard2012EquilibriumMuscle* mmM = static_cast<Millard2012EquilibriumMuscle*>(mm);
			mmM->set_default_activation(0.05);
			mmM->set_fiber_damping(0.017); // the artical mention it is 0.017
		}

		DeGrooteFregly2016Muscle newMuscle = DeGrooteFregly2016Muscle();

		newMuscle.replaceMuscles(model, true);


		// start moco inverse process
		MocoInverse inverse;
		inverse.setName("I_dynamic");
		// choose the target mdoel
		ModelProcessor modelProcessor(model);
		// load external force data
		modelProcessor.append(ModOpAddExternalLoads(ex_file));
		// limit reserves
		modelProcessor.append(ModOpAddReserves(1, 10));
		modelProcessor.append(ModOpIgnoreTendonCompliance());

		// Only valid for DeGrooteFregly2016Muscles.
		modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
		// Only valid for DeGrooteFregly2016Muscles.
		modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
		modelProcessor.append(ModOpIgnoreTendonCompliance());
		// end new

		// input the model processor in the model
		inverse.setModel(modelProcessor);

		//load the joint kinematic data
		inverse.setKinematics(TableProcessor("D:/D/unimelb/capstone/UpAndStopAndDown0.7s_JointsKin.sto"));


		// allowed extra columns to avoid error
		inverse.set_kinematics_allow_extra_columns(true);

		// set the start and end time of the moco inverse process
		inverse.set_mesh_interval(0.002);
		inverse.set_initial_time(0);
		inverse.set_final_time(0.4);

		// set the simulation tolerance
		inverse.set_convergence_tolerance(0.001);

		// convert to moco study
		MocoStudy inverseStudy = inverse.initialize();


		// set the problem of the moco inverse
		MocoProblem& inverseProblem = inverseStudy.updProblem();
		MocoSolution solution = inverseStudy.solve();
		solution = solution.unseal();

		// output the solution as a sto file
		solution.write("muscle_activation.sto");
		// return the solution which is the muscle active values
		return solution;

}


void force_prediction(MocoSolution moco_inverse_solution, String model_path) {

	STOFileAdapter::write(moco_inverse_solution.exportToStatesTable(), "Muscle_states.sto");
	STOFileAdapter::write(moco_inverse_solution.exportToControlsTable(), "Muscle_controls.sto");

	{
		// Create an AnalyzeTool setup file.
		AnalyzeTool analyze;
		analyze.setName("analyze");
		analyze.setModelFilename(model_path);
		analyze.setStatesFileName("Muscle_states.sto");
		analyze.updAnalysisSet().adoptAndAppend(new MuscleAnalysis());
		analyze.updControllerSet().adoptAndAppend(
			new PrescribedController("Muscle_controls.sto"));
		analyze.print("results.xml");
	}
	// start analyze
	AnalyzeTool analyze("results.xml");
	analyze.run();
}


// Build an OpenSim model and save it to an OSIM file.
int main() {
	std::string model_path = "../../MoBL-ARMS OpenSim tutorial_33/ModelFiles/MOBL_ARMS_module2_4_allmuscles.osim";
    try {
		Model model(model_path);
		model.setName("RightArm");
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

	

	///////////////////// scalling
	// load the model

	String ex_file = "../data/input/ExtForce.xml";
	double g_value = -9.81;


	Model model(model_path);
	model.setName("RightArm");
	
	//check if model is scalled
	ScaleTool scaletool = ScaleTool(model_path);
	////////////////////////////////////////////optional set the gravity
	Vec3 g = model.getGravity();
	std::cout << "The original gravity setting is: " << g << std::endl;
	model.set_gravity(SimTK::Vec3(0, g_value, 0));
	g = model.getGravity();
	std::cout << "The original gravity setting is: " << g << std::endl;
	

	//useing scaletool of model to get model scaler
	ModelScaler modelScaler = scaletool.getModelScaler();
	std::cout << "The pass is: " << scaletool.getPathToSubject() << std::endl;

	//scale set:
	ScaleSet scaleSet = modelScaler.getScaleSet();
	//get the initial state of model
	State initialState = model.initSystem();
	//check the mass:
	double orginMass = model.getTotalMass(initialState);
	std::cout << "The mass of model is: " << orginMass << std::endl;
	//scale the model
	double finalMasss =6;

	bool scaleProcess = modelScaler.processModel(&model, scaletool.getPathToSubject(), finalMasss);
	
	if (scaleProcess) {
		std::cout << "Scale done!" << std::endl;
	}
	
	
	// check if the current system is valid
	bool systemValid = model.isValidSystem();
	initialState = model.initSystem();
	Ground();
	double newnMass = model.getTotalMass(initialState);
	std::cout << "The mass of scaled model is: " << newnMass << std::endl;

	//String kin_file = "../..data/input/UpAndStopAndDown0.7s_JointsKin.sto";
	//moco_inverse(model, ex_file, kin_file);

	int numberMuscles = model.getMuscles().getSize();
	//v2
	try {
		//replace muscle in the model

		for (int i = 0; i < numberMuscles; i++) {
			Muscle* mm = &model.getMuscles().get(i);
			Millard2012EquilibriumMuscle* mmM = static_cast<Millard2012EquilibriumMuscle*>(mm);
			mmM->setDefaultActivation(0.05); //Osim API is broken, accessors don;t work
			mmM->setFiberDamping(0.017); // the article mention it is 0.017, try this value later //Osim API is broken, accessors don;t work
		}

		DeGrooteFregly2016Muscle newMuscle = DeGrooteFregly2016Muscle();
		newMuscle.replaceMuscles(model, true);

		
		
		MocoInverse inverse;
		inverse.setName("I_dynamic");

		ModelProcessor modelProcessor(model);
		modelProcessor.append(ModOpAddExternalLoads(ex_file)); //TODO: replace w/ dynamic loading of force w/o XML file (but not documented in osim API)
		modelProcessor.append(ModOpAddReserves(1,10));
		//modelProcessor.append(ModOpIgnoreTendonCompliance()); //Broken, raise an exception, not valid for this model
		//modelProcessor.append(ModOpRemoveMuscles());


		//modelProcessor.append(ModOpReplaceMusclesWithDeGrooteFregly2016());
		// Only valid for DeGrooteFregly2016Muscles.
		modelProcessor.append(ModOpIgnorePassiveFiberForcesDGF());
		// Only valid for DeGrooteFregly2016Muscles.
		modelProcessor.append(ModOpScaleActiveFiberForceCurveWidthDGF(1.5));
		//modelProcessor.append(ModOpIgnoreTendonCompliance()); //Broken, raise an exception, not valid for this model
		// end new
		

		inverse.setModel(modelProcessor);

		inverse.setKinematics(TableProcessor("../data/input/arm_extension_0.7s_JointsKin.sto"));


		// allowed extra columns to avoid error
		inverse.set_kinematics_allow_extra_columns(true);

		inverse.set_mesh_interval(0.002);
		inverse.set_initial_time(0);
		inverse.set_final_time(0.4);
		inverse.set_convergence_tolerance(0.001);
		//inverse.set_constraint_tolerance(0.01);

		// convert to moco study
		MocoStudy inverseStudy = inverse.initialize();

		MocoProblem& inverseProblem = inverseStudy.updProblem();

		MocoSolution solution = inverseStudy.solve();


		solution = solution.unseal();
		solution.write("raise_0g_-force_-y.sto");

		STOFileAdapter::write(solution.exportToStatesTable(), "Muscle_states.sto");
		STOFileAdapter::write(solution.exportToControlsTable(), "Muscle_controls.sto");
		////try 0.5 initial stop, 0.5 moving, 0.2 stop
		std::cout << "e\n";
		{
			// Create an AnalyzeTool setup file.
			AnalyzeTool analyze;
			analyze.setName("analyze");
			analyze.setModelFilename(model_path);
			analyze.setStatesFileName("Muscle_states.sto");
			analyze.updAnalysisSet().adoptAndAppend(new MuscleAnalysis());
			analyze.updControllerSet().adoptAndAppend(
				new PrescribedController("Muscle_controls.sto"));
			analyze.print("results.xml");
		}
		AnalyzeTool analyze("results.xml");
		analyze.run();
		//try 0.5 initial stop, 0.5 moving, 0.2 stop

		



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
	return 0;


}
