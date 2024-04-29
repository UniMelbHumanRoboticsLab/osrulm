//All of OpenSim and Simbody's classes available.
#include <OpenSim/OpenSim.h>
// "Use" the OpenSim namespace and certain SimTK symbols to shorten
// code ("using namespace SimTK" would cause conflicts for Body).
using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Inertia;
using SimTK::Pi;
using SimTK::convertDegreesToRadians;

// Build an OpenSim model and save it to an OSIM file.
int main() {
    try {
		Model model;
		model.setName("bicep_curl");
#ifdef VISUALIZE
		model.setUseVisualizer(true);
#endif
		OpenSim::Body* humerus = new OpenSim::Body(
			"humerus", 1, Vec3(0), Inertia(0.052, 0.004, 0.052));
		OpenSim::Body* radius = new OpenSim::Body(
			"radius", 1, Vec3(0), Inertia(0.052, 0.004, 0.052));
		// Connect the bodies with pin joints. Assume each body is 1 m long.
		PinJoint* shoulder = new PinJoint("shoulder",
			// Parent body, location in parent, orientation in parent.
			model.getGround(), Vec3(0), Vec3(0),
			// Child body, location in child, orientation in child.
			*humerus, Vec3(0, 0.5, 0), Vec3(0));
		PinJoint* elbow = new PinJoint("elbow",
			*humerus, Vec3(0, -0.5, 0), Vec3(0),
			*radius, Vec3(0, 0.5, 0), Vec3(0));
		// Add components to the model.
		model.addBody(humerus);    model.addBody(radius);
		model.addJoint(shoulder);  model.addJoint(elbow);
		model.print("OsimTr.osim");
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
    return 0;
}
