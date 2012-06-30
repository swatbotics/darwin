#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <stdio.h>
#include <fstream>

#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/format.hpp>

using namespace OpenRAVE;
using namespace std;

int main(int argc, char ** argv)
{
    if( argc < 3 ) {
        RAVELOG_INFO("ikloader robot iktype\n");
        return 1;
    }

    string robotname = argv[1];
    string iktype = argv[2];
    RaveInitialize(true); // start openrave core

    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment

    // lock the environment to prevent changes
    EnvironmentMutex::scoped_lock lock(penv->GetMutex());
    // load the scene
    RobotBasePtr probot = penv->ReadRobotXMLFile(robotname);
    if( !probot ) {
        penv->Destroy();
        return 2;
    }
    penv->AddRobot(probot);

    ModuleBasePtr pikfast = RaveCreateModule(penv,"ikfast");
    penv->AddModule(pikfast,"");
    stringstream ssin,ssout;
    ssin << "LoadIKFastSolver " << probot->GetName() << " " << iktype;
        // if necessary, add free inc for degrees of freedom
        //ssin << " " << 0.04f;
        // get the active manipulator
    probot->SetActiveManipulator("rightleg");
    RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();
    if( !pikfast->SendCommand(ssout,ssin) ) {
        RAVELOG_ERROR("failed to load iksolver\n");
        penv->Destroy();
        return 1;
    }

	//        while(1) {
	/*     Transform trans;
            trans.rot = quatFromAxisAngle(Vector(90,90,0));
            trans.trans = Vector(-.005, .037, .01781);
	*/  
    Transform trans = pmanip->GetEndEffectorTransform();
    trans.trans[2]+=.02;
    //    printf("%f %f %f %f\n",trans.trans[0],trans.trans[1],trans.trans[2],trans.trans[3]);
  
    vector<dReal> vsolution;
    if( pmanip->FindIKSolution(IkParameterization(trans),vsolution,IKFO_CheckEnvCollisions) ) {
        vector<int> indices = pmanip->GetArmIndices();
        fstream output;
	output.open("jointControl.txt",ios::out|ios::trunc);
        for(size_t i = 0; i < vsolution.size(); ++i) {
	  output << indices[i] << " " << (double)vsolution[i] << " \n";
        }
        output << "+" << endl;
	output.close();
    }
    
    else {
        // could fail due to collisions, etc
        printf("oops, failed\n");
    }
   
    //    RaveDestroy(); // destroy
    return 0;
}
