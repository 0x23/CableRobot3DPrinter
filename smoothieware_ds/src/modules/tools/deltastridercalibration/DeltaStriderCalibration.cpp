
//*** INCLUDE ************************************************************************************* 

#include "DeltaStriderCalibration.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"
#include "Gcode.h"
#include "Robot.h"
#include "Stepper.h"
#include "AppendFileStream.h"

#include <stdio.h>
#include <math.h>

//*** CLASS ***************************************************************************************

//--- DeltaStriderModelOptimizer ------------------------------------------------------------------
        
DeltaStriderModelOptimizer::DeltaStriderModelOptimizer(DeltaStriderSolution* pArmSolution) {
    DeltaStriderModelOptimizer::pArmSolution = pArmSolution;
}

/**
 * optimizes the arm solution model and the probing plane parameters so that all measured motor positions are mapped as close as possible to the probing plane.
 */
void DeltaStriderModelOptimizer::optimize(std::vector<Vector3>& PlaneMotorPositionMeasurements, DeltaStriderModelParameter& ModelParameter, PlaneParameter& Plane, int OptIterationCount) {
    DeltaStriderModelOptimizer::PlaneMotorPositionMeasurements = PlaneMotorPositionMeasurements;
    DeltaStriderModelOptimizer::InitialModelParameter = ModelParameter;
	DeltaStriderModelOptimizer::IterationModelParameter = ModelParameter;
		
    // flip plane if normal points into wrong direction
    if(Plane.Normal.z < 0) {
        Plane.Normal = -Plane.Normal;
        Plane.Distance = -Plane.Distance;
    }
	
	InitialPlaneParameter = Plane;
    
    // build initial parameter vector
    float ParameterVector[32];
    encodeParameterVector(ParameterVector, InitialModelParameter, Plane);
    
    // optimize
    Kernel::instance->streams->printf("Starting optimization of kinematic model...\n");
    float RMSError = optimizeLevMar(ParameterVector, OptIterationCount, 0.2f);
    Kernel::instance->streams->printf("\nOptimization finished: rms = %f\n", RMSError);
    
    // decode optimized parameter
    decodeParameterVector(ParameterVector, ModelParameter, Plane);
}

int DeltaStriderModelOptimizer::getResidualElementCount() {
    return PlaneMotorPositionMeasurements.size();
}

/**
 * Computes the world space position for each measured motor position using the forward kinematic with the current kinematic parameter set.
 * The method then returns a vector of residuals that are computed as distances from the worldspace points to the probing plane (which is simultaneously optimized)
 */
void DeltaStriderModelOptimizer::computeResidualVector(float* ParameterVector, float* ResidualVector) {
    // decode parameter vector
    PlaneParameter Plane = InitialPlaneParameter;
    DeltaStriderModelParameter ModelParameter = InitialModelParameter;
    decodeParameterVector(ParameterVector, ModelParameter, Plane);
	
    // iterate over measurements
	float l=0;
    for(unsigned int i=0; i<PlaneMotorPositionMeasurements.size(); i++) {
        // compute Cartesian position with forward kinematic and current kinematic parameters
        Vector3 p;
        bool Result = pArmSolution->forwardKinematic(PlaneMotorPositionMeasurements[i], p, ModelParameter);
		
        // compute residual as powered distance from probing plane
		// xx we use odd powers to preserve the sign. This punished large deviations harder xx
        ResidualVector[i] = Result ? (Plane.Normal|p)-Plane.Distance : 10000000;
		l += ResidualVector[i]*ResidualVector[i];
    }
}

void DeltaStriderModelOptimizer::onIterationBegin(int Itertaion, float* ParameterVector, bool* ParameterLocks) {
    PlaneParameter ProbingPlane = InitialPlaneParameter;
    DeltaStriderModelParameter ModelParameter = InitialModelParameter;
    decodeParameterVector(ParameterVector, ModelParameter, ProbingPlane);
	
	IterationModelParameter = ModelParameter;
	for(int i=0; i<3; i++)
		IterationModelParameter.Arm[i].RestLength = 0;
			
	encodeParameterVector(ParameterVector, ModelParameter, ProbingPlane);
	
	// lock plane parameter
	bool Locked = Itertaion<8;
	ParameterLocks[0] = Locked;
	ParameterLocks[1] = Locked;
	ParameterLocks[2] = Locked;
	
	// lock roller parameter
	Locked = Itertaion<16;
	ParameterLocks[3] = Locked;		// roller radius 2
	//ParameterLocks[4] = Locked;	// roller radius 2
}

void DeltaStriderModelOptimizer::onIterationEnd(int Itertaion, float RMSError, float Improvement, float* ParameterVector) {
    PlaneParameter ProbingPlane = InitialPlaneParameter;
    DeltaStriderModelParameter ModelParameter = InitialModelParameter;
    decodeParameterVector(ParameterVector, ModelParameter, ProbingPlane);
				
	encodeParameterVector(ParameterVector, ModelParameter, ProbingPlane);

    Kernel::instance->streams->printf(" Iteration %i finished: rms = %6.5f improvement: %6.3f %%\n", Itertaion, RMSError, Improvement);
    ModelParameter.print();
	Kernel::instance->streams->printf("Probeplane Dist: %f\n\n", ProbingPlane.Distance);
	
	for(int i=0; i<200; i++)
		THEKERNEL->call_event(ON_IDLE);
}

int DeltaStriderModelOptimizer::getParameterCount() {
    return 3+3+1;
}   
   
/**
 * encodes all values that need to be optimized into a parameter vector 
 * @param pParameters       pointer to parameter vector, size must be equal to the return value of 'getParameterCount'
 * @param ModelParameter    kinematic parameter of arm solution
 * @param Plane             encoded plane where xy are the xy components of the normal (z = -1) and z is the distance from origin
 */
void DeltaStriderModelOptimizer::encodeParameterVector(float* pParameters, DeltaStriderModelParameter& ModelParameter, PlaneParameter& Plane) {  
    int i=0;
    pParameters[i++] = Plane.Normal.x/Plane.Normal.z;
    pParameters[i++] = Plane.Normal.y/Plane.Normal.z;
    pParameters[i++] = Plane.Distance;

    //pParameters[i++] = ModelParameter.BaseRadiusMinusEndeffectorRadius;
    pParameters[i++] = ModelParameter.Roller1Radius;
	//pParameters[i++] = ModelParameter.Roller2Radius;
	
	Vector3 cable_offset(0,0,0);
    pArmSolution->inverseKinematic(Vector3(0,0,Plane.Distance), cable_offset, IterationModelParameter);
    
    pParameters[i++] = ModelParameter.Arm[0].RestLength-cable_offset.x;
    pParameters[i++] = ModelParameter.Arm[1].RestLength-cable_offset.y;
    pParameters[i++] = ModelParameter.Arm[2].RestLength-cable_offset.z;

    // safety check for developer
    if(getParameterCount() != i) {
        Kernel::instance->streams->printf("ERROR! in DeltaStriderModelParameter::toVector: parameter count doesn't equal the value returned by getParameterVectorSize()\n");
        return;
    }
}

/**
 * decodes a parameter vector, see 'encodeParameterVector'
 */
void DeltaStriderModelOptimizer::decodeParameterVector(float* pParameters, DeltaStriderModelParameter& ModelParameter, PlaneParameter& Plane) {
    int i=0;
    Plane.Normal.x = pParameters[i++];
    Plane.Normal.y = pParameters[i++];
    Plane.Normal.z = 1.0f;
    Plane.Normal.normalize();
    Plane.Distance = pParameters[i++];

    //ModelParameter.BaseRadiusMinusEndeffectorRadius = pParameters[i++];
    ModelParameter.Roller1Radius = pParameters[i++];
    //ModelParameter.Roller2Radius = pParameters[i++];
	
	Vector3 cable_offset(0,0,0);
    pArmSolution->inverseKinematic(Vector3(0,0,Plane.Distance), cable_offset, IterationModelParameter);
	    
    ModelParameter.Arm[0].RestLength = pParameters[i++]+cable_offset.x;
    ModelParameter.Arm[1].RestLength = pParameters[i++]+cable_offset.y;
    ModelParameter.Arm[2].RestLength = pParameters[i++]+cable_offset.z;
    
    // safety check for developer
    if(getParameterCount() != i) {
        Kernel::instance->streams->printf("ERROR! in DeltaStriderModelParameter::toVector: parameter count doesn't equal the value returned by getParameterVectorSize()\n");
        return;
    }
}
    
//--- DeltaStriderCalibration ---------------------------------------------------------------------

DeltaStriderCalibration::DeltaStriderCalibration() {
    CalibrationMCode = 350;
	ToolZeroMCode = 351;
}

void DeltaStriderCalibration::on_module_loaded() {
	Touchprobe::on_module_loaded();
}

void DeltaStriderCalibration::on_gcode_received(void* argument) {
	Touchprobe::on_gcode_received(argument);
    Gcode *gcode = static_cast<Gcode *>(argument);

	// calibration
    if( gcode->has_m && gcode->m == CalibrationMCode) {
        gcode->mark_as_taken();
        
        int ProbingCount = 54;
        int IterationCount = 30;
        float MaxRadius = 50;
        float ProbingFeedrate = 30;
        
        if(gcode->has_letter('R'))
            MaxRadius = gcode->get_value('R');
        if(gcode->has_letter('N'))
            ProbingCount = (int)gcode->get_value('N');
        if(gcode->has_letter('F'))
            ProbingFeedrate = THEKERNEL->robot->to_millimeters( gcode->get_value('F') )/THEKERNEL->robot->get_seconds_per_minute();
		if(gcode->has_letter('I'))
            IterationCount = gcode->get_value('I');
                        
        doCalibration(200, ProbingFeedrate, MaxRadius, ProbingCount, IterationCount);
    }
	
	// set tool tip zero height
    if( gcode->has_m && gcode->m == ToolZeroMCode) {
        gcode->mark_as_taken();
		
		// get current position
		float InitialPosition[3];
		float ActuatorPositions[3];
		THEKERNEL->robot->get_axis_position(InitialPosition);
		THEKERNEL->robot->arm_solution->cartesian_to_actuator(InitialPosition, ActuatorPositions);

		DeltaStriderSolution* pArmSolution = DeltaStriderSolution::getInstance();
		if(pArmSolution == NULL) {
			Kernel::instance->streams->printf("ERROR: The delta strider calibration only works with the delta strider arm solution!\n");
			return;     
		}
		
		// change arm solution
		Matrix4x4 WorldToBasis;
		pArmSolution->getBaseToWorldTransform(WorldToBasis);
		WorldToBasis._34 -= InitialPosition[2];
		pArmSolution->setBaseToWorldTransform(WorldToBasis);
				
		// synchronize
		float NewPosition[3];
		THEKERNEL->robot->arm_solution->actuator_to_cartesian(ActuatorPositions, NewPosition);
		for(int i = X_AXIS; i<=Z_AXIS; i++)
			THEKERNEL->robot->reset_axis_position(NewPosition[i], i);
		
		THEKERNEL->streams->printf("Set current position: %f %f %f\n", NewPosition[0], NewPosition[1], NewPosition[2]);
	}
	
	if( gcode->has_m && gcode->m == 352) {
        gcode->mark_as_taken();
		
		DeltaStriderSolution* pArmSolution = DeltaStriderSolution::getInstance();
		if(pArmSolution == NULL) {
			Kernel::instance->streams->printf("ERROR: The delta strider calibration only works with the delta strider arm solution!\n");
			return;     
		}
		
		DeltaStriderModelParameter ModelParameter;
		pArmSolution->getModelParameter(ModelParameter);
		
		ModelParameter.print();
	}

}

void DeltaStriderCalibration::on_config_reload(void* argument) {
}
        
/**
 * Performs the calibration of the delta strider
 * After calibration the position (0,0,0) is located at the robot center exactly on the probe plane
 * and the XY plane is coplanar to he probe plane
 * @param MoveSpeed         speed for rapid moves in mm/s
 * @param ProbeSpeed        speed during probing in mm/s
 * @param MaxProbeRadius    radius of probing disk around X=0,Y=0
 * @param ProbePointCount   number of measuring points
 */
bool DeltaStriderCalibration::doCalibration(float MoveSpeed, float ProbeSpeed, float MaxProbeRadius, int ProbePointCount, int OptIterationCount) {
    // check if delta strider arm solution exists
    DeltaStriderSolution* pArmSolution = DeltaStriderSolution::getInstance();
    if(pArmSolution == NULL) {
        Kernel::instance->streams->printf("ERROR: The delta strider calibration only works with the delta strider arm solution!\n");
        return false;     
    }
	
    // optimize model parameter
    DeltaStriderModelParameter ModelParameter;
    pArmSolution->getModelParameter(ModelParameter);
	
	Vector3 home;
	pArmSolution->forwardKinematic(Vector3(0,0,0), home, ModelParameter);
    
    PlaneParameter ProbingPlane;
    ProbingPlane.Normal = Vector3(0,0,1);
    ProbingPlane.Distance = home.z;
	
	THEKERNEL->streams->printf("------------------------- Auto Calibration -------------------------\n");	
	THEKERNEL->streams->printf("Initial Model Parameter\n");	
	ModelParameter.print();
	THEKERNEL->streams->printf("\n\n");	
    // compute probing points
    std::vector<Vector3> ProbePoints;
    computeProbePoints(ProbePoints, MaxProbeRadius, ProbePointCount);
    
    // measure motor positions for points on the probing plane
    THEKERNEL->streams->printf("Starting measurement of %i points (radius = %f)...\n", ProbePointCount, MaxProbeRadius);
	THEKERNEL->call_event(ON_IDLE);
    std::vector<Vector3> MotorPositionMeasurements;
    if(doCalibrationMeasurements(MotorPositionMeasurements, ProbePoints, MoveSpeed, ProbeSpeed) == false) {
        Kernel::instance->streams->printf("Probe plane not found. Move the Touch probe less then 10mm above the plane.\n");
        return false;
    }    
    
    DeltaStriderModelOptimizer Optimizer(pArmSolution);
    Optimizer.optimize(MotorPositionMeasurements, ModelParameter, ProbingPlane, OptIterationCount);
    
    // compute transform matrix for bed levelling
    Matrix4x4 WorldToBasis = computeWorldToBasisTransform(ProbingPlane);
    pArmSolution->setWorldToBaseTransform(WorldToBasis);
	pArmSolution->setModelParameter(ModelParameter);

	// synchronize tool position (this works because we know the actuator positions at our current position: 0,0,0)
    float WorldPos[3];
	float ActuatorPositions[3] = {0,0,0};
    THEKERNEL->robot->arm_solution->actuator_to_cartesian(ActuatorPositions, WorldPos);
     for(int i = X_AXIS; i<=Z_AXIS; i++)
        THEKERNEL->robot->reset_axis_position(WorldPos[i], i);
	
    // print results
    Kernel::instance->streams->printf("\n----------------------------------------------------------------------\n");
	ModelParameter.print();
    Kernel::instance->streams->printf("Probing Plane: n=(%f,%f,%f)  d=%f\n", ProbingPlane.Normal.x, ProbingPlane.Normal.y, ProbingPlane.Normal.z, ProbingPlane.Distance);
    Kernel::instance->streams->printf("World to Basis Translation: n=(%f,%f,%f)\n", WorldToBasis._14, WorldToBasis._24, WorldToBasis._34);

	// move probe to center position
	moveProbe(0, 0, NAN, 50, false);    
	THEKERNEL->stepper->turn_enable_pins_off();
	
	// write point plot file
    std::vector<Vector3> points;
	for(auto& ap : MotorPositionMeasurements) {
		Vector3 cp;
		THEKERNEL->robot->arm_solution->actuator_to_cartesian(ap.elem, cp.elem);
		points.push_back(cp);
	}

	Kernel::instance->streams->printf("Writing dscalib_points.gp ...\n");
	if(writePointPlotFile("/sd/dscalib_points.gp", points) == false) {
		Kernel::instance->streams->printf("Unable to write dscalib_points.gp !\n");
	}
	
    return true;
}

/**
 * Performs measurements for calibration by probing a planar object. 
 * Assumes that current kinematic model is accurate enough to move within 
 * the workspace without collisions. Also the position (0,0,0) should be safely reachable within the workspace.
 * The probe should be located not more than 10mm above the probing plane
 */
bool DeltaStriderCalibration::doCalibrationMeasurements(std::vector<Vector3>& MotorPositionMeasurements, std::vector<Vector3>& ProbePoints, float MoveSpeed, float ProbeSpeed) {
    Vector3 ReferenceMotorPos;
    int StepsMoved[3];
    Vector3 ProbingOffset(0,0,-10);
    
	// initial motor positions
    float WorldPos[3];
	Vector3 InitialMotorPositions;
    THEKERNEL->robot->get_axis_position(WorldPos);
    THEKERNEL->robot->arm_solution->cartesian_to_actuator(WorldPos, InitialMotorPositions.elem);
	
    Kernel::instance->streams->printf("Initial motor position: (%f,%f,%f)\n", InitialMotorPositions.x, InitialMotorPositions.y, InitialMotorPositions.z);

    // probe reference point
    moveProbe(0, 0, NAN, MoveSpeed, false);
    if(doProbing(StepsMoved, ReferenceMotorPos.elem, ProbingOffset.elem, ProbeSpeed, true) == false)
        return false;
        
    // probe plane
    for(auto ProbePoint : ProbePoints) {
        // probe current position
        Vector3 MotorPositions;
        moveProbe(ProbePoint.x, ProbePoint.y, NAN, MoveSpeed, false);
        doProbing(StepsMoved, MotorPositions.elem, ProbingOffset.elem, ProbeSpeed, true);
		
        // save measurement
        MotorPositionMeasurements.push_back(MotorPositions-InitialMotorPositions);
    }
    
	moveProbe(0, 0, NAN, MoveSpeed, false);
	THEKERNEL->stepper->turn_enable_pins_off();
	
    return true;
}

/**
 * computes the transformation from world coordiantes to basis plattform coordinates so that the XY plane is coplanar to the 'ProbePlane'
 * @param ProbePlane    plane in basis plattform coordinates
 */
Matrix4x4 DeltaStriderCalibration::computeWorldToBasisTransform(PlaneParameter ProbePlane) {
    Matrix4x4 Result;
    Vector3 DirZ = (ProbePlane.Normal).normalized();
    Vector3 DirX = (Vector3(0,1,0)%DirZ).normalized();
    Vector3 DirY = (DirZ%DirX).normalized();
    Vector3 Translation(0,0,ProbePlane.Distance/DirZ.z);   // intersection of z-axis and plane
     
    Result.buildFromBasisVectors(DirX, DirY, DirZ, Translation);
    return Result;
}

/**
 * generates evenly distributed probing points on a disk in the XY plane
 * @param ProbePoints       output list of probing points
 * @param MaxProbeRadius    radius of the disc
 * @param ProbePointCount   number of probing points
 */
void DeltaStriderCalibration::computeProbePoints(std::vector<Vector3>& ProbePoints, float MaxProbeRadius, int ProbePointCount) {
    const float GoldenAngle = 3.1415926f * (3.0f-sqrtf(5.0f));
	ProbePointCount /= 3;
	
	float theta = 0;
	std::vector<Vector3> UnsortedProbePoints;
    for(int i=0; i<ProbePointCount; i++) {
        float r = sqrt(float(1+i)/float(ProbePointCount+1))*MaxProbeRadius;
		for(int j=0; j<3; j++) {
			UnsortedProbePoints.push_back(Vector3(cosf(theta), sinf(theta), 0)*r);
			theta += GoldenAngle;
		}
	}
	
	// do some cheap travelling salesman thing
	Vector3 p = UnsortedProbePoints[0];
	UnsortedProbePoints.erase(UnsortedProbePoints.begin());
	ProbePoints.push_back(p);
	
	while(UnsortedProbePoints.size() > 0) {
		// find nearest point to p
		float dmin = 10000000;
		unsigned int Index = 0;
		for(unsigned int i=0; i<UnsortedProbePoints.size(); i++) {
			float d = (p-UnsortedProbePoints[i]).mag();
			if(d < dmin) {
				Index = i;
				dmin = d;
			}
		}
		
		p = UnsortedProbePoints[Index];
		UnsortedProbePoints.erase(UnsortedProbePoints.begin()+Index);
		ProbePoints.push_back(p);
	}
}

//*** FUNCTION ************************************************************************************
/**
 * writes the given points into a file that can directly be plotted with gnuplot
 */
bool writePointPlotFile(const char* Filename, const std::vector<Vector3>& Points) {
   /* FILE* fd = fopen(Filename, "w");
    if(fd == nullptr)
		return false;
	fclose(fd);*/
	
	//AppendFileStream fs(Filename);
	auto& fs = *Kernel::instance->streams;
	fs.printf("splot '-' using 1:2:3 with points\n");
	
	for(auto& p : Points)
		fs.printf("%f %f %f\n", p.x, p.y, p.z);
		
	fs.printf("EOF\n");
	
	return true;
}

//*************************************************************************************************
