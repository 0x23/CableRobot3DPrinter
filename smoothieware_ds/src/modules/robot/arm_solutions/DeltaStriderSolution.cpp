
#include "DeltaStriderSolution.h"
#include "StreamOutputPool.h"
#include "checksumm.h"
#include "ConfigValue.h"

#include <fastmath.h>

const float sqrt3 = sqrtf(3.0f);
const float cos120 = -0.5f;    
const float sin120 = sqrt3/2.0f; 

#define base_radius_checksum          CHECKSUM("base_radius")
#define cable_radius_checksum         CHECKSUM("cable_radius")
#define plattform_radius_checksum     CHECKSUM("plattform_radius")
#define roller1_radius_checksum       CHECKSUM("roller1_radius")
#define roller2_radius_checksum       CHECKSUM("roller2_radius")
#define roller2_distance_checksum     CHECKSUM("roller2_distance")
#define intitial_height_checksum      CHECKSUM("intitial_height")
#define planedev_x_checksum      	  CHECKSUM("planedev_x")
#define planedev_y_checksum    		  CHECKSUM("planedev_y")

//--- DeltaStriderArmParameter --------------------------------------------------------------------
   
DeltaStriderArmParameter::DeltaStriderArmParameter() {
    RestLength = 190;
}

//--- DeltaStriderModelParameter ------------------------------------------------------------------

DeltaStriderModelParameter::DeltaStriderModelParameter() {
    BaseRadiusMinusEndeffectorRadius = 100.0f;
    Roller1Radius = 4.125f;
    Roller2Radius = 2.875f;
    Roller2Distance = 6.0f;
}

void DeltaStriderModelParameter::print() {
    Kernel::instance->streams->printf("Base radius minus end effector radius: : %f\n", BaseRadiusMinusEndeffectorRadius);
	Kernel::instance->streams->printf("RestLength 0: %f\n", Arm[0].RestLength);
    Kernel::instance->streams->printf("RestLength 1: %f\n", Arm[1].RestLength);
    Kernel::instance->streams->printf("RestLength 2: %f\n", Arm[2].RestLength);
	Kernel::instance->streams->printf("Roller1 Radius: %f\n", Roller1Radius);
    Kernel::instance->streams->printf("Roller2 Radius: %f\n", Roller2Radius);
    Kernel::instance->streams->printf("Roller2 Distance: %f\n", Roller2Distance);
}

//--- DeltaStriderSolution ------------------------------------------------------------------------

DeltaStriderSolution* DeltaStriderSolution::pInstance = NULL;
      
DeltaStriderSolution::DeltaStriderSolution() {
    pInstance = this;
    BaseToWorldTransform.buildUnit();
    WorldToBaseTransform.buildUnit();
}
       
DeltaStriderSolution::DeltaStriderSolution(Config* passed_config) : config(passed_config){
	float CableRadius = config->value(cable_radius_checksum)->by_default(0.125f)->as_number();
	ModelParameter.BaseRadiusMinusEndeffectorRadius = config->value(base_radius_checksum)->by_default(126.18f)->as_number() - config->value(plattform_radius_checksum)->by_default(25.4f)->as_number();
	ModelParameter.Roller1Radius = config->value(roller1_radius_checksum)->by_default(4.0f)->as_number()+CableRadius;
	ModelParameter.Roller2Radius = config->value(roller2_radius_checksum)->by_default(2.75f)->as_number()+CableRadius;
	ModelParameter.Roller2Distance = config->value(roller2_distance_checksum)->by_default(6.0f)->as_number();
	float InitialHeight = config->value(intitial_height_checksum)->by_default(-200.0f)->as_number();
	
	BaseToWorldTransform.buildUnit();
    WorldToBaseTransform.buildUnit();
	
    // initialize arm rest length from z offset
    Vector3 Offset(0,0,InitialHeight);
	for(int i=0; i<3; i++)
		ModelParameter.Arm[i].RestLength = 0.0f;
        
    Vector3 l;
    inverseKinematic(Offset, l, ModelParameter);    
    ModelParameter.Arm[0].RestLength = l.x;
	ModelParameter.Arm[1].RestLength = l.y;
	ModelParameter.Arm[2].RestLength = l.z;
    
    // set z offset
    WorldToBaseTransform.setTranslation(Offset);
    setWorldToBaseTransform(WorldToBaseTransform);
    
    pInstance = this;
}
       
DeltaStriderSolution::~DeltaStriderSolution() {
    if(pInstance == this)
        pInstance = NULL;
}

/**
 * returns a pointer to the last instance that was created
 * this is useful if full access is required from somewhere else
 * @return  returns NULL if no instance of this class exists
 */ 
DeltaStriderSolution* DeltaStriderSolution::getInstance() {
    return pInstance;
}

/**
 * inverse kinematic converting cartesian coordinates of the end effector to motor positions 
 */
void DeltaStriderSolution::cartesian_to_actuator(float cartesian_mm[], float actuator_mm[]) {
	Vector3 l;
	Vector3 p = Vector3(cartesian_mm[X_AXIS], cartesian_mm[Y_AXIS], cartesian_mm[Z_AXIS]);
    // transform p from world coordinates to base platform coordinates
	p = WorldToBaseTransform.transformPoint(p);

	if(inverseKinematic(p, l, ModelParameter) == false)
		THEKERNEL->streams->printf("ERROR: inverseKinematic has no solution\n");

	//THEKERNEL->streams->printf("l: %f, %f, %f\n", l.x, l.y, l.z);
    actuator_mm[ALPHA_STEPPER] = l.x;
    actuator_mm[BETA_STEPPER ] = l.y;
    actuator_mm[GAMMA_STEPPER] = l.z;
}

/**
 * forward kinematic converting motor positions to cartesian coordinates of the end effector 
 */
void DeltaStriderSolution::actuator_to_cartesian(float actuator_mm[], float cartesian_mm[]) {
 	Vector3 p;
	Vector3 l = Vector3(actuator_mm[X_AXIS], actuator_mm[Y_AXIS], actuator_mm[Z_AXIS]);
    forwardKinematic(l, p, ModelParameter);
 
    // transform p from base plattform coordinates to world coordinates
    p = BaseToWorldTransform.transformPoint(p);
	
	cartesian_mm[X_AXIS] = p.x;
    cartesian_mm[Y_AXIS] = p.y;
    cartesian_mm[Z_AXIS] = p.z;
}

/**
 * Inverse kinematic for cable actuated delta robot. Computes the cable lengths
 * relative to rest length at the home position for a given end effector position
 * @param p      	end effector position in basis platform coordinate system
 * @param l			result vector with the three cable lengths
 * @param K			global kinematic parameter
 * @return 			true if a solution exists false otherwise
 */
bool DeltaStriderSolution::inverseKinematic(Vector3 p, Vector3& l, DeltaStriderModelParameter& K) {
    if(p.z>0)
        return false;

    bool Result = true;
    Result &= inverseArmKinematic(Vector3(p.x                  -K.BaseRadiusMinusEndeffectorRadius, p.y, p.z),                   l.x, K, 0);
    Result &= inverseArmKinematic(Vector3(p.x*cos120-p.y*sin120-K.BaseRadiusMinusEndeffectorRadius, p.y*cos120+p.x*sin120, p.z), l.y, K, 1);
    Result &= inverseArmKinematic(Vector3(p.x*cos120+p.y*sin120-K.BaseRadiusMinusEndeffectorRadius, p.y*cos120-p.x*sin120, p.z), l.z, K, 2);

    return Result;
}

/**
 * Inverse kinematic for a single arm 
 * @param p  end effector position in arm coordinate system (arm in XZ plane, origin at shoulder joint)
 * @param l	 result cable lengths relative to rest length
 * @param K	 kinematic parameter for arm
 * @return 	 true if a solution exists false otherwise
 */
bool DeltaStriderSolution::inverseArmKinematic(Vector3 p, float& l, DeltaStriderModelParameter& K, int ArmIndex) {
	//THEKERNEL->streams->printf("ARM %i\n", ArmIndex);
    l = computeRelativeCableLength(p, K) - K.Arm[ArmIndex].RestLength;
    return true;
}

/**
 * Forward kinematic for cable actuated delta robot. Computes the endeffector position given the three cable lengths
 * relative to rest length at the home position. This method might be slow
 * @param l	        cable length (relative to rest length) for each arm
 * @param p      	result end effector position
 * @param K			kinematic model parameter
 * @return			true if a solution exists false otherwise
 */
bool DeltaStriderSolution::forwardKinematic(Vector3 l, Vector3& p, DeltaStriderModelParameter& K) {
    float r = K.BaseRadiusMinusEndeffectorRadius;
    const float DampingFactor = 0.5f;
    const float SqrTargetAccuracy = powf(0.0004f, 2);
    const int MaxIterations = 200;

    // compute upper joint position on base platform
    Vector3 P1 = Vector3(r, 0, 0);
    Vector3 P2 = Vector3(r*cos120, -r*sin120, 0);
    Vector3 P3 = Vector3(r*cos120,  r*sin120, 0);
    
    // set start solution for p to origin
    p = Vector3(0,0,-100);
    
    // use an iterative approximation through the inverse kinematic to find p for the given cable lengths
    for(int i=0; i<MaxIterations; i++) {
        // compute cable length for current solution
        Vector3 lc;
        if(inverseKinematic(p, lc, K) == false) {
            return false;
		}
        
        if((l-lc).magsq()<SqrTargetAccuracy)
            return true;
        
        // compute cable directions
        Vector3 d1 = (P1-p).normalized();
        Vector3 d2 = (P2-p).normalized();
        Vector3 d3 = (P3-p).normalized();
        
        // improve solution
		p -= (d1*(l.x-lc.x) + d2*(l.y-lc.y) + d3*(l.z-lc.z))*DampingFactor;
    }
    
    // target accuracy not reached
    THEKERNEL->streams->printf("ERROR: DeltaStriderSolution::forwardKinematic: Target accuracy not reached in %i iterations\n", MaxIterations);
    return false;
}

void DeltaStriderSolution::getModelParameter(DeltaStriderModelParameter& Parameter) {
    Parameter = ModelParameter;
}
       
void DeltaStriderSolution::setModelParameter(DeltaStriderModelParameter& Parameter) {
	ModelParameter = Parameter;
}

void DeltaStriderSolution::setBaseToWorldTransform(const Matrix4x4& Transform) {
    Matrix4x4 InvTransform = Transform;
    if(InvTransform.invert() == false)
        return;
        
    BaseToWorldTransform = Transform;
    WorldToBaseTransform = InvTransform;
}

 void DeltaStriderSolution::setWorldToBaseTransform(const Matrix4x4& Transform) {
    Matrix4x4 InvTransform = Transform;
    if(InvTransform.invert() == false)
        return;
        
    WorldToBaseTransform = Transform;
    BaseToWorldTransform = InvTransform;
 }
 
 void DeltaStriderSolution::getBaseToWorldTransform(Matrix4x4& Transform) {
	Transform = BaseToWorldTransform;
 }
 
 void DeltaStriderSolution::getWorldToBaseTransform(Matrix4x4& Transform) {
	Transform = WorldToBaseTransform;
 }

/**
 * computes the deflected cable length.
 * @param r 		radius of the deflection roller
 * @param P			Endpoint of the cable in 2D roller coordinate system (center of roller is the origin)
 * @param T			point where the cable touches the deflection roller
 * @param alpha		deflection angle
 */
void DeltaStriderSolution::computeCableDeflection(float r, Vector2 P, Vector2& T, float& alpha) {
    float x2 = P.x*P.x; 	// P.x²
    float y2 = P.y*P.y;		// P.y²
    float r2 = r*r;			// r²
    float r2x2 = r2*x2;		// r²x²

	// compute point T where the cable touches the deflection roller
	float SqrRoot = sqrtf(r2x2*(x2+y2-r2));
//	T.x = (r2x2-P.y*SqrRoot)/(P.x*(x2+y2));
// 	T.y = (SqrRoot+r2*P.y)/(x2+y2);
	T.x = -(r2x2-P.y*SqrRoot)/(P.x*(x2+y2));
	T.y = -(SqrRoot+r2*P.y)/(x2+y2);
  
    // compute deflection angle
    alpha = atan2f(T.x, T.y);
}

/**
 * computes the cable length offset caused by the horizontal deflection of the cable on the winch during wind up
 * with increased wind up the cable is routed non perpendicular to the winch axis causing an slight offset in cable length
 */
void DeltaStriderSolution::computeCableWindingOffset(float l) {
	
}

/**
 * computes the relative cable length from the winch to a given point P int arm coordinate system
 * @param 		P point in arm coordinate system
 * @param 		K kinematic Parameter set
 */
float DeltaStriderSolution::computeRelativeCableLength(Vector3 P, DeltaStriderModelParameter& K) {
//	THEKERNEL->streams->printf("   P: %f,%f,%f\n", P.x, P.y, P.z);

    // kinematic model constants
    const float r1 = K.Roller1Radius;	    // deflection roller radius 1 in mm
    const float r2 = K.Roller2Radius; 	    // deflection roller radius 2 in mm
    const float rd2 = K.Roller2Distance; 	// distance of second deflection roller from T1 in mm
    
    // compute deflection on first roller
    float alpha1;	// deflection angle on first roller
    Vector2 T1;		// point where cable touches first roller
    computeCableDeflection(r1, Vector2(P.x, P.z), T1, alpha1);
//	THEKERNEL->streams->printf("Px: %f Pz: %f  /  T1: %f,%f   /   Alpha1: %f\n", P.x, P.z, T1.x, T1.y, alpha1*180.0f/3.1415926f);
    
    // compute deflection on second roller
    float alpha2;	// deflection angle on second roller
    Vector2 T2;		// point where cable touches second roller
    Vector2 D(T1.x-P.x, T1.y-P.z);
    Vector2 Q = Vector2(D.mag()-rd2,  r2-fabs(P.y));
    computeCableDeflection(r2, Q, T2, alpha2);
	alpha2 = fmod(alpha2+3.14159265f, 2*3.14159265f);
//	THEKERNEL->streams->printf("Qx: %f Qy: %f  /  T2: %f,%f   /   Alpha2: %f\n", Q.x, Q.y, T2.x, T2.y, alpha2*180.0f/3.1415926f);

    // compute final relative cable length (from upper most point of deflection roller 1 to cable joint on end effector)
    return (alpha1*r1 + alpha2*r2 + rd2 + (Q-T2).mag()); 
}

/**
 * Find the intersection points of three spheres. 
 * Implementaton based on Wikipedia Trilateration article.                
 * @param p1,p2,p3 	sphere centers
 * @param r1,r2,r3 	sphere radii
 * @param i1,i2 	the two intersection points
 * @return			true if intersection points exist, false otherwise
 */ 
bool DeltaStriderSolution::trilaterate(Vector3 p1, Vector3 p2, Vector3 p3, float r1, float r2, float r3, Vector3& i1, Vector3& i2) {
	  // compute ex
	  Vector3 temp1 = p2-p1;
	  float d = temp1.mag();
	  Vector3 e_x = temp1*(1.0/d); 

	  // compute ey
	  Vector3 temp2 = p3-p1;
	  float i = e_x|temp2;
	  Vector3 e_y = (temp2 - e_x*i).normalized();
	  float j = e_y|temp2;

	  // compute ez
	  Vector3 e_z = e_x % e_y;                                                                

	  float x = (r1*r1 - r2*r2 + d*d) / (2*d);                
	  float y = (r1*r1 - r3*r3 -2*i*x + i*i + j*j) / (2*j);      
	  float temp4 = r1*r1 - x*x - y*y;                          
	  if(temp4<0)
		return false;

	  Vector3 dz = e_z*sqrt(temp4);
	  Vector3 b = p1 + e_x*x + e_y*y;
	  i1 = b+dz;               
	  i2 = b-dz; 

	  return true;
}