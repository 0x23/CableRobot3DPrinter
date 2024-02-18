#ifndef DELTASTRIDERSOLUTION_H
#define DELTASTRIDERSOLUTION_H

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "BaseSolution.h"
#include "libs/nuts_bolts.h"
#include "libs/Config.h"
#include "libs/Math/Vector3.h"
#include "libs/Math/Vector2.h"
#include "libs/Math/Matrix4x4.h"

//--- DeltaStriderArmParameter --------------------------------------------------------------------

/**
 * arm model parameter, all units in mm and radians
 */
struct DeltaStriderArmParameter {
    DeltaStriderArmParameter();

    float RestLength;                           //cable length at motor zero position
};

//--- DeltaStriderModelParameter ------------------------------------------------------------------

/**
 * kinematik model parameter, all units in mm and radians
 * an explanation of the single parameters can be found in the delta strider kinematics documentation
 */
class DeltaStriderModelParameter {	
    public:
        DeltaStriderModelParameter();
        void print();
		
        float BaseRadiusMinusEndeffectorRadius;
        float Roller1Radius;
        float Roller2Radius;
        float Roller2Distance;
        
        DeltaStriderArmParameter Arm[3];
};

//--- DeltaStriderSolution ------------------------------------------------------------------------

/**
 * arm solution for delta strider robot
 * a delta strider is 'homed' by finding the arm rest lengths when motor positions are zero
 */
class DeltaStriderSolution : public BaseSolution {
    public:
        DeltaStriderSolution();
        DeltaStriderSolution(Config* passed_config);
        virtual ~DeltaStriderSolution();
        static DeltaStriderSolution* getInstance();  
        
        void cartesian_to_actuator(float cartesian_mm[], float actuator_mm[]);
        void actuator_to_cartesian(float actuator_mm[], float cartesian_mm[]);
              
        bool inverseKinematic(Vector3 p, Vector3& l, DeltaStriderModelParameter& K);
        bool forwardKinematic(Vector3 l, Vector3& p, DeltaStriderModelParameter& K);
        
        void getModelParameter(DeltaStriderModelParameter& Parameter);
        void setModelParameter(DeltaStriderModelParameter& Parameter);
		void setBaseToWorldTransform(const Matrix4x4& Transform);
        void setWorldToBaseTransform(const Matrix4x4& Transform);
		void getBaseToWorldTransform(Matrix4x4& Transform);
        void getWorldToBaseTransform(Matrix4x4& Transform);
        
    private:
        bool  inverseArmKinematic(Vector3 p, float& l, DeltaStriderModelParameter& K, int ArmIndex);
        void  computeCableDeflection(float r, Vector2 P, Vector2& T, float& alpha);
		void  computeCableWindingOffset(float l);
        float computeRelativeCableLength(Vector3 P, DeltaStriderModelParameter& K);
        bool  trilaterate(Vector3 p1, Vector3 p2, Vector3 p3, float r1, float r2, float r3, Vector3& i1, Vector3& i2);
        
        Config* config;
        Matrix4x4 WorldToBaseTransform;
        Matrix4x4 BaseToWorldTransform;
        DeltaStriderModelParameter ModelParameter;
		
        static DeltaStriderSolution* pInstance;             // allows to access the solution object from somewhere else
};

#endif
