
#ifndef DELTASTRIDERCALIB_H_
#define DELTASTRIDERCALIB_H_

//*** INCLUDE ************************************************************************************* 

#include <vector>

#include "libs/Module.h"
#include "Vector3.h"
#include "Matrix4x4.h"
#include "optimization.h"
#include "Touchprobe.h"
#include "DeltaStriderSolution.h"

//*** CLASS ***************************************************************************************

struct PlaneParameter {
    Vector3 Normal;
    float Distance;
};

//--- DeltaStriderModelOptimizer ------------------------------------------------------------------

/**
 * the model function seems to have local minima so good initial values are required (especailly for the plane distance)
 * the plane distance parameter were also decoupled from the arm rest lengths parameter. This speeds up convergence.
 */


class DeltaStriderModelOptimizer : public CLeastSquareProblem {
    public:
        DeltaStriderModelOptimizer(DeltaStriderSolution* pArmSolution);
        void optimize(std::vector<Vector3>& PlaneMotorPositionMeasurements, DeltaStriderModelParameter& ModelParameter, PlaneParameter& Plane, int OptIterationCount);
     
    protected:   
        int  getParameterCount() override;
        int  getResidualElementCount() override;
        void computeResidualVector(float* ParameterVector, float* ResidualVector) override;
		void onIterationBegin(int Itertaion, float* ParameterVector, bool* ParameterLocks) override;
        void onIterationEnd(int Itertaion, float RMSError, float Improvement, float* ParameterVector) override;

    private:
        void encodeParameterVector(float* pParameters, DeltaStriderModelParameter& ModelParameter, PlaneParameter& Plane);
        void decodeParameterVector(float* pParameters, DeltaStriderModelParameter& ModelParameter, PlaneParameter& Plane);
   
        DeltaStriderSolution* pArmSolution;
		PlaneParameter InitialPlaneParameter;
        DeltaStriderModelParameter InitialModelParameter;
        DeltaStriderModelParameter IterationModelParameter;
        std::vector<Vector3> PlaneMotorPositionMeasurements;
};

//--- DeltaStriderCalibration ---------------------------------------------------------------------

class DeltaStriderCalibration : public Touchprobe {
    public:
        DeltaStriderCalibration();

        void on_module_loaded() override;
        void on_gcode_received(void* argument) override;
        void on_config_reload(void* argument);
        
        bool doCalibration(float MoveSpeed, float ProbeSpeed, float MaxProbeRadius, int ProbePointCount, int OptIterationCount);
    
    protected:
        bool      doCalibrationMeasurements(std::vector<Vector3>& MotorPositionMeasurements, std::vector<Vector3>& ProbePoints, float MoveSpeed, float ProbeSpeed);
        void 	  synchronizeToolPosition();
        Matrix4x4 computeWorldToBasisTransform(PlaneParameter ProbePlane);
        void      computeProbePoints(std::vector<Vector3>& ProbePoints, float MaxProbeRadius, int ProbePointCount);

    private:
        unsigned int CalibrationMCode;
        unsigned int ToolZeroMCode;
};

//*** FUNCTION ************************************************************************************

bool writePointPlotFile(const char* Filename, const std::vector<Vector3>& Points);

//*************************************************************************************************

#endif