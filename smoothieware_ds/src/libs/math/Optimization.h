// author: manuel scholz

#ifndef LEASTSQUARES_OPTIMIZATION_H
#define LEASTSQUARES_OPTIMIZATION_H

//*** CLASS ***************************************************************************************

class CLeastSquareProblem {
  public:
    // optimizes the least squares problem with the levenberg-marquard optimization method, returns final rms error
    float         optimizeLevMar(float* Parameter, int MaxIterations, float ImprovementThreshold = 0.5f);

    // returns number of system parameter
    virtual int	  getParameterCount() = 0;
    // returns the size of the residual vector (often the number of measurements)
    virtual int	  getResidualElementCount() = 0;
    // computes the residual vector (e.g. differences between model and measurements). The entries must not be squared ! e.g. store distances not squared distances )
    virtual void  computeResidualVector(float* ParameterVector, float* ResidualVector) = 0;
   // called before each iteration
    virtual void  onIterationBegin(int Itertaion, float* ParameterVector, bool* ParameterLocks) {};
    // called after each iteration, parameter constraints can be enforced here. also useful for user feedback
    virtual void  onIterationEnd(int Itertaion, float RMSError, float Improvement, float* ParameterVector) {};
 
  protected:
    void computeResidualJacobiMatrix(float* ParameterVector, float h, float* mJacobiMatrix, float* Residual);
};

#endif
