// author: manuel scholz

//*** INCLUDE *************************************************************************************

#include "Optimization.h"
#include "Linearsolver.h"
#include "fastmath.h"
#include "stdlib.h"

#include <string.h>

#include "libs/Kernel.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"

//*** CLASS ***************************************************************************************

/**
* computes the jacobi matrix of the residual (the derivative of the system) at the given location in parameter space
* @param ParameterVector	parameter vector defining the location at which the jacobi matrix is computed
* @param h			        step size for finite differences
* @param JacobiMatrix		output jacobi matrix of the residual. Must be an n x m matrix where n=getParameterCount() and m=getResidualElementCount()
* @param ResidualVector		output residual vector at location defined by ParameterVector
*/
void CLeastSquareProblem::computeResidualJacobiMatrix(float* ParameterVector, float h, float* JacobiMatrix, float* ResidualVector) {
    // jacobimatrix must have n rows, m cols
	int n = getParameterCount();		// number of parameter
    int m = getResidualElementCount();	// number of measurements
    // create parameter vector
    float* r0 = ResidualVector;	        // residual vector for given parameter vector
    float* r = new float[m];	        // residual vector for modified parameter vector

    // compute residual at initial dParameterVector
    computeResidualVector(ParameterVector, r0);

    // iterate over parameters
    for(int j=0; j<n; j++) {
        // backup parameter value
        float tmp = ParameterVector[j];

        // get residual vector after parameter j was changed
        ParameterVector[j] += h;
        computeResidualVector(ParameterVector, r);
        ParameterVector[j] = tmp;

        // fill matrix with finite differences
        for(int i=0; i<m; i++) {
            JacobiMatrix[i*n+j]  = (r[i] - r0[i])/h;
		}
			
		THEKERNEL->call_event(ON_IDLE);	
    }
    
    // clean up
    delete[] r;
}

/**
* Performs a Levenberg Marquard optimization of the given least squares system. 
* The method alters the given parameters so that the squared residual length is minimized
* @param Parameter				vector of initial parameters of length 'lsp->getParameterCount()'. Will contain the result parameters after optimization.
* @param lsp					a least square problem
* @param MaxIterations			maximum number of optimization iterations
* @param ImprovementThreshold 	when rms improvement (in percent) between successive iteration falls below this value the optimization is stopped
* @return			    		error after optimization (rms)
*/
float CLeastSquareProblem::optimizeLevMar(float* Parameter, int MaxIterations, float ImprovementThreshold) {
    const float lambdaMax = 1;		    // 
    const float lambdaMin = 0.0001;		// 
    float h = 0.01;		    			// step size for numeric differentiation

    int n = getParameterCount();		// number of parameters
    int m = getResidualElementCount();	// number of measurements

    float lambda = 1.0;					// 
    float dSqrResidualLength = 1.0e10;	//
	float lastRms = 100;
	
    float* pold	= new float[n];		// old parameter delta	
    float* dp	= new float[n];		// parameter delta	
    float* r	= new float[m];		// residual vector
    float* J	= new float[m*n];	// Jacobi matrix of residual vector
    float* b	= dp;				// right side of linear equation system	(reuse memory of dp)
    float* A	= new float[n*n];	// matrix of the left side of linear equation system (Ax = b)
    bool* lock	= new bool[n];		// holds a lock flack for each parameter
	
	for(int i=0; i<n; i++)
		lock[i] = false;
  
    float FilteredImprovement = 100;         // filtered improvement 
    for(int a=0; a<MaxIterations; a++) {
	    onIterationBegin(a, Parameter, lock);
	
		// this improves convergence dramatically
		//h = min(lambda, 0.1f);
		
        // compute Jacobi matrix of residual and residual for current parameter set
        computeResidualJacobiMatrix(Parameter, h, J, r);
		
		//Kernel::instance->streams->printf("lambda = %.16f\n", lambda);			
		//Kernel::instance->streams->printf("Jacobi Matrix:\n");
		//printMatrix(J, n, n);
		//THEKERNEL->call_event(ON_IDLE);	
			
        // compute new residual length
        float l = 0;
        for(int i=0; i<m; i++)
            l += r[i]*r[i];

        // check if we got a better solution
		bool Backstepped = false;
        if(l <= dSqrResidualLength) {
            // update lambda
            if(lambda>lambdaMin)
				lambda /= 2;
            dSqrResidualLength = l;
        } else {
            // adjust lambda
            if(lambda<lambdaMax)
				lambda *= 17;
			
            // restore old parameter set
            memcpy(Parameter, pold, sizeof(float)*n);
            // recompute Jacobi matrix of residual and residual for current parameter set
            computeResidualJacobiMatrix(Parameter, h, J, r);
			Backstepped = true;	
        }

        // A = J-Transposed x J 
        for(int i=0; i<n; i++) {
            for(int j=0; j<n; j++) {
                A[n*i+j] = 0;
                for(int k=0; k<m; k++)
	                A[n*i+j] += J[n*k+i] * J[n*k+j];
            }
        }
	
        // add diag(A)*lambda to A
        for(int i=0; i<n; i++)
            A[n*i+i] += A[n*i+i]*lambda;
		
        // compute J-Transposed x r
        for(int i=0; i<n; i++) {
            b[i] = 0;
            for(int k=0; k<m; k++)
                b[i] += J[n*k+i] * r[k];
        }
		
        // solve linear equation system A*dp = b
        linearSolveLU(A, b, dp, n);

		// backup old parameters set
        memcpy(pold, Parameter, sizeof(float)*n);
		
		// compute h
		h = 0;
        for(int i=0; i<n; i++) {
			if(lock[i] == false)
				Parameter[i] -= dp[i];
			h += dp[i]*dp[i];
		}
		h = sqrtf(h)*0.5f;		
		
		float rms = sqrtf(dSqrResidualLength/m);
		float Improvement = fabs((lastRms-rms)/rms)*100;
		if(!Backstepped)
			FilteredImprovement = FilteredImprovement*0.6f + Improvement*0.4f;
        onIterationEnd(a, sqrtf(dSqrResidualLength/m), Backstepped ? 0 : Improvement, Parameter);
		
		if(FilteredImprovement < ImprovementThreshold*0.01f && Backstepped==false)
			break;
		lastRms = rms;
    }

    // clean up
    delete[] pold;
    delete[] dp;	// parameter delta	
    delete[] r;	// residual vector
    delete[] J;	// Jacobi matrix of residual vector
    // delete[] b;	// right side of linear equation system	
    delete[] A;	// matrix of the left side of linear equation system (Ax = b)
    delete[] lock;
	
    return sqrtf(dSqrResidualLength/m);
}

//*************************************************************************************************
