// author: manuel scholz

#include "libs/Kernel.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"

//*** FUNCTION ************************************************************************************

/**
 * prints a matrix to console
 */
void printMatrix(float* A, int m, int n) {
    for(int i=0; i<m; i++) {
		Kernel::instance->streams->printf("    ");
		for(int j=0; j<n; j++)
            Kernel::instance->streams->printf("%10.5f ", A[i*n+j]);
		Kernel::instance->streams->printf("\n");
    }
	Kernel::instance->streams->printf("\n");
}

/**
 * solves linear equation system Ax = b for x using LU-decomposition
 * where A is a square n x n matrix and x and b are n dimensional vectors
 */
void linearSolveLU(float* A, float* b, float* x, int n) {
    // create temporary matrices and vectors
    float* L = new float[n*n];
    float* U = new float[n*n];
    float* z = new float[n];
	
    // initialize matrices
    for(int i=0; i<n; i++) {
        for(int j=0; j<n; j++) {
            U[i*n+j] = 0;
            L[i*n+j] = int(i==j);
        }
    }
		
    //Ax=b, decomposition of A into L and U    
    for(int i=0; i<n; i++) {
        for(int j=0; j<n; j++) {
            float sum = 0;
            if(i<=j) {
                for(int k=0; k<n; k++)
					if(k!=i)
						sum += L[i*n+k] * U[k*n+j];
                U[i*n+j] = A[i*n+j]-sum;
            } else {
                for(int k=0; k<n; k++)
                    if(k!=j)
                        sum += L[i*n+k] * U[k*n+j];
                L[i*n+j] = (A[i*n+j]-sum) / U[j*n+j];
            }
        }
    }

    //now LUx=b, i.e Lz=b obtaining Z by forward substitution
    z[0]=b[0]/L[0*n+0];
    for(int i=1; i<n; i++) {
        float sum=0;
        for(int j=0; j<i; j++)
            sum += z[j]*L[i*n+j];
        z[i]=b[i]-sum;
    }

    //now, Ux=z Finding X by backward substitution
    x[n-1] = z[n-1] / U[n*n-1];
    for(int i=n-2; i>=0; i--) {
        float sum=0;
        for(int j=n-1;j>i;j--)
            sum += x[j]*U[i*n+j];
        x[i]=(z[i]-sum)/U[i*n+i];
    }
	
    // clean up
    delete[] L;
    delete[] U;
    delete[] z;
}

// check if Ax = b
float checkLinearSolver(float* A, float* b, float* x, int n) {
    float* Ax = new float[n];
    for(int i=0; i<n; i++) {
        Ax[i] = 0;
        for(int k=0; k<n; k++)
            Ax[i] += A[n*i+k] * x[k];
    }

    float error = 0;
    for(int i=0; i<n; i++) {
        float e = Ax[i]-b[i];
        error += e*e;
    }

    return error/n;
}
