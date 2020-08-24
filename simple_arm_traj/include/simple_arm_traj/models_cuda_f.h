/*----------------------------------------------------------
Adrien Durand-Petiteville
Universidade Federal de Pernambuco
April 2019
----------------------------------------------------------*/

#ifndef MODELS_CLASS_CUDA_F_H
#define MODELS_CLASS_CUDA_F_H

#include <iostream>
#include <math.h>
#include <stdint.h>
#include <time.h>
#include <iomanip>

// Solver
//////////////////////////////#include <nlopt.h>

// User
// #include "models.h"

// Cuda
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

using namespace std;

///////////////////////////////////
typedef struct {
    int nbPrediction;// to erease!!
    int nbFeatures;// to erease!!
    float ts;// to erease!!
    float dx;// to erease!!
    float focal;// to erease!!
    float* parameters;

    // float* v_X; // to erease!!
    // float* v_Y; // to erease!!
    // float* v_z; // to erease!!
    float* v_XYz;
    float* v_Xref; // to erease!!
    float* v_Yref; // to erease!!
    float theta;
    float thetaP;

    float* h_command;
    float* h_commandDelta;
    float* h_predictedState;
    float* h_Cgrad;

    int iteration;
    int iteration_grad;
    float timeGrad;
    // float timeGradcuda;
    float timeFunction;
    // float timeFunctioncuda;
    // float timeEq;

    float* d_command;
    float* d_command2;
    float* d_commandDelta;
    float* d_commandEqDelta;
    float* d_param;
    float* d_XYz;
    float* d_XYref;
    float* d_thetaP;
    float* d_C;
    float* d_Cgrad;

    float* d_CMat;

    float* d_deltaTTP;
    float* d_deltaXY;

    bool finalConstraint;
    double C_finalConstraint;
    double* h_G_finalConstraint;

} xyz_model_data_cuda_f;

void setupModelDataCuda_f(xyz_model_data_cuda_f* data, int nbPrediction, int nbFeatures, float ts, float dx, float focal, float* XYref);

double xyz_model_wrap_cuda_f(unsigned n, const double *x, double *grad, void *my_func_data);

double xyz_constraint_cuda_f_wrap(unsigned n, const double *x, double *grad, void *my_func_data);

void equivalentSpeed(int nbPred, float* speedIn, float* speedOut, float theta, float thetaP, float dx, float ts);

void checkEqSpeed(int nbPred, float* speed, float* speedEq, float x, float y, float theta, float thetaP, float ts);

__global__
void xyz_model_cuda(float* C, float* command, float* XY, float* XYstar, float* param, float thetaP, int nbFeat);

__global__
void xyz_gradient_cuda(float* C, float* command, float* XY, float* XYstar, float* param, float thetaP);

__global__
void xyz_gradient_cuda2(float* C, float* command, float* XY, float* XYstar, float* param, float thetaP);

// --------------------------------------------------

__global__
void equivalentSpeed_cuda(int nbPred, float* speedIn, float* speedOut, float thetaI, float thetaPI, float dx, float ts);

// --------------------------------------------------
__global__
void computeDeltaTheta(float* speed, float* deltaTTP, int offset, int nbPrediction, float ts, float delta, float thetaI, float thetaPI);

__global__
void sumDelta(float* delta, int nbPrediction);

__global__
void sumC(float* C, float* Cmat, int nbPrediction, int nbFeatures);

__global__
void sum_finalconstraint(float* C, float* Cmat, int nbPrediction, int nbFeatures);

__global__
void computeDeltaXY(float* deltaXY, float* speed, float* deltaTTP, int offset, int nbPrediction, float ts, float dx, float delta);

__global__
void eqSpeed_cuda(float* speedOut, float* deltaXY, int offsetXY, float* deltaTTP, int offsetTTP, int nbPrediction, float dx, float ts);

__global__
void zeros_cuda(float* vec, int size);

#endif //MODELS_CLASS_CUDA_F_H
