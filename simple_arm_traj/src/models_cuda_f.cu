// /*----------------------------------------------------------
// Adrien Durand-Petiteville
// Universidade Federal de Pernambuco
// June 2019
// ----------------------------------------------------------*/

#include "../include/simple_arm_traj/models_cuda_f.h"

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
void setupModelDataCuda_f(xyz_model_data_cuda_f* func_data_cuda, int nbPrediction, int nbFeatures, float ts, float dx, float focal, float* h_XYref)
{

  // Allocate host memory
  // func_data_cuda->v_X = new float[nbFeatures];
  // func_data_cuda->v_Y = new float[nbFeatures];
  // func_data_cuda->v_z = new float[nbFeatures];
  func_data_cuda->v_XYz = new float[3*nbFeatures];
  func_data_cuda->parameters = new float[5];
  func_data_cuda->h_command = new float[3*nbPrediction];
  func_data_cuda->h_Cgrad = new float[3*nbPrediction];
  func_data_cuda->h_commandDelta = new float[3*nbPrediction*3*nbPrediction];
  func_data_cuda->h_predictedState = new float[4*(nbPrediction+1)];

  func_data_cuda->h_G_finalConstraint = new double[3*nbPrediction];

  // Copy host data
  func_data_cuda->nbPrediction = nbPrediction;
  func_data_cuda->nbFeatures = nbFeatures;
  func_data_cuda->ts = ts;
  func_data_cuda->dx = dx;
  func_data_cuda->focal = focal;
  func_data_cuda->parameters[0] = nbPrediction;
  func_data_cuda->parameters[1] = nbFeatures;
  func_data_cuda->parameters[2] = ts;
  func_data_cuda->parameters[3] = dx;
  func_data_cuda->parameters[4] = focal;

  // Allocate device memory
  cudaMalloc(&func_data_cuda->d_command, 3*nbPrediction*sizeof(float));
  cudaMalloc(&func_data_cuda->d_command2, 3*nbPrediction*sizeof(float));
  cudaMalloc(&func_data_cuda->d_param, 5*sizeof(float));
  cudaMalloc(&func_data_cuda->d_XYz, 3*nbFeatures*sizeof(float));
  cudaMalloc(&func_data_cuda->d_XYref, 2*nbFeatures*sizeof(float));
  cudaMalloc(&func_data_cuda->d_thetaP, sizeof(float));

  // cudaMalloc(&func_data_cuda->d_C, sizeof(float));
  cudaMalloc(&func_data_cuda->d_C, nbPrediction*nbFeatures*sizeof(float));

  cudaMalloc(&func_data_cuda->d_Cgrad, 3*nbPrediction*sizeof(float));
  cudaMalloc(&func_data_cuda->d_commandDelta, 3*nbPrediction*3*nbPrediction*sizeof(float));
  cudaMalloc(&func_data_cuda->d_commandEqDelta, 3*nbPrediction*3*nbPrediction*sizeof(float));

  cudaMalloc(&func_data_cuda->d_deltaTTP, 2*(nbPrediction+1)*(nbPrediction+1)*sizeof(float));
  cudaMalloc(&func_data_cuda->d_deltaXY, 6*nbPrediction*nbPrediction*sizeof(float));

  cudaMalloc(&func_data_cuda->d_CMat, 3*nbPrediction*nbPrediction*nbFeatures*sizeof(float));

  // Copy device data
  cudaMemcpy(func_data_cuda->d_param, func_data_cuda->parameters, 5*sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(func_data_cuda->d_XYref, h_XYref, 2*nbFeatures*sizeof(float), cudaMemcpyHostToDevice);

}

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
double xyz_model_wrap_cuda_f(unsigned n, const double *x, double *grad, void *my_func_data)
{

    // Cost function
    double h_C = 0.0;

    // Get the data from the structure
    xyz_model_data_cuda_f *d = (xyz_model_data_cuda_f *) my_func_data;

    // Count the number of calls
    d->iteration = d->iteration + 1;

const clock_t begin_time_func = clock();

    // Cast the command vector
    float* xx = new float[3*d->nbPrediction];
    for(int idx = 0; idx < 3*d->nbPrediction;idx++){xx[idx] = (float)x[idx];}

// cout << xx[0] << " " << xx[1] << " " << xx[2] << endl;

    // Compute the equivalent speeds
    equivalentSpeed(d->nbPrediction, xx, d->h_command, d->theta, d->thetaP, d->dx, d->ts);

    // Copy the data from host to device
    cudaMemcpy(d->d_command, d->h_command, 3*d->nbPrediction*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d->d_XYz, d->v_XYz, 3*d->nbFeatures*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d->d_thetaP,&d->thetaP, sizeof(float), cudaMemcpyHostToDevice);

    // Parallel computing of the cost function
int threadDim_model = d->nbFeatures*d->nbPrediction; // TBDC for a large number of features
// const clock_t begin_time_func_cuda = clock();
    xyz_model_cuda<<<1,threadDim_model>>>(d->d_C, d->d_command, d->d_XYz, d->d_XYref, d->d_param, d->thetaP, d->nbFeatures);
float* h_CMat = new float[d->nbFeatures*d->nbPrediction];
cudaMemcpy(h_CMat, d->d_C, d->nbFeatures*d->nbPrediction*sizeof(float), cudaMemcpyDeviceToHost);
float C_f = 0.0;
for(int idx = 0; idx < d->nbFeatures*d->nbPrediction; idx++){C_f += h_CMat[idx];}
h_C = (double) C_f;

// if(isnan(h_C))
// {
    // cout << "C " << h_C << endl;
//     // for(int idx = 0; idx < d->nbFeatures*d->nbPrediction; idx++){cout << h_CMat[idx] << " ";}
//     // cout << endl;
//     for(int idx = 0; idx < 3*d->nbPrediction; idx++){cout << d->h_command[idx] << " ";}
//     cout << endl;
// }


// d->timeFunctioncuda += float( clock () - begin_time_func_cuda ) /  CLOCKS_PER_SEC*1000;

// cudaDeviceSynchronize();
d->timeFunction += float( clock () - begin_time_func ) /  CLOCKS_PER_SEC*1000;

/////////
// Compute the final constraint
if(d->finalConstraint)
{
  C_f = 0.0;
  for(int idx = d->nbFeatures*(d->nbPrediction-1); idx < d->nbFeatures*d->nbPrediction; idx++)
  {
    C_f += h_CMat[idx];
  }
  d->C_finalConstraint = (double)C_f;
}
/////////

    // Compute the gradient
    if (grad)
    {
      // Count the number of calls
      d->iteration_grad = d->iteration_grad + 1;


const clock_t begin_time_grad = clock();


      // Delta to compute the gradient
      float delta = 0.001;


// const clock_t begin_time_eq = clock();

cudaMemcpy(d->d_command2, xx, 3*d->nbPrediction*sizeof(float), cudaMemcpyHostToDevice);


int offsetTTP = (d->nbPrediction+1)*(d->nbPrediction+1);
int nb0 = d->nbPrediction*(d->nbPrediction + 1);
int thread0 = 1024;
int block0 = (nb0 / thread0) + 1;

computeDeltaTheta<<<block0,thread0>>>(d->d_command2, d->d_deltaTTP, offsetTTP, d->nbPrediction, d->ts, delta, d->theta, d->thetaP);

sumDelta<<<1,2*(d->nbPrediction+1)>>>(d->d_deltaTTP, d->nbPrediction);

int offsetXY = 3*d->nbPrediction*d->nbPrediction;
int nb1 = 3*d->nbPrediction*d->nbPrediction;
int thread1 = 1024;
int block1 = (nb1 / thread1) + 1;

computeDeltaXY<<<block1,thread1>>>(d->d_deltaXY, d->d_command2, d->d_deltaTTP, offsetXY, d->nbPrediction, d->ts, d->dx, delta);

//**
//
// float* debug2 = new float[6*d->nbPrediction*d->nbPrediction];
// cudaMemcpy(debug2, d->d_deltaXY, 6*(d->nbPrediction)*(d->nbPrediction)*sizeof(float), cudaMemcpyDeviceToHost);
//
// for(int i = 0; i < 6*(d->nbPrediction)*(d->nbPrediction); i++)
// {
//   cout << debug2[i] << " ";
//   if((i+1)%(d->nbPrediction) == 0){cout << endl;}
//   if((i+1)%(d->nbPrediction*d->nbPrediction) == 0){cout << endl;}
// }
//**


sumDelta<<<1,6*d->nbPrediction>>>(d->d_deltaXY, d->nbPrediction-1);

eqSpeed_cuda<<<block1,thread1>>>(d->d_commandEqDelta, d->d_deltaXY, offsetXY, d->d_deltaTTP, offsetTTP, d->nbPrediction, d->ts, d->dx);

// cudaDeviceSynchronize();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1

// d->timeEq += float( clock () - begin_time_eq ) /  CLOCKS_PER_SEC*1000;

int nb2 = 3*d->nbFeatures*d->nbPrediction*d->nbPrediction;
int thread2 = 1024;
int block2 = (nb2 / thread2) + 1;

// cudaDeviceSynchronize();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
// const clock_t begin_time_gradcuda = clock();
//       // xyz_gradient_cuda<<<gridDim_grad,threadDim_grad>>>(d->d_Cgrad, d->d_commandEqDelta, d->d_XYz, d->d_XYref, d->d_param, d->thetaP);
int thread3 = ceil((float)(3*d->nbPrediction)/32.0)*32;
// // cout << "asd "  << 3*d->nbPrediction << " " << thread3 << endl;
      zeros_cuda<<<1,thread3>>>(d->d_Cgrad, 3*d->nbPrediction);
//       // xyz_gradient_cuda<<<block2,thread2>>>(d->d_Cgrad, d->d_commandEqDelta, d->d_XYz, d->d_XYref, d->d_param, d->thetaP);
      xyz_gradient_cuda2<<<block2,thread2>>>(d->d_CMat, d->d_commandEqDelta, d->d_XYz, d->d_XYref, d->d_param, d->thetaP);

// float* pp = new float[3*d->nbFeatures*d->nbPrediction*d->nbPrediction];
// cudaMemcpy(pp, d->d_CMat, 3*d->nbFeatures*d->nbPrediction*d->nbPrediction*sizeof(float), cudaMemcpyDeviceToHost);


      sumC<<<1,3*d->nbPrediction>>>(d->d_Cgrad, d->d_CMat, d->nbPrediction, d->nbFeatures);
// cudaDeviceSynchronize();//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
// d->timeGradcuda += float( clock () - begin_time_gradcuda ) /  CLOCKS_PER_SEC*1000;

      // Retrieve the delta cost function
      cudaMemcpy(d->h_Cgrad, d->d_Cgrad, 3*d->nbPrediction*sizeof(float), cudaMemcpyDeviceToHost);

      // Compute the gradient
      for(int idx = 0; idx < 3*d->nbPrediction; idx++)
      {
        grad[idx] = (double)((d->h_Cgrad[idx] - h_C)/delta);

// if(isnan(grad[0]))
// {
// printf("%f %f  %f %f %f %f\n", grad[0], grad[1], grad[2], grad[3], grad[4], grad[5]);
// printf("A: %f %f %f %f %f\n", d->h_Cgrad[idx], h_C, delta, (d->h_Cgrad[idx] - h_C), (d->h_Cgrad[idx] - h_C)/delta);
// // printf("A: ");
// // for(int ddd = 0; ddd < 3*d->nbPrediction; ddd++)
// // {
// //   printf("%f ", d->h_Cgrad[ddd]);
// // }
// // printf("\n");
//
// }
      }

///////////////////////////////////
// Gradient of the final constraint
if(d->finalConstraint)
{
  zeros_cuda<<<1,thread3>>>(d->d_Cgrad, 3*d->nbPrediction);
  sum_finalconstraint<<<1,3*d->nbPrediction>>>(d->d_Cgrad, d->d_CMat, d->nbPrediction, d->nbFeatures);
  // Retrieve the delta cost function
  cudaMemcpy(d->h_Cgrad, d->d_Cgrad, 3*d->nbPrediction*sizeof(float), cudaMemcpyDeviceToHost);

  // Compute the gradient
  for(int idx = 0; idx < 3*d->nbPrediction; idx++)
  {
    d->h_G_finalConstraint[idx] = (double)((d->h_Cgrad[idx] - d->C_finalConstraint)/delta);
  }
}

d->timeGrad += float( clock () - begin_time_grad ) /  CLOCKS_PER_SEC*1000;
///////////////////////////////////
    }// If grad

  return h_C;

}// myfunc

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
double xyz_constraint_cuda_f_wrap(unsigned n, const double *x, double *grad, void *my_func_data)
{
  // Cast
  xyz_model_data_cuda_f *d = (xyz_model_data_cuda_f *) my_func_data;

  // Retrieve the final constraint grad
  if (grad)
  {
    for(int idx = 0; idx < 3*d->nbPrediction; idx++)
    {
      grad[idx] = d->h_G_finalConstraint[idx];
    }
  }

  // Retrieve the final constraint cost
  return d->C_finalConstraint;
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
void equivalentSpeed(int nbPred, float* speedIn, float* speedOut, float thetaI, float thetaPI, float dx, float ts)
{
  // Camera position
  float xci = dx*cos(thetaI);
  float yci = dx*sin(thetaI);
  float xc = xci;
  float yc = yci;

  // Platform orientation
  float theta = thetaI;
  float thetaP = thetaPI;

  // Robot control inputs
  float v, w, wp;

  // Constants
  float c1, c2;
  float DX, DY;

  for(int idxPred = 0; idxPred < nbPred; idxPred++)
  {
    // Retrieve the current commands
    v = speedIn[3*idxPred];
    w = speedIn[3*idxPred + 1];
    wp = speedIn[3*idxPred + 2];

    // Update the constants
    c1 = w*ts/2;
    c2 = (2*theta+w*ts)/2;

    // Update the camera position
    if(abs(w) > 0.0001)
    {
      xc += 2*v/w*sin(c1)*cos(c2) - 2*dx*sin(c1)*sin(c2);
      yc += 2*v/w*sin(c1)*sin(c2) + 2*dx*sin(c1)*cos(c2);
    }
    else
    {
      // cout << "straight" << endl;
      xc += v*cos(theta)*ts - w*dx*sin(theta)*ts;
      yc += v*sin(theta)*ts + w*dx*cos(theta)*ts;
    }

    // Update the orientation
    theta += w*ts;
    thetaP += wp*ts;

    // Compute the equivalent speeds
    DX = xc - xci;
    DY = yc - yci;

    speedOut[3*idxPred + 1] = 2/ts*atan2(-DX*sin(thetaI) + DY*cos(thetaI), 2*dx + DX*cos(thetaI) + DY*sin(thetaI));
    if(abs(speedOut[3*idxPred + 1]) < 0.000001)
    {
      speedOut[3*idxPred] = sqrt(pow(DX,2) + pow(DY,2)) / ts;
    }
    else
    {
      float rslt = pow(speedOut[3*idxPred + 1],2)*((pow(DX,2) + pow(DY,2)) /
                                 (4*pow(sin(speedOut[3*idxPred + 1]*ts/2),2)) - dx*dx);
      if(rslt < 0.000001)
      {
        speedOut[3*idxPred] = 0.0;
      }
      else
      {
        speedOut[3*idxPred] = sqrt(rslt);
      }

    }
    speedOut[3*idxPred + 2] = (thetaP - thetaPI + theta - thetaI - speedOut[3*idxPred + 1]*ts)/ts;

  }// for Pred

}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
__global__
void xyz_model_cuda(float* C, float* command, float* XYz, float* XYref, float* param, float thetaP, int nbFeat)
{

  // Thread index
  // + blockIdx.x*blockDim.x
  int idx = threadIdx.x;

  if (idx < param[0]*param[1])
  {

    // Command index
    int idxCom = floor((float)idx/(float)nbFeat);

    // Visual features index
    int idxVF = fmodf((float)idx,(float)nbFeat);

    // Retrieve parameters
    float ts = param[2];
    float dx = param[3];
    float f = param[4];

    float A, t0, t1, c1, c2;
    float Xp, Yp, zp;
    float cx = 0.095837; ///////// Not the correct values
    float cy = 0.022; ///////// See robotClass.cpp

    // Current comand vector
    float v = command[idxCom*3];
    float w = command[idxCom*3 + 1];
    float wp = command[idxCom*3 + 2];

    // Current visual features
    float Xi = XYz[3*idxVF];
    float Yi = XYz[3*idxVF + 1];
    float zi = XYz[3*idxVF + 2];

    // Reference visual features
    float Xref = XYref[2*idxVF];
    float Yref = XYref[2*idxVF + 1];



    // Case #1
    if ((abs(w+wp) > 0.000001) & (abs(w) > 0.000001))
    {
      // cout << "CASE 1" << endl;
      A = (w+wp)*ts;
      t0 = thetaP;
      t1 = thetaP + wp*ts;

      c1 = Yi*zi/f - dx*sin(t0) - v/w*cos(t0) + cy;
      c2 = zi + dx*cos(t0) - v/w*sin(t0) + cx;

      zp = c1*sin(A) + c2*cos(A) - dx*cos(t1) + v/w*sin(t1) - cx;
      Xp = zi*Xi/zp;
      Yp = f/zp*(c1*cos(A) - c2*sin(A) + dx*sin(t1) + v/w*cos(t1) - cy);
    }// Case #1

    // Case #2
    else if ( (abs(w)<0.000001) && (abs(wp) > 0.000001))
    {
      // cout << "CASE 2" << endl;
      A = wp*ts;
      t0 = thetaP;
      t1 = thetaP + wp*ts;

      c1 = Yi*zi/f - v/(2*wp)*cos(t0) + cy;
      c2 = zi - v/(2*wp)*sin(t0) + cx;

      zp = c1*sin(A) + c2*cos(A) - v*ts*cos(t1) + v/(2*wp)*sin(t1) - cx;
      Xp = zi*Xi/zp;
      Yp = f/zp*(c1*cos(A) - c2*sin(A) + v*ts*sin(t1) + v/(2*wp)*cos(t1) -cy);
    }// Case #2

    // Case #3
    else if ((abs(w + wp) < 0.000001) && (abs(w) > 0.000001))
    {
      // cout << "CASE 3" << endl;
      t0 = thetaP;
      t1 = thetaP + wp*ts;

      zp = -v/wp*(sin(t1) - sin(t0)) + w/wp*dx*(cos(t1) - cos(t0)) + zi;
      Xp = zi*Xi/zp;
      Yp = (-f*v*(cos(t1) - cos(t0)) - f*w*dx*(sin(t1) - sin(t0)) + wp*zi*Yi) / (wp*zp);
    }// Case #3

    // Case #4
    else if ((abs(w)<0.000001) && (abs(wp)<0.000001))
    {
      t0 = thetaP;
      t1 = thetaP;

      zp = -v*cos(t0)*ts + zi;
      Xp = zi*Xi/zp;
      Yp = (-f*v*sin(t0)*ts + Yi*zi) / (zp);

    }// Case #4

    // Update the cost function
    C[idx] = (Xp - Xref)*(Xp - Xref) + (Yp - Yref)*(Yp - Yref);

  }// if thread
}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
__global__
void xyz_gradient_cuda2(float* C, float* command, float* XYz, float* XYref, float* param, float thetaP)
{

  /////
  // Retrieve parameters
  int nbPrediction = (int) param[0];
  int nbFeat = (int) param[1];
  float ts = param[2];
  float dx = param[3];
  float f = param[4];

  int tIdx = threadIdx.x + blockIdx.x*blockDim.x;

  if(tIdx < (3*nbPrediction*nbPrediction*nbFeat))
  {

    int idxGrad = floor((float)tIdx/(float)(nbFeat*nbPrediction));
    int rest = fmodf((float)tIdx, (float)(nbFeat*nbPrediction));
    int idxPred = floor((float)rest / (float)nbFeat);
    int idxVF = fmodf((float)rest, (float)nbFeat);

    // Define constants
    float A, t0, t1, c1, c2;
    float Xp, Yp, zp;
    float cx = 0.095837; ///////// Not the correct values
    float cy = 0.022; ///////// See robotClass.cpp

    // Current comand vector
    float v = command[idxGrad*3*nbPrediction + idxPred*3];
    float w = command[idxGrad*3*nbPrediction + idxPred*3 + 1];
    float wp = command[idxGrad*3*nbPrediction + idxPred*3 + 2];

    // Current visual features
    float Xi = XYz[3*idxVF];
    float Yi = XYz[3*idxVF + 1];
    float zi = XYz[3*idxVF + 2];

    // Reference visual features
    float Xref = XYref[2*idxVF];
    float Yref = XYref[2*idxVF + 1];
// if(tIdx == 0){
// printf("%f %f %f %f %f %f %f %f \n", ts, dx, f, Xi, Yi, zi, Xref, Yref);
// }
    // Case #1
    if ((abs(w+wp) > 0.000001) & (abs(w) > 0.000001))
    {
      A = (w+wp)*ts;
      t0 = thetaP;
      t1 = thetaP + wp*ts;

      c1 = Yi*zi/f - dx*sin(t0) - v/w*cos(t0) + cy;
      c2 = zi + dx*cos(t0) - v/w*sin(t0) + cx;

      zp = c1*sin(A) + c2*cos(A) - dx*cos(t1) + v/w*sin(t1) - cx;
      Xp = zi*Xi/zp;
      Yp = f/zp*(c1*cos(A) - c2*sin(A) + dx*sin(t1) + v/w*cos(t1)) -cy;
    }// Case #1

    // Case #2
    else if ( (abs(w)<0.000001) && (abs(wp) > 0.000001))
    {
      A = wp*ts;
      t0 = thetaP;
      t1 = thetaP + wp*ts;

      c1 = Yi*zi/f - v/(2*wp)*cos(t0) + cy;
      c2 = zi - v/(2*wp)*sin(t0) + cx;

      zp = c1*sin(A) + c2*cos(A) - v*ts*cos(t1) + v/(2*wp)*sin(t1) - cx;
      Xp = zi*Xi/zp;
      Yp = f/zp*(c1*cos(A) - c2*sin(A) + v*ts*sin(t1) + v/(2*wp)*cos(t1) -cy);
    }// Case #2

    // Case #3
    else if ((abs(w + wp) < 0.000001) && (abs(w) > 0.000001))
    {
      t0 = thetaP;
      t1 = thetaP + wp*ts;

      zp = -v/wp*(sin(t1) - sin(t0)) + w/wp*dx*(cos(t1) - cos(t0)) + zi;
      Xp = zi*Xi/zp;
      Yp = (-f*v*(cos(t1) - cos(t0)) - f*w*dx*(sin(t1) - sin(t0)) + wp*zi*Yi) / (wp*zp);
    }// Case #3

    // Case #4
    else if ((abs(w)<0.000001) && (abs(wp)<0.000001))
    {
      t0 = thetaP;
      t1 = thetaP;

      zp = -v*cos(t0)*ts + zi;
      Xp = zi*Xi/zp;
      Yp = (-f*v*sin(t0)*ts + Yi*zi) / (zp);
    }// Case #4

// printf("G %f %f %f %f\n", Xp, Yp, zp, (Xp - Xref)*(Xp - Xref) + (Yp - Yref)*(Yp - Yref));

    C[idxGrad*nbFeat*nbPrediction + rest] = (Xp - Xref)*(Xp - Xref) + (Yp - Yref)*(Yp - Yref);

  }//if thread
}


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

__global__
void computeDeltaTheta(float* speed, float* deltaTTP, int offset, int nbPrediction, float ts, float delta, float thetaI, float thetaPI)
{

  int tIdx = threadIdx.x + blockIdx.x*blockDim.x;

  if (tIdx < (nbPrediction*(nbPrediction+1)))
  {

    int rowIdx = tIdx / nbPrediction;
    int colIdx = fmodf(tIdx, nbPrediction);

    if(rowIdx == colIdx+1)
    {
      // Fill the first element
      deltaTTP[0] = thetaI;
      deltaTTP[offset] = thetaPI;

      // Fill the first colum of the current row (initial value)
      deltaTTP[rowIdx*(nbPrediction + 1)] = thetaI;
      deltaTTP[rowIdx*(nbPrediction + 1) + offset] = thetaPI;

      // Fill the diagonal with noise (rowIdx + 1 is here to deal with the first line without noise)
      deltaTTP[tIdx + rowIdx + 1] = (speed[3*colIdx + 1] + delta)*ts;
      deltaTTP[tIdx + rowIdx + 1 + offset] = (speed[3*colIdx + 2] + delta)*ts;

    }
    else
    {
      // Compute delta theta
      deltaTTP[tIdx + rowIdx + 1] = speed[3*colIdx + 1]*ts;
      deltaTTP[tIdx + rowIdx + 1 + offset] = speed[3*colIdx + 2]*ts;
    }
  }
}

///////////////////////////////////////////
///////////////////////////////////////////

__global__
void sumDelta(float* delta, int nbPrediction)
{
  // Compute the offset to acces next row
  int offset = threadIdx.x*(nbPrediction + 1);

  // Sum each element of the row vector
  for(int idx = 0; idx < nbPrediction; idx++)
  {
    delta[offset + idx + 1] += delta[offset + idx];
  }

}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
__global__
void sumC(float* C, float* Cmat, int nbPrediction, int nbFeatures)
{
  if(threadIdx.x < 3*nbPrediction)
  {

    // Compute the offset to access next row of the gradient
    int offset = threadIdx.x*nbPrediction*nbFeatures;

    // Sum each element of the row
    float sum = 0.0;
    for(int idx = 0; idx < nbPrediction*nbFeatures; idx++)
    {
      sum += Cmat[offset + idx];
    }

    C[threadIdx.x] = sum;
  }
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
__global__
void sum_finalconstraint(float* C, float* Cmat, int nbPrediction, int nbFeatures)
{
  if(threadIdx.x < 3*nbPrediction)
  {

    // Compute the offset to access next row of the gradient
    int offset = threadIdx.x*nbPrediction*nbFeatures;

    // Sum each element of the row
    float sum = 0.0;
    for(int idx = 0; idx < nbFeatures; idx++)
    {
      sum += Cmat[offset + idx*nbPrediction + nbPrediction - 1];
    }
    C[threadIdx.x] = sum;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
__global__
void computeDeltaXY(float* deltaXY, float* speed, float* deltaTTP, int offset, int nbPrediction, float ts, float dx, float delta)
{

  // Thread index
  int tIdx = threadIdx.x + blockIdx.x*blockDim.x;

  if(tIdx < 3*nbPrediction*nbPrediction)
  {

    //
    int rowOutIdx = tIdx / nbPrediction;
    int rowInIdx = rowOutIdx / 3 + 1;
    int colIdx = fmodf(tIdx, nbPrediction);
    int code = fmodf(rowOutIdx,3);

    // Declare parameters
    float v, w, theta, c1, c2;

    // Select the relevant parameters
    if(code == 0)
    {
      if(rowInIdx == colIdx + 1)
      {
        v = speed[3*colIdx] + delta;
      }
      else
      {
        v = speed[3*colIdx];
      }

      w = speed[3*colIdx + 1];

      theta = deltaTTP[colIdx];

    }
    else if(code == 1)
    {
      v = speed[3*colIdx];

      if(rowInIdx == colIdx + 1)
      {
        w = speed[3*colIdx + 1] + delta;;
      }
      else
      {
        w = speed[3*colIdx + 1];
      }

      theta = deltaTTP[rowInIdx*(nbPrediction + 1) + colIdx];

    }
    else
    {
      v = speed[3*colIdx];
      w = speed[3*colIdx + 1];
      theta =  deltaTTP[colIdx];
    }

    // Compute DeltaX and DeltaY
    c1 = w*ts/2;
    c2 = (2*theta+w*ts)/2;

    if(abs(w) > 0.000001)
    {
      deltaXY[tIdx] = 2*v/w*sin(c1)*cos(c2) - 2*dx*sin(c1)*sin(c2);
      deltaXY[tIdx + offset] = 2*v/w*sin(c1)*sin(c2) + 2*dx*sin(c1)*cos(c2);
    }
    else
    {
      // cout << "straight" << endl;
      deltaXY[tIdx] = v*cos(theta)*ts - w*dx*sin(theta)*ts;
      deltaXY[tIdx + offset] = v*sin(theta)*ts + w*dx*cos(theta)*ts;
    }
  }
}


///////////////////////////////////////////////////////////
__global__
void eqSpeed_cuda(float* speedOut, float* deltaXY, int offsetXY, float* deltaTTP, int offsetTTP, int nbPrediction, float dx, float ts)
{

  // Thread index -> match the output format
  int tIdx = threadIdx.x + blockIdx.x*blockDim.x;

  if(tIdx < 3*nbPrediction*nbPrediction)
  {

    int row = tIdx / nbPrediction;
    int col = fmodf(tIdx, nbPrediction);
    int rowTTP = row / 3 + 1;
    int mode  = fmodf(row, 3);

    float DX = deltaXY[tIdx];
    float DY = deltaXY[tIdx + offsetXY];
    float thetaI = deltaTTP[0];
    float thetaPI = deltaTTP[offsetTTP];
    float theta, thetaP;


    if(mode == 0) // No noise
    {
      theta = deltaTTP[col + 1];
      thetaP = deltaTTP[col + 1 + offsetTTP];
    }
    else if(mode == 1) // Noise on theta
    {
      theta = deltaTTP[rowTTP*(nbPrediction + 1) + col + 1];
      thetaP = deltaTTP[col + 1 + offsetTTP];
    }
    else // Noise on theta platine
    {
      theta = deltaTTP[col + 1];
      thetaP = deltaTTP[rowTTP*(nbPrediction + 1) + col + 1 + offsetTTP];
    }

    // w eq
    float w = 2/ts*atan2(-DX*sin(thetaI) + DY*cos(thetaI), 2*dx + DX*cos(thetaI) + DY*sin(thetaI));

    speedOut[3*tIdx + 1] = w;

    // v eq
    float v, rslt;
    if(abs(w) < 0.000001)
    {
      v = sqrt(pow(DX,2) + pow(DY,2)) / ts;
    }
    else
    {
      rslt = pow(w,2)*((pow(DX,2) + pow(DY,2)) / (4*pow(sin(w*ts/2),2)) - dx*dx);
      if(rslt < 0.000001)
      {
        v = 0.0;
      }
      else
      {
        v = sqrt(rslt);
      }


    }
    speedOut[3*tIdx + 0] = v;

    // wp eq
    float wp = (thetaP - thetaPI + theta - thetaI - w*ts)/ts;
    speedOut[3*tIdx + 2] = wp;
if(isnan(v)){printf(" EQ: %f | %f | %f %f %f %f %f %f \n", v, rslt, w, wp, DX, DX, theta, thetaP);}

  }

}



////////////////////////////////////////////
//
__global__
void zeros_cuda(float* vec, int size)
{
  if(threadIdx.x < size)
  {
    vec[threadIdx.x] = 0.0;
  }
}
