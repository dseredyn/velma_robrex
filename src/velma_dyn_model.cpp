// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#include "velma_dyn_model.h"

#include <stdlib.h>
#include <math.h>

#include "params.inc"

DynModelVelma::DynModelVelma() :
    Mp_(16, 16),
    invMp_(16, 16)
{
    M_.resize(15,15);
    invM_.resize(15,15);
    tmpTau_.resize(15);
}

DynModelVelma::~DynModelVelma() {
}
 
void DynModelVelma::inertia(Eigen::MatrixXd &x, const Eigen::VectorXd &qq){

    Eigen::VectorXd q(16);
    q(0) = qq(0);
    q(1) = -3.141592653589793/2.0;  // torso_1_joint is fixed
    for (int q_idx = 2; q_idx < 16; q_idx++) {
        q(q_idx) = qq(q_idx-1);
    }

    x.setZero();
#include "inertia.inc"
}

void DynModelVelma::gaussjordan(const Eigen::MatrixXd &inMatrix, Eigen::MatrixXd &outMatrix, int dim){
 
	int iRow, iCol, diagIndex;
	double diagFactor, tmpFactor;
    Eigen::MatrixXd inMatrixCopy(inMatrix);
 
	/* make deep copy of input matrix */
	for(iRow = 0; iRow < dim; iRow++ ){
		for (iCol = 0; iCol < dim; iCol++){
			inMatrixCopy(iCol,iRow) = inMatrix(iCol,iRow);
		}
	}
	/* Initialize output matrix as identity matrix. */
	for (iRow = 0; iRow < dim; iRow++ ){
		for (iCol = 0; iCol < dim; iCol++ ){
			if (iCol == iRow){
				outMatrix(iCol,iRow) = 1;
			}
			else{
				outMatrix(iCol,iRow) = 0;
			}
		}
	}
 
	for (diagIndex = 0; diagIndex < dim; diagIndex++ )
	{
		/* determine diagonal factor */
		diagFactor = inMatrixCopy(diagIndex,diagIndex);
 
		/* divide column entries by diagonal factor */
		for (iCol = 0; iCol < dim; iCol++){
			inMatrixCopy(iCol,diagIndex) /= diagFactor;
			outMatrix(iCol,diagIndex) /= diagFactor;
		}
 
		/* perform line-by-line elimination */
		for (iRow = 0; iRow < dim; iRow++){
			if (iRow != diagIndex){
				tmpFactor = inMatrixCopy(diagIndex,iRow);
 
				for(iCol = 0; iCol < dim; iCol++){
				inMatrixCopy(iCol,iRow)  -= inMatrixCopy(iCol,diagIndex)*tmpFactor;
				outMatrix(iCol,iRow) -= outMatrix(iCol,diagIndex)*tmpFactor;
				}
			}
		} /* line-by-line elimination */
 
	}
}

void DynModelVelma::computeM(const Eigen::VectorXd &q) {
/*	inertia(Mp_, q);
    M_(0,0) = Mp_(0,0);
    for (int i=1; i<15; i++) {
        M_(i,0) = Mp_(i+1,0);
        M_(0,i) = Mp_(0,i+1);
    }
    for (int i=1; i<15; ++i) {
        for (int j=1; j<15; ++j) {
            M_(i,j) = Mp_(i+1,j+1);
        }
    }
*/
/*
    inertia(invMp_, q);
    invM_(0,0) = invMp_(0,0);
    for (int i=1; i<15; i++) {
        invM_(i,0) = invMp_(i+1,0);
        invM_(0,i) = invMp_(0,i+1);
    }
    for (int i=1; i<15; ++i) {
        for (int j=1; j<15; ++j) {
            invM_(i,j) = invMp_(i+1,j+1);
        }
    }
	gaussjordan(invM_, M_, 15);
//*/
/*
	gaussjordan(Mp_, invMp_, 16);
    invM_(0,0) = invMp_(0,0);
    for (int i=1; i<15; i++) {
        invM_(i,0) = invMp_(i+1,0);
        invM_(0,i) = invMp_(0,i+1);
    }
    for (int i=1; i<15; ++i) {
        for (int j=1; j<15; ++j) {
            invM_(i,j) = invMp_(i+1,j+1);
        }
    }
*/
    M_ = Eigen::MatrixXd::Identity(15, 15);
	gaussjordan(M_, invM_, 15);
}

void DynModelVelma::accel(Eigen::VectorXd &QDD, const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &t){

	/* declare variables */
	int iCol;

	/* call the computational routines */
//	coriolis(C_, q, dq);
//	gravload(gravload, q);
//	friction(friction, dq);
 
	/* fill temporary vector */
//    tmpTau_ = C_ * dq;
	for (iCol = 0; iCol < 15; iCol++){
//		tmpTau_[iCol] = t[iCol] -  tmpTau_[iCol] - gravload[iCol][0] + friction[iCol][0];
		tmpTau_[iCol] = t[iCol] - dq[iCol] * 1.0;
	}
	/* compute acceleration */
    QDD = invM_ * tmpTau_;
}

