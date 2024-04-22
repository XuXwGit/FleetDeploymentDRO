/*****************************************************************//**
 *  @file					: test.h
 *  @namespace		: DRO
 *  @brief				: test algos
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/28
 *********************************************************************/

#ifndef _TEST_H_
#define _TEST_H_

#include "include.h"
#include "DroData.h"
#include "determine_model.h"
#include "dro_mip_model.h"
#include "subproblem.h"
#include "dualSubproblem.h"
#include "BendersDecomposition.h"

using namespace std;
void testDRO() {
//
    // T = 60
    // S = 10
    string path = "Data/input/data2/";
    DroData Data(path, 180, 2);


    cout << "============== Determine Model ==============" << endl;
    clock_t start, end;
    start = clock();
    DetermineModel dp(Data);
    end = clock();
    cout << "Total Solve Time : " << (double)(end - start) / CLOCKS_PER_SEC << "s"<<endl;
    cout << endl<<"===================================================" << endl<<endl;

    cout << "============== SubProblem Model ==============" << endl;
    SubProblem sp(Data);
    sp.setVesselDecision(dp.getVesselDecision());
    sp.setInitialContainer(dp.getInitialContainer());
    sp.solveProblem();
    cout << endl << "===================================================" << endl<<endl;

    cout << "============== Dual Problem Model ==============" << endl;
    DualSP dsp(Data);
    dsp.setVesselDecision(dp.getVesselDecision());
    dsp.setSegmentCapacity(dp.getCapacity());
    dsp.setInitialContainer(dp.getInitialContainer());
    dsp.solveProblem();
    cout << endl << "===================================================" << endl<<endl;

    cout << "============== DRO Model with Type I ==============" << endl;
    start = clock();
    DROMIPModel dro1(Data, Type1);
    end = clock();
    cout << "Total Solve Time of " << " MIP-DRO-I: " << (double)(end - start) / CLOCKS_PER_SEC << "s";
    cout << endl << "===================================================" << endl<<endl;


    cout << "============== Benders Decomposition Type-I ==============" << endl;
    start = clock();
    BendersDecomposition BD(Data);
    end = clock();
    cout << "Total Solve Time of " << " BD-DRO-I: " << (double)(end - start) / CLOCKS_PER_SEC << "s";
    cout << endl << "===================================================" << endl << endl;


    cout << "============== DRO Model with Type II =============" << endl;
    start = clock();
    //DROMIPModel dro2(Data, Type2);
    end = clock();
    cout << "Total Solve Time of " << " MIP-DRO-II: " << (double)(end - start) / CLOCKS_PER_SEC << "s";
    cout << endl << "===================================================" << endl<<endl;
}


/*
Core：Matrix和Array类，基础的线性代数运算和数组操作；
Geometry：旋转，平移，缩放，2维和3维的各种变换；
LU：求逆，行列式，LU分解；
Cholesky：LLT和LDLT Cholesky分解；
Householder：Householder变换；
SVD：SVD分解；
QR：QR分解。
Eigenvalues：特征值，特征向量分解。
Sparse：稀疏矩阵的存储和运算。
Dense：包含了Core、Geometry、LU、Cholesky、SVD、QR、Eigenvalues等模块。
Eigen：包含了Dense和Sparse模块。
*/
void testEigen() {

    //产生随机数引擎，采用time 0作为种子
    static default_random_engine e(time(0));
    //产生正态分布对象
    static normal_distribution<double> n(0, 1);
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(2, 5).unaryExpr([&](double dummy) {
        double value = n(e);  // 生成高斯分布的随机数
        value = std::max(value, -3.0);  // 限制随机数的下界为 -3
        value = std::min(value, 3.0);   // 限制随机数的上界为 3
        return (value + 3.0) / 6.0;  // 变换和缩放到范围 [0, 1]
        });
    cout << "Gaussian random matrix:\n" << m << endl;
    cout << "First-moment: " << endl << m.colwise().mean() << endl;
    cout << "Second-moment: " << endl << m.colwise().squaredNorm() / 2 << endl;
    m.rowwise() -= m.colwise().mean();
    cout << "Covariance matrix: " << endl << m.transpose() * m << endl;
    cout << "Square: " << endl << m.colwise().squaredNorm() / 2 << endl;

}

#endif // !_TEST_H_