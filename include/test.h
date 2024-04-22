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
Core��Matrix��Array�࣬���������Դ�����������������
Geometry����ת��ƽ�ƣ����ţ�2ά��3ά�ĸ��ֱ任��
LU�����棬����ʽ��LU�ֽ⣻
Cholesky��LLT��LDLT Cholesky�ֽ⣻
Householder��Householder�任��
SVD��SVD�ֽ⣻
QR��QR�ֽ⡣
Eigenvalues������ֵ�����������ֽ⡣
Sparse��ϡ�����Ĵ洢�����㡣
Dense��������Core��Geometry��LU��Cholesky��SVD��QR��Eigenvalues��ģ�顣
Eigen��������Dense��Sparseģ�顣
*/
void testEigen() {

    //������������棬����time 0��Ϊ����
    static default_random_engine e(time(0));
    //������̬�ֲ�����
    static normal_distribution<double> n(0, 1);
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(2, 5).unaryExpr([&](double dummy) {
        double value = n(e);  // ���ɸ�˹�ֲ��������
        value = std::max(value, -3.0);  // ������������½�Ϊ -3
        value = std::min(value, 3.0);   // ������������Ͻ�Ϊ 3
        return (value + 3.0) / 6.0;  // �任�����ŵ���Χ [0, 1]
        });
    cout << "Gaussian random matrix:\n" << m << endl;
    cout << "First-moment: " << endl << m.colwise().mean() << endl;
    cout << "Second-moment: " << endl << m.colwise().squaredNorm() / 2 << endl;
    m.rowwise() -= m.colwise().mean();
    cout << "Covariance matrix: " << endl << m.transpose() * m << endl;
    cout << "Square: " << endl << m.colwise().squaredNorm() / 2 << endl;

}

#endif // !_TEST_H_