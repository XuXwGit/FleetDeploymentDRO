/*****************************************************************//**
 *  @file					: subproblem.h
 *  @namespace		: DRO
 *  @brief				: the primal subproblem of DRO, i.e., the two-stage problem (or the recourse problem)
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/27
 *********************************************************************/

#ifndef DRO_SUBPROBLEM_H_
#define DRO_SUBPROBLEM_H_

#include "include.h"
#include "DroData.h"

// this is a class for subproblem
// class SubProblem
class SubProblem
{
public:
	SubProblem(DroData& _data)
		: Data(_data){
		setDemand(_data.GetMeanDemand());
	}

private:
	// shipping path network
	DroData Data;
	// output file path
	string filepath;

	// model and environment
	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);

	// input
	// the first decision (V, I)
	vector<vector<int>> VesselDecision;
	vector<int> InitialContainer;
	// the demand (scenerio)
	vector<double> demand;

	// the second decision (X, Y, Z, G)
	// transport decision variables
	// X[od]
	// Y[od]
	// Z[od]
	// G[od]
	vector<GRBVar> X;
	vector<GRBVar> Y;
	vector<GRBVar> Z;
	vector<GRBVar> G;

	vector<GRBConstr> DemandConstrSet;
	vector<GRBConstr> CapacityConstrSet;
	vector<GRBConstr> FlowBalanceConstrSet;

public:
	// set Data
	inline void setDemand(const vector<double>& demand) {
		this->demand = demand;
	}
	inline void setVesselDecision(const vector<vector<int>>& vesselDecision) {
		this->VesselDecision = vesselDecision;
	}
	inline void setInitialContainer(const vector<int>& initialContainer) {
		this->InitialContainer = initialContainer;
	}
private:
	// create model
	// add decision variables
	void addDecisionVars();
	// add constraints
	inline void addConstraints() {
		setDemandCosntraint();
		setVesselCapacityConstraint();
		setEmptyFlowBalanceConstraint();
	}
	// add objective
	void setObjective();

	// add demand constraint
	void setDemandCosntraint();
	// add capacity constraint
	void setVesselCapacityConstraint();
	// add flow balance constraint
	void setEmptyFlowBalanceConstraint();

public:
	// solve problem
	inline void solveProblem() {
		addDecisionVars();
		setObjective();
		addConstraints();
		solveModel();
		//printSolution();
	}
	// solve model
	inline void solveModel()
	{
#ifndef DRO_SP_OUTPUT_LOG_
		model.set(GRB_IntParam_OutputFlag, 0);
#endif // DRO_SP_OUTPUT_LOG_

		clock_t  start = clock();
		model.optimize();
		clock_t end = clock();

#ifdef DRO_PRINT_SP_SOLUTION_H_
		cout << "optimize time = " << (end - start) / CLOCKS_PER_SEC << endl;
		model.write(filepath + "sp-model.lp");
		if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
		{
			cout << setprecision(2);
			cout << "Optimal Obj = " << fixed << model.get(GRB_DoubleAttr_ObjVal) << endl;
			cout << "Model Solve Time = " << model.get(GRB_DoubleAttr_Runtime) << endl;
		}
#endif // DRO_PRINT_SP_SOLUTION_H_
	}
	// print solution
	void printSolution();
};
#endif // !_SUBPROBLEM_H_

