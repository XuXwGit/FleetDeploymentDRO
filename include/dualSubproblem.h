/*****************************************************************//**
 *  @file					: dualSubproblem.h
 *  @namespace		: DRO
 *  @brief				: dual problem of the primal subproblem
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/27
 *********************************************************************/

#ifndef DRO_DUAL_SUBPROBLEM_H_
#define DRO_DUAL_SUBPROBLEM_H_

#include "include.h"
#include "DroData.h"

class DualSP
{
public:
	DualSP() {};
	// constructor
	DualSP(DroData& _data)
		: Data(_data) {
		// initialize
		initializeObjCoeff();
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
	vector<int> capacity;
	// the demand (scenerio)
	vector<double> demand;

	// the second decision (U, V, W)
	// dual decision variables
	// U[od]
	// V[a]
	// W[n]
	vector<GRBVar> U;
	vector<GRBVar> V;
	vector<GRBVar> W;

	// constraints set
	vector<GRBConstr> ConstrX;
	vector<GRBConstr> ConstrY;
	vector<GRBConstr> ConstrZ;
	vector<GRBConstr> ConstrG;

	// objective function
	GRBLinExpr obj;

	// solution
	vector<double> Uvalue;
	vector<double> Vvalue;
	vector<double> Wvalue;

	double objValue;

public:
	// set Data
	inline void setDemand(const vector<double>& demand) {
		this->demand = demand;
	}
	inline void setVesselDecision(const vector<vector<int>>& vesselDecision) {
		this->VesselDecision = vesselDecision;
	}
	inline void setSegmentCapacity(const vector<int>& Capacity) {
		this->capacity = Capacity;
	}
	inline void setInitialContainer(const vector<int>& initialContainer) {
		this->InitialContainer = initialContainer;
	}

	// reset objective
	void resetObjective(vector<double>& demand, vector<int>& capacity, vector<int>& initialContainer);

	// get Data
	// get solution
	// get dual variables value
	inline vector<double>& getUvalue() {
		return Uvalue;
	}
	inline vector<double>& getVvalue() {
		return Vvalue;
	}
	inline vector<double>& getWvalue() {
		return Wvalue;
	}
	
	// get objective value
	inline double getObjective() {
		return objValue;
	}

private:
	// initialize
	inline void initializeObjCoeff() {
		setDemand(Data.GetMeanDemand());
		capacity = vector<int>(Data.getSegmentSet().size(), 0);
		InitialContainer = vector<int>(Data.getPortSet().size(), 0);
	}

	// create model
	// add decision variables
	void addDecisionVars();
	// add constraints
	inline void addConstraints() {
		setConstraintX();
		setConstraintY();
		setConstraintZ();
		setConstraintG();
	}

	// add objective
	void setObjective();

	void setConstraintX();
	void setConstraintY();
	void setConstraintZ();
	void setConstraintG();

	// set solution value
	void setUvalue();
	void setVvalue();
	void setWvalue();
	inline void setObjValue() {
		objValue = model.get(GRB_DoubleAttr_ObjVal);
	}

public:
	// build model
	inline void bulidModel() {
		addDecisionVars();
		addConstraints();
		setObjective();
	}
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
#ifndef DRO_DSP_OUTPUT_LOG_
		model.set(GRB_IntParam_OutputFlag, 0);
#endif // DRO_DSP_OUTPUT_LOG_

		clock_t  start = clock();
		model.optimize();
		clock_t end = clock();

		setUvalue();
		setVvalue();
		setWvalue();
		setObjValue();

#ifdef DRO_PRINT_DUAL_SP_SOLUTION_H_
		// print solution of dual sub problem
		cout << "optimize time = " << (end - start) / CLOCKS_PER_SEC << endl;
		model.write(filepath + "dsp-model.lp");
		if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
		{
			cout << setprecision(2);
			cout << "DSP Optimal Obj = " << fixed << model.get(GRB_DoubleAttr_ObjVal) << endl;
			cout << "Model Solve Time = " << model.get(GRB_DoubleAttr_Runtime) << endl;
		}
#endif // DRO_PRINT_DUAL_SP_SOLUTION_H_

	}
};


#endif // !DRO_DUAL_SUBPROBLEM_H_