/*****************************************************************//**
 *  @file					: dro_mip_model.h
 *  @namespace		: DRO
 *  @brief				: a MIP model for moment-based distributional robust optimization solving by solver
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/27
 *********************************************************************/

#ifndef _DRO_MIP_MODEL_
#define _DRO_MIP_MODEL_

#include "include.h"
#include "DroData.h"
#include "determine_model.h"
 
struct StageIIVarSet
{
	vector<GRBVar> X;
	vector<GRBVar> Y;
	vector<GRBVar> Z;
	vector<GRBVar> G;
};



/// <summary>
/// Class : DRO-MIP-Model
/// Model: the reformulation of distributional robust model
/// Input  : (Data , Ambigguity Set Type)
/// </summary>
class DROMIPModel
{
public:
	DROMIPModel(DroData& data, AmbiguitySetType type)
		: Data(data),
		SetType(type)
	{
		filepath = data.getFilepath();
		solveProblem();
	}

public:
	// frame : solve problem
	inline void solveProblem() {
		addDecisionVars();
		setObjective();
		addConstraints();
		solveModel();

#ifdef DRO_PRINT_MIP_DRO_SOLUTION_
		printSolution();
#endif // DRO_PRINT_MIP_DRO_SOLUTION_
	}

private:
	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);

	// input
	// Data / Parameter
	DroData Data;
	int M = 99999999;
	// uncertain set
	AmbiguitySetType SetType = Type1;

	// output
	string filepath;

	// Decision Variables
	// Stage I :
	// vessel decision vars
	// V[h][r] : the vessel h at shipping path
	vector<vector<GRBVar>> V;
	// initial empty containers
	// L[p]
	vector<GRBVar> L;

	// dual variables for type1 ambiguity set
	// yita
	GRBVar yita;

	// alpha1[od]
	// alpha2[od]
	vector<GRBVar> alpha1;
	vector<GRBVar> alpha2;

	// beta1[od]
	// beta2[od]
	vector<GRBVar> beta1;
	vector<GRBVar> beta2;

	// dual variables fot type2 ambiguity set
	// gamma
	GRBVar gamma;	 
	// pi[i]
	vector<GRBVar> pi;	
	// Phi[i][i'] for type-II
	vector<vector<GRBVar>> Phi;

	// Decision variables
	inline void addDecisionVars()
	{
		// Type-I Ambiguity Set : moment-based first-moment and second- moment
		setDecisionsStageI();
		if (SetType == Type1)
		{
			setDualDecisionsTypeI();
		}
		// Type-I Ambiguity Set : moment-based first-moment and convariance matrix
		else if (SetType == Type2)
		{
			setDualDecisionsTypeII();
		}

		// here we create the stage II decision primal variables in the function that add constraints, owing to vars' large scale
	}
	/**
	 * add decisions in Stage I, Stage II
	 *  Stage II vars are created according to samples set (support set), in which each sample
	 *  has a group of corresponding Vars (X, Y, Z, G) (denote it as struct StageIIVarSet)
	*/
	void setDecisionsStageI();
	StageIIVarSet& setDecisionsStageII(StageIIVarSet& Vars, vector<double>& demand, int sample_index);

	 // set dual decisions for moment-based ambiguity set
	 // type-I
	void setDualDecisionsTypeI();
	// type-II
	void setDualDecisionsTypeII();



	// Objective
	// set objective
	void setObjective();


	// Constraints
	// addConstraints
	inline void addConstraints()
	{
		setConstraintStageI();
		setConstraintPartII();
	}


	/*
	the following functions is about add constraints
	*/
	inline void setConstraintStageI() {
		setVesselConstraint1();
		setVesselConstraint2();
		//setVesselConstraint3();
	}
	void setConstraintPartII();
	inline void setConstraintStageII(StageIIVarSet& varSet, vector<double>& demand, int sample_index) {
		setDemandCosntraint(varSet, demand, sample_index);
		setVesselCapacityConstraint(varSet, sample_index);
		setEmptyFlowBalanceConstraint(varSet, sample_index);
	}
	void setDualConstraintsTypeI(StageIIVarSet& VarSet, vector<double>& demand, int sample_index);
	void setDualConstraintsTypeII(StageIIVarSet& VarSet, vector<double>& request, int sample_index);

	// constraints in stage I : 
	// add vessel rotation constriant
	void setVesselConstraint1();
	// add vessel number constraint
	void setVesselConstraint2();
	//// add weekly frequency constraint
	//void setVesselConstraint3();

	// constraints in stage II :
	// add demand constraint, i.e., at time t: demand(od) = X(od) + Z(od) + G(od)
	void setDemandCosntraint(StageIIVarSet& Vars, vector<double>& demand, int sample_index);
	// add vessel capacity constraint
	void setVesselCapacityConstraint(StageIIVarSet& Vars, int sample_index);
	// add empty container flow balance, i.e., L_m^t >= 0 at any time t in any port p
	// in which L_m^t = sum(Flow in) - sum(Flow out)
	void setEmptyFlowBalanceConstraint(StageIIVarSet& Vars, int sample_index);


	// solve DRO MIP model
	void solveModel();

	void printSolution();
};
#endif // !_DRO_MIP_MODEL_
