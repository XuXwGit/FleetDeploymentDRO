#include "dro_mip_model.h"

/// <summary>
/// set desicion variables in stage I
/// </summary>
void DROMIPModel::setDecisionsStageI()
{
	vector<Vessel> vesselSet = Data.getVesselSet();
	int Np = Data.getNumShipPath();

	V = vector<vector<GRBVar>>(vesselSet.size(), vector<GRBVar>(Np));
	for (size_t h = 0; h < V.size(); h++)
	{
		for (size_t p = 0; p < Np; p++)
		{
			string VarName = "V[" + to_string(h) + "][" + to_string(p) + "]";
			V[h][p] = model.addVar(0, 1, vesselSet[h].GetOperationCost(), GRB_BINARY, VarName);
		}
	}

	size_t numPort = Data.getPortMap().size();
	L = vector<GRBVar>(numPort);
	for (size_t p = 0; p < numPort; p++)
	{
		string VarName = "L[" + to_string(p) + "]";
		L[p] = model.addVar(1000, 1000, 250, GRB_CONTINUOUS, VarName);
	}
}


/// <summary>
/// set decision vars in stage II
/// </summary>
/// <param name="Vars"></param>
/// <param name="demand"></param>
/// <param name="sample_index"></param>
/// <returns></returns>
StageIIVarSet& DROMIPModel::setDecisionsStageII(StageIIVarSet& Vars, vector<double>& demand, int sample_index)
{
	vector<Vessel> vesselSet = Data.getVesselSet();
	vector<Request> RequestSet = Data.GetRequestSet();
	size_t numPort = Data.getPortMap().size();
	size_t numOD = demand.size();
	const int T = Data.GetTimePointSet().size();

	vector<GRBVar>& X = Vars.X;
	vector<GRBVar>& Y = Vars.Y;
	vector<GRBVar>& Z = Vars.Z;
	vector<GRBVar>& G = Vars.G;

	X = vector<GRBVar>(numOD);
	Y = vector<GRBVar>(numOD);
	Z = vector<GRBVar>(numOD);
	G = vector<GRBVar>(numOD);

	for (size_t od = 0; od < numOD; od++)
	{
		Port o = RequestSet[od].getOrigin().getPort();
		Port d = RequestSet[od].getDestination().getPort();

		string VarName = "X[" + to_string(sample_index) + "][" + to_string(od) + "]";
		X[od] = model.addVar(0, demand[od], 0, GRB_CONTINUOUS, VarName);
		VarName = "Y[" + to_string(sample_index) + "][" + to_string(od) + "]";
		Y[od] = model.addVar(0, demand[od], 0, GRB_CONTINUOUS, VarName);
		VarName = "Z[" + to_string(sample_index) + "][" + to_string(od) + "]";
		Z[od] = model.addVar(0, demand[od], 0, GRB_CONTINUOUS, VarName);
		VarName = "G[" + to_string(sample_index) + "][" + to_string(od) + "]";
		G[od] = model.addVar(0, demand[od], 0, GRB_CONTINUOUS, VarName);
	}

	return Vars;
}



/// <summary>
/// set dual decisions
/// </summary>
void DROMIPModel::setDualDecisionsTypeI()
{
	vector<Request> RequestSet = Data.GetRequestSet();
	vector<int> Lodt1 = Data.getLodt1();
	vector<int> Lodt2 = Data.getLodt2();
	vector<int> Uodt1 = Data.getUodt1();
	vector<int> Uodt2 = Data.getUodt2();

	yita = model.addVar(-GRB_INFINITY, GRB_INFINITY, 1, GRB_CONTINUOUS, "yita");

	alpha1 = vector<GRBVar>(RequestSet.size());
	alpha2 = vector<GRBVar>(RequestSet.size());
	beta1 = vector<GRBVar>(RequestSet.size());
	beta2 = vector<GRBVar>(RequestSet.size());

	for (size_t od = 0; od < RequestSet.size(); od++)
	{
		alpha1[od] = model.addVar(0, M, Uodt1[od], GRB_CONTINUOUS, "alpha1[" + to_string(od) + "]");
		alpha2[od] = model.addVar(0, M, Uodt2[od], GRB_CONTINUOUS, "alpha2[" + to_string(od) + "]");
		beta1[od] = model.addVar(-M, 0, Lodt1[od], GRB_CONTINUOUS, "beta1[" + to_string(od) + "]");
		beta2[od] = model.addVar(-M, 0, Lodt2[od], GRB_CONTINUOUS, "beta2[" + to_string(od) + "]");
	}
}


/// <summary>
/// set dual decisions
/// </summary>
void DROMIPModel::setDualDecisionsTypeII()
{
	vector<double> mean_request = Data.GetMeanRequest();
	vector<vector<double>> covariance_request = Data.GetCovarianceRequest();
	size_t num_request = mean_request.size();

	gamma = model.addVar(-M, M, 1, GRB_CONTINUOUS, "gamma");

	pi = vector<GRBVar>(num_request, GRBVar());
	Phi = vector<vector<GRBVar>>(num_request, vector<GRBVar>(num_request, GRBVar()));

	for (size_t i = 0; i < num_request; i++)
	{
		pi[i] = model.addVar(-M, M, mean_request[i], GRB_CONTINUOUS, "pi[" + to_string(i) + "]");

		for (size_t j = 0; j < num_request; ++j)
		{
			Phi[i][j] = model.addVar(-M, M, covariance_request[i][j], GRB_CONTINUOUS, "Phi[" + to_string(i) + "][" + to_string(j) + "]");
		}
	}
}




/// <summary>
/// 
/// </summary>
/// <param name="VarSet"></param>
/// <param name="demand"></param>
/// <param name="sample_index"></param>
void DROMIPModel::setDualConstraintsTypeI(StageIIVarSet& VarSet, vector<double>& demand, int sample_index)
{
	vector<vector<double>> supportSet = Data.getDemandSet();
	vector<Request> requestSet = Data.GetRequestSet();
	vector<vector<OD>> Wod = Data.getWod();

	vector<GRBVar>& X = VarSet.X;
	vector<GRBVar>& Y = VarSet.Y;
	vector<GRBVar>& Z = VarSet.Z;
	vector<GRBVar>& G = VarSet.G;

	GRBLinExpr left = yita;

	for (size_t od = 0; od < requestSet.size(); od++)
	{
		left += demand[od] * alpha1[od]
			+ (demand[od] * demand[od]) * alpha2[od]
			+ demand[od] * beta1[od]
			+ (demand[od] * demand[od]) * beta2[od];
	}

	GRBLinExpr right = 0;
	for (size_t od = 0; od < requestSet.size(); od++)
	{
		Port oo = requestSet[od].getOrigin().getPort();
		Port dd = requestSet[od].getDestination().getPort();

		double unitRental = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitRentalCost();
		double unitPenalty = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitPenaltyCost();

		right += (oo.getUnitLoadingCost() + dd.getUnitDischargeCost()) * Y[od] + unitRental * Z[od] + unitPenalty * G[od];
	}

	string name = "c-dual-[" + to_string(sample_index) + "]";
	model.addConstr(left - right >= 0, name);
}


/// <summary>
/// 
/// </summary>
/// <param name="VarSet"></param>
/// <param name="request"></param>
/// <param name="sample_index"></param>
void DROMIPModel::setDualConstraintsTypeII(StageIIVarSet& VarSet, vector<double>& demand, int sample_index)
{
	vector<double> mean_request = Data.GetMeanDemand();
	vector<Request> requestSet = Data.GetRequestSet();
	vector<vector<OD>> Wod = Data.getWod();

	vector<GRBVar>& X = VarSet.X;
	vector<GRBVar>& Y = VarSet.Y;
	vector<GRBVar>& Z = VarSet.Z;
	vector<GRBVar>& G = VarSet.G;

	GRBLinExpr left = gamma;

	// item1
	for (size_t i = 0; i < demand.size(); i++)
	{
		left += pi[i] * demand[i];
	}
	// item2
	for (size_t i = 0; i < demand.size(); i++)
	{
		for (size_t j = 0; j < demand.size(); j++)
		{
			left += (demand[i] - mean_request[i] ) * ( demand[j] - mean_request[j]) * Phi[i][j];
		}
	}

	GRBLinExpr right = 0;
	for (size_t od = 0; od < requestSet.size(); od++)
	{
		Port oo = requestSet[od].getOrigin().getPort();
		Port dd = requestSet[od].getDestination().getPort();

		double unitRental = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitRentalCost();
		double unitPenalty = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitPenaltyCost();

		right += (oo.getUnitLoadingCost() + dd.getUnitDischargeCost()) * Y[od] + unitRental * Z[od] + unitPenalty * G[od];
	}

	string name = "c-dual-[" + to_string(sample_index) + "]";
	model.addConstr(left - right >= 0, name);
}


/// <summary>
/// 
/// </summary>
void DROMIPModel::setVesselConstraint1()
{
	vector<Vessel> vesselSet = Data.getVesselSet();

	for (size_t path = 0; path < Data.getNumShipPath(); path++)
	{
		GRBLinExpr left = 0;
		for (size_t h = 0; h < vesselSet.size(); h++)
		{
			left += V[h][path];
		}
		string name = "c-I-1[" + to_string(path) + "]";
		model.addConstr(left == 1, name);
	}
}


/// <summary>
/// 
/// </summary>
void DROMIPModel::setVesselConstraint2()
{
	vector<Vessel> vesselSet = Data.getVesselSet();

	for (size_t h = 0; h < vesselSet.size(); h++)
	{
		GRBLinExpr left = 0;
		for (size_t p = 0; p < Data.getNumShipPath(); p++)
		{
			left += V[h][p];
		}
		string name = "c-I-2[" + to_string(h) + "]";
		model.addConstr(left <= vesselSet[h].getMaxNum(), name);
	}
}


/// <summary>
/// 
/// </summary>
/// <param name="Vars"></param>
/// <param name="demand"></param>
/// <param name="sample_index"></param>
void DROMIPModel::setDemandCosntraint(StageIIVarSet& Vars, vector<double>& demand, int sample_index)
{
	vector<GRBVar>& X = Vars.X;
	vector<GRBVar>& Z = Vars.Z;
	vector<GRBVar>& G = Vars.G;

	string name;
	for (size_t od = 0; od < demand.size(); od++)
	{
		name = "c-II-2[" + to_string(sample_index) + "][" + to_string(od) + "]";
		model.addConstr(X[od] + Z[od] + G[od] == demand[od], name);
	}
}



/// <summary>
/// 
/// </summary>
/// <param name="Vars"></param>
/// <param name="sample_index"></param>
void DROMIPModel::setVesselCapacityConstraint(StageIIVarSet& Vars, int sample_index)
{

	vector<GRBVar>& X = Vars.X;
	vector<GRBVar>& Y = Vars.Y;
	vector<GRBVar>& Z = Vars.Z;

	typedef pair<CallNode, CallNode> SegMent;
	vector<SegMent> segmentSet = Data.getSegmentSet();
	vector<Request> requestSet = Data.GetRequestSet();
	vector<vector<int>> transitTime = Data.getTransitTime();
	vector<int> portCallSet = Data.getPortCallSet();
	vector<vector<OD>> Wod = Data.getWod();
	vector<Vessel> vesselSet = Data.getVesselSet();
	map<string, Port> M = Data.getPortMap();

	string name;
	for (size_t seg = 0; seg < segmentSet.size(); seg++)
	{
		int p = segmentSet[seg].first.getShipPathIndex();

		double originTime = segmentSet[seg].first.getArrivalTime();
		double destinationTime = segmentSet[seg].second.getArrivalTime();

		GRBLinExpr left = 0;

		for (size_t od = 0; od < requestSet.size(); od++)
		{
			if (requestSet[od].getOrigin().getShipPathIndex() != p)
				continue;
			if (requestSet[od].getOrigin().getArrivalTime() <= originTime
				&& requestSet[od].getDestination().getArrivalTime() >= destinationTime)
			{
				left += X[od] + Y[od] + Z[od];
			}
		}

		GRBLinExpr right = 0;
		for (size_t h = 0; h < vesselSet.size(); h++)
		{
			right += vesselSet[h].GetCapacity() * V[h][p];
		}

		string name = "c-II-1[" + to_string(sample_index) + "][" + to_string(seg) + "]";
		model.addConstr(left <= right, name);
	}
}



/// <summary>
/// 
/// </summary>
/// <param name="Vars"></param>
/// <param name="sample_index"></param>
void DROMIPModel::setEmptyFlowBalanceConstraint(StageIIVarSet& Vars, int sample_index)
{
	vector<GRBVar>& Y = Vars.Y;
	vector<GRBVar>& X = Vars.X;

	// M - port_set
	map<string, Port> M = Data.getPortMap();
	vector<vector<OD>> Wod = Data.getWod();
	vector<vector<int>> transitTime = Data.getTransitTime();
	vector<vector<bool>> callWithPort = Data.GetThetaMp();

	vector<CallNode> callNodeSet = Data.getCallNodeSet();
	vector<Request> requestSet = Data.GetRequestSet();

	for (size_t n = 0; n < callNodeSet.size(); n++)
	{
		CallNode p = callNodeSet[n];

		GRBLinExpr left = L[p.getPort().getPortIndex()];

		for (size_t od = 0; od < requestSet.size(); od++)
		{
			CallNode origin = requestSet[od].getOrigin();
			CallNode destination = requestSet[od].getDestination();
			CallNode release = requestSet[od].getReleaseNode();

			bool flag = requestSet[od].getWhetherEmptyContainer();

			// laden container -> empty container (release after arrival and turnover)
			if (flag && release.getArrivalTime() <= p.getArrivalTime() && release.getPort() == p.getPort())
			{
				left += X[od];
			}

			// empty container -> empty container (reposition from other port)
			if (destination.getArrivalTime() <= p.getArrivalTime() && destination.getPort() == p.getPort())
			{
				left += Y[od];
			}

			// empty container -> laden container (transport to other port)
			if (origin.getArrivalTime() <= p.getArrivalTime() && origin.getPort() == p.getPort())
			{
				left -= X[od];
			}

			// empty container -> empty container (reposition to other port)
			if (origin.getArrivalTime() <= p.getArrivalTime() && origin.getPort() == p.getPort())
			{
				left -= Y[od];
			}
		}

		string name = "c-II-3[" + to_string(sample_index) + "][" + to_string(n) + "]";
		model.addConstr(left >= 0, name);
	}
}


/// <summary>
/// 
/// </summary>
void DROMIPModel::setObjective()
{
	GRBLinExpr obj = 0;

	vector<Vessel> vesselSet = Data.getVesselSet();
	vector<Port> PortSet = Data.getPortSet();
	vector<int> num_rotation = Data.getNumRotation();

	// Stage I : item1 + item2
	// add item1
	for (size_t p = 0; p < Data.getNumShipPath(); p++)
	{
		for (size_t h = 0; h < vesselSet.size(); h++)
		{
			obj += vesselSet[h].GetOperationCost() * num_rotation[p] * V[h][p];
		}
	}

	//// add item2
	//for (size_t lp = 0; lp < PortSet.size(); lp++)
	//{
	//	obj += Data.getContainerPrice() * L[lp];
	//}

	// Stage II :
	// add item3
	if (SetType == Type1)
	{
		vector<int> Lodt1 = Data.getLodt1();
		vector<int> Lodt2 = Data.getLodt2();
		vector<int> Uodt1 = Data.getUodt1();
		vector<int> Uodt2 = Data.getUodt2();

		obj += yita;
		for (size_t od = 0; od < Data.GetRequestSet().size(); od++)
		{
				obj += alpha1[od] * Uodt1[od]
					+ alpha2[od] * Uodt2[od]
					+ beta1[od] * Lodt1[od]
					+ beta2[od] * Lodt2[od];
		}
	}
	else if (SetType == Type2)
	{
		vector<double> mean_request = Data.GetMeanRequest();
		vector<vector<double>> covariance_request = Data.GetCovarianceRequest();

		obj += gamma;

		for (size_t i = 0; i < mean_request.size(); ++i)
		{
			obj += mean_request[i] * pi[i];

			for (size_t j = 0; j < covariance_request.size(); ++j)
			{
				obj += covariance_request[i][j] * Phi[i][j];
			}
		}
	}

	model.setObjective(obj, GRB_MINIMIZE);

	// the front codes can also be substituted by the following one code:
	// (because we have set the obj coefficients while adding decision varibales)
	//	model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
}


/// <summary>
/// 
/// </summary>
void DROMIPModel::setConstraintPartII()
{
	const vector<vector<double>> supportSet = Data.getDemandSet();
	const int sample_size = Data.GetSampleSize();

	for (int w = 0; w < sample_size; w++)
	{
		vector<double> demand = supportSet[w];

		StageIIVarSet VarSet;
		setDecisionsStageII(VarSet, demand, w+1);

		setConstraintStageII(VarSet, demand, w+1);

		// set dual constraints
		if (SetType == Type1)
		{
			setDualConstraintsTypeI(VarSet, demand,w+1);
		}
		else if (SetType == Type2)
		{
			setDualConstraintsTypeII(VarSet, demand, w+1);
		}
	}
}

/// <summary>
/// solve the MIP model with solver
/// </summary>
void DROMIPModel::solveModel()
{
	model.set(GRB_IntParam_Presolve, 0);

#ifndef DRO_MIP_OUTPUT_LOG_
	model.set(GRB_IntParam_OutputFlag, 0);
#endif // DRO_MIP_OUTPUT_LOG_

	model.optimize();
	if (SetType == Type1) {
		model.write(filepath + "dro_mip_model-I.lp");
	}
	else
	{
		model.write(filepath + "dro_mip_model-II.lp");
	}

	if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
	{
		cout << setprecision(2);
		cout << "Model Status : " << "Optimal" << endl;
		cout << "Optimal Obj = " << fixed << model.get(GRB_DoubleAttr_ObjVal) << endl;
		cout << "Model Solve Time = " << model.get(GRB_DoubleAttr_Runtime) << endl;
	}
	else
	{
		// print violated constraint
		const int numConstrs = model.get(GRB_IntAttr_NumConstrs);
		auto constrs = model.getConstrs();
		for (int i = 0; i < numConstrs; ++i) {
			GRBConstr constr = constrs[i];
			double slack = constr.get(GRB_DoubleAttr_Slack);
			if (slack < 0) {
				double violation = -slack;
				cout << "Constraint " << constr.get(GRB_StringAttr_ConstrName) << " is violated by " << violation << endl;
			}
		}
	}
}


/// <summary>
/// print solution of solver
/// </summary>
void DROMIPModel::printSolution()
{
	cout << "Vessel Ship Schedule : " << endl;
	for (size_t v = 0; v < Data.getNumShipPath(); v++)
	{
		cout << "Ship Path " << v << ": ";
		for (size_t h = 0; h < Data.getVesselSet().size(); h++)
		{
			int value_V_hv = static_cast<int>(model.getVarByName(V[h][v].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X) + 0.5);
			if (value_V_hv != 0)
			{
				cout << h << " ( " << Data.getVesselSet()[h].GetCapacity() << " TEUs )" << endl;
			}
		}
	}

	cout << "Initial Empty Containers £º" << endl;
	cout << "Port" << '\t' << "NumEmptyContainer" << endl;
	for (size_t p = 0; p < Data.getPortSet().size(); p++)
	{
		int L_value = model.getVarByName(L[p].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X);
		cout << Data.getPortSet()[p].getPort() << '\t' << L_value << endl;
	}
}