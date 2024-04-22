#include "determine_model.h"

void DetermineModel::setObjective()
{
	GRBLinExpr obj = 0;

	vector<Vessel> vesselSet = Data.getVesselSet();
	vector<vector<OD>> Wod = Data.getWod();
	vector<Request> requestSet = Data.GetRequestSet();
	vector<Port> PortSet = Data.getPortSet();
	size_t numPort = Data.getPortMap().size();
	
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

	// add item2
	for (size_t lp = 0; lp < PortSet.size(); lp++)
	{
		obj += Data.getContainerPrice() * L[lp];
	}

	// Stage II :
	// add item3
	int od = 0;
	for (size_t od = 0; od < requestSet.size(); od++)
	{
		Port oo = requestSet[od].getOrigin().getPort();
		Port dd = requestSet[od].getDestination().getPort();

		double unitRental = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitRentalCost();
		double unitPenalty = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitPenaltyCost();

		obj += (oo.getUnitLoadingCost() + dd.getUnitDischargeCost()) * Y[od] + unitRental * Z[od] + unitPenalty * G[od];
	}

	model.setObjective(obj, GRB_MINIMIZE);

	// the front codes can also be substituted by the following one code:
	// (because we have set the obj coefficients while adding decision varibales)
	//model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
}

void DetermineModel::setDecisionsStageI()
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
		L[p] = model.addVar(0, M, 100, GRB_CONTINUOUS, VarName);
	}
}

void DetermineModel::setDecisionsStageII()
{
	vector<double> demand = Data.GetMeanDemand();

	vector<vector<OD>> Wod = Data.getWod();
	vector<Request> RequestSet = Data.GetRequestSet();
	size_t numRequest = RequestSet.size();
	size_t numPort = Data.getPortMap().size();

	X = vector<GRBVar>(numRequest, GRBVar());
	Y = vector<GRBVar>(numRequest, GRBVar());
	Z = vector<GRBVar>(numRequest, GRBVar());
	G = vector<GRBVar>(numRequest, GRBVar());

	for (size_t od = 0; od < numRequest; od++)
	{
		Port oo = RequestSet[od].getOrigin().getPort();
		Port dd = RequestSet[od].getDestination().getPort();

		string VarName = "X[" + to_string(od) + "]";
		X[od] = model.addVar(0, demand[od], 0, GRB_CONTINUOUS, VarName);
		VarName = "Y[" + to_string(od) + "]";
		Y[od] = model.addVar(0, demand[od], oo.getUnitLoadingCost() + dd.getUnitDischargeCost(), GRB_CONTINUOUS, VarName);

		double unitRental = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitRentalCost();
		double unitPenalty = Wod[oo.getPortIndex()][dd.getPortIndex()].GetUnitPenaltyCost();

		VarName = "Z[" + to_string(od) + "]";
		Z[od] = model.addVar(0, demand[od], unitRental, GRB_CONTINUOUS, VarName);
		VarName = "G[" + to_string(od) + "]";
		G[od] = model.addVar(0, demand[od], unitPenalty, GRB_CONTINUOUS, VarName);
	}
}

void DetermineModel::setVesselConstraint()
{
	vector<Vessel> vesselSet = Data.getVesselSet();

	for (size_t p = 0; p < Data.getNumShipPath(); p++)
	{
		GRBLinExpr left = 0;
		for (size_t h = 0; h < vesselSet.size(); h++)
		{
			left += V[h][p];
		}
		string name = "c-I-1[" + to_string(p) + "]";
		model.addConstr(left == 1, name);
	}

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

void DetermineModel::setDemandCosntraint()
{
	vector<Request> requestSet = Data.GetRequestSet();
	vector<double> demand = Data.GetMeanDemand();

	string name;
	for (size_t od = 0; od < requestSet.size(); od++)
	{
		name = "c-II-2[" + to_string(od) + "]";
		model.addConstr(X[od] + Z[od] + G[od] == demand[od], name);
	}
}

void DetermineModel::setVesselCapacityConstraint()
{
	typedef pair<CallNode, CallNode> SegMent;
	vector<SegMent> segmentSet = Data.getSegmentSet();
	vector<Request> requestSet = Data.GetRequestSet();
	vector<vector<int>> transitTime = Data.getTransitTime();
	vector<int> portCallSet = Data.getPortCallSet();
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

		string name = "c-II-1[" + to_string(seg) + "]";
		model.addConstr(left <= right, name);
	}
}

void DetermineModel::setEmptyFlowBalanceConstraint()
{
	// M - port_set
	map<string, Port> M = Data.getPortMap();
	vector<vector<OD>> Wod = Data.getWod();
	vector<vector<int>> transitTime = Data.getTransitTime();
	vector<vector<bool>> callWithPort = Data.GetThetaMp();

	vector<CallNode> callNodeSet = Data.getCallNodeSet();
	vector<Request> requestSet = Data.GetRequestSet();

	string constrName = "C-II-3";
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

		model.addConstr(left >= 0, constrName + "[" + to_string(n) + "]");
	}
}

void DetermineModel::setVesselDecision()
{
	vesselDeploymentCost = 0;
	VesselDecision = vector<vector<int>>(Data.getVesselSet().size(), vector<int>(Data.getNumShipPath()));
	for (size_t h = 0; h < Data.getVesselSet().size(); h++)
	{
		for (size_t v = 0; v < Data.getNumShipPath(); v++)
		{
			VesselDecision[h][v] = int(V[h][v].get(GRB_DoubleAttr_X) + 0.5);
			if (VesselDecision[h][v] == 1) {
				vesselDeploymentCost += Data.getVesselSet()[h].GetOperationCost() * Data.getNumRotation()[v];
			}
		}
	}
}


void DetermineModel::setInitialContainer()
{
	initialContainerCost = 0;
	InitialContainer = vector<int>(Data.getPortSet().size());
	for (size_t p = 0; p < Data.getPortSet().size(); p++)
	{
		InitialContainer[p] = int(L[p].get(GRB_DoubleAttr_X));
		initialContainerCost = InitialContainer[p] * Data.getContainerPrice();
	}
}

void DetermineModel::printSolution()
{
	if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
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

		cout << "Request Solution :" << endl;
		cout << "demand" << "\t\t" << "Self-Own" << "\t\t" <<"Reposition" << "\t\t" << "Leased" << "\t\t" << "Unfullfill" << endl;
		for (size_t re = 0; re < Data.GetMeanDemand().size(); re++)
		{
			int X_value = model.getVarByName(X[re].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X);
			int Y_value = model.getVarByName(Y[re].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X);
			int Z_value = model.getVarByName(Z[re].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X);
			int G_value = model.getVarByName(G[re].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X);

			cout << Data.GetRequestSet()[re].getOrigin().getPort().GetRegion() <<"->"
				<< Data.GetRequestSet()[re].getDestination().getPort().GetRegion() << "\t"
				<< Data.GetMeanDemand()[re] <<"\t\t" 
				<< X_value << "\t\t" 
				<< Y_value << "\t\t"
				<< Z_value << "\t\t" 
				<< G_value << endl;
		}

		cout << "Initial Empty Containers £º" << endl;
		cout << "Port" <<"\t\t" << "NumEmptyContainer" << endl;
		for (size_t p = 0; p < Data.getPortSet().size(); p++)
		{
			int L_value = model.getVarByName(L[p].get(GRB_StringAttr_VarName)).get(GRB_DoubleAttr_X);
			cout << Data.getPortSet()[p].getPort() << "\t\t" << L_value << endl;
		}
	}
	else
	{
		cout << "!! Not Optimal !!" << endl;
	}
}
