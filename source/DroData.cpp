#include "DroData.h"

void DroData::GenerateTimePointSet()
{
	const int planning_horizon = this->getPlanningHorizon();
	const vector<int> arrival_time = this->getRoute().getArrivalTimeSequence();
	const vector<Port> port_sequence = this->getRoute().getPortSequence();
	const int round_trip = this->getRoundTripTime();
	vector<int> TimePointSet;

	numRotation = vector<int>(num_ship_path);
	for (int v = 0; v < num_ship_path; v++)
	{
		// set the number of ship rotation for each shipping path
		numRotation[v] = floor((planning_horizon - arrival_time[0] - v * 7) / roundTripTime);

		int arrival_time_call = 0;
		int N = 0;
		while (arrival_time_call < planning_horizon)
		{
			for (size_t call_index = 0; call_index < arrival_time.size(); ++call_index)
			{
				// a complete rotation path call
				arrival_time_call = arrival_time[call_index] + 7 * v + N * roundTripTime;
				if (arrival_time[0] + v * 7 + N * roundTripTime + roundTripTime <= planning_horizon)
				{
					if (call_index == 0 && N > 0)
						continue;

					TimePointSet.push_back(arrival_time_call);
				}
				else
				{
					break;
				}
			}
			N++;
		}
	}

	this->SetTimePointSet(TimePointSet);
}

void DroData::GeneratePortCallSet()
{
	const int planning_horizon = this->getPlanningHorizon();
	const vector<int> arrival_time = this->getRoute().getArrivalTimeSequence();
	const vector<Port> port_sequence = this->getRoute().getPortSequence();
	const int round_trip = this->getRoundTripTime();

	for (int v = 0; v < num_ship_path; v++)
	{
		int arrival_time_call = 0;

		// N is the ship path ID
		int N = 0;
		while (arrival_time_call < planning_horizon)
		{
			for (size_t call_index = 0; call_index < arrival_time.size(); ++call_index)
			{
				// a complete rotation path call
				arrival_time_call = arrival_time[call_index] + 7 * v + N * roundTripTime;
				if (arrival_time[0] + v * 7 + N * roundTripTime + roundTripTime <= planning_horizon)
				{
					if (call_index == 0 && N > 0)
						continue;

					this->CallNodeSet.push_back(CallNode(port_sequence[call_index], arrival_time_call, v, N));
				}
				else
				{
					break;
				}
			}
			N++;
		}
	}

	for (size_t c = 0; c < CallNodeSet.size() - 1; c++)
	{
		if (CallNodeSet[c].getShipPathIndex() == CallNodeSet[c + 1].getShipPathIndex())
		{
			SegmentSet.push_back(make_pair(CallNodeSet[c], CallNodeSet[c + 1]));
		}
	}
}


// generate od pairs if there exits request between o and d
// according to the set , demand = 0 if o and d are in the same region
void DroData::GenerateODpairs()
{
	vector<Port> P = this->getPortSet();
	vector<vector<OD>> Wod = vector<vector<OD>>(P.size(), vector<OD>(P.size(), OD()));

	for (size_t o = 0; o < P.size(); o++)
	{
		for (size_t d = 0; d < P.size(); d++)
		{
			if (o == d)
				continue;
			Wod[o][d] = (OD(int(o), int(d), P[o], P[d]));
		}
	}

	this->setWod(Wod);
}

void DroData::GenerateTravelTimeAdj()
{
	Route r = this->getRoute();
	vector<Port> P = this->getRoute().getPortSequence();
	vector<vector<int>> time_adj = vector<vector<int>>(P.size(), vector<int>(P.size(), int()));

	for (size_t o = 0; o < P.size(); o++)
	{
		for (size_t d = o; d < P.size(); d++)
		{
			if (o == d)
			{
				time_adj[o][d] = 0;
			}
			else
			{
				time_adj[o][d] = r.getArrivalTimeSequence()[d] - r.getArrivalTimeSequence()[o];
			}
		}
	}
	for (size_t o = 0; o < P.size(); o++)
	{
		for (size_t d = 0; d < o; d++)
		{
			time_adj[o][d] = r.getRoundTripTime() - time_adj[d][o];
		}
	}

	this->SetTransitTime(time_adj);
}

void DroData::GenerateDemand()
{
#ifdef DRO_USE_EIGEN_
	GenerateRequestSet();
	RequestDataMap();
#elif
	GenerateRandomDemandSet();
	GenerateDetermineDemand();
	CalculateMeanVariance();
	CalculateCovariance();
	CalculateLUBound();
#endif // DRO_USE_EIGEN_
}

void DroData::GenerateRequestSet()
{
	int request_id = 0;

	// create request and random demand for each request
	int num_samples = this->GetSampleSize();
	int num_port = this->getPortSet().size();

	static default_random_engine e(0);

	for (size_t o = 0; o < this->CallNodeSet.size(); o++)
	{
		for (size_t d = o + 1; (d < o + num_port && d < CallNodeSet.size()); d++)
		{
			if (CallNodeSet[o].getShipPathIndex() == CallNodeSet[d].getShipPathIndex())
			{
				int o_region = CallNodeSet[o].getPort().GetRegion() - 1;
				int d_region = CallNodeSet[d].getPort().GetRegion() - 1;
				int left = demand_bound[o_region][d_region].first;
				int right = demand_bound[o_region][d_region].second;

				if (left == right)
					continue;

				std::uniform_int_distribution<int> dist(left, right);

				vector<int> demandSampleSet(num_samples);
				int sumdemand = 0;

				for (int i = 0; i < num_samples; i++) {
					demandSampleSet[i] = dist(e);
					sumdemand += demandSampleSet[i];
				}

				// get the release node
				bool flag = false;
				int release = d;
				int min_release_time = planningHorizon;
				for (size_t rel = d; rel < CallNodeSet.size(); rel++)
				{
					if (CallNodeSet[d].getPort() == CallNodeSet[rel].getPort()
						&& CallNodeSet[rel].getArrivalTime() >= CallNodeSet[d].getArrivalTime() + CallNodeSet[d].getPort().getTurnoverTime()) 
					{
						flag = true;
						if (min_release_time > CallNodeSet[rel].getArrivalTime())
						{
							min_release_time = CallNodeSet[rel].getArrivalTime();
							release = rel;
						}
					}
     			}

				request_id++;
				Request req = Request(request_id, CallNodeSet[o], CallNodeSet[d], CallNodeSet[release], (sumdemand / num_samples), flag, demandSampleSet);

				this->RequestSet.push_back(req);
			}
		}
	}
}

void DroData::RequestDataMap()
{
	int num_request = RequestSet.size();
	int num_samples = this->GetSampleSize();
	demandSet = Eigen::MatrixXd(num_samples, num_request);

	for (size_t r = 0; r < RequestSet.size(); r++) {
		for (int i = 0; i < num_samples; i++) {
			demandSet(i, r) = RequestSet[r].getUncertainDemandSet()[i];
		}
	}

	demandFirstMoment = Eigen::MatrixXd(1, num_request);
	demandFirstMoment = demandSet.colwise().mean();
	demandSecondMoment = Eigen::MatrixXd(1, num_request);
	demandSecondMoment = demandSet.colwise().squaredNorm() / num_samples;
	demandCovarianceMatrix = Eigen::MatrixXd(num_request, num_request);
	Eigen::MatrixXd m = demandSet.rowwise() - demandSet.colwise().mean();
	demandCovarianceMatrix = (m).transpose() * (m) / (num_samples);

#ifdef DRO_PRINT_DATA_
	cout << "=================== demand set begin===================" << endl;
	cout << demandSet << endl;
	cout << "=================== demand set end===================" << endl;

	cout << "=================== First-moment begin===================" << endl;
	cout << demandFirstMoment << endl;
	cout << "=================== First-moment end===================" << endl;

	cout << "=================== Second-moment begin===================" << endl;
	cout << demandSecondMoment << endl;
	cout << "=================== Second-moment end===================" << endl;

	cout << "=================== Covariance matrix begin===================" << endl;
	cout << demandCovarianceMatrix << endl;
	cout << "=================== Covariance matrix end===================" << endl;
#endif // DRO_PRINT_DATA_

	int numSamples = demandSet.rows();
	int numRequest = demandSet.cols();

	demandSampleSet.resize(numSamples, std::vector<double>(numRequest));
	meanDemand.resize(numRequest);
	varianceDemand.resize(numRequest);
	covarianceDemand.resize(numRequest, std::vector<double>(numRequest));

	for (size_t m = 0; m < numSamples; m++)
	{
		for (size_t n = 0; n < numRequest; n++)
		{
			demandSampleSet[m][n] = demandSet(m, n);
		}
	}

	for (size_t n = 0; n < numRequest; n++)
	{
		for (size_t m = 0; m < numSamples; m++)
		{
			demandSampleSet[m][n] = demandSet(m, n);
			if (demandSampleSet[m][n] < demandSet(m, n))
				cout << "E" << endl;
		}
		meanDemand[n] = demandFirstMoment(n);
		varianceDemand[n] = demandSecondMoment(n);

		for (size_t nn = 0; nn < numRequest; nn++)
		{
			covarianceDemand[n][nn] = demandCovarianceMatrix(n, nn);
		}
	}
}

map<string, Port> DroData::inputPortData(string filename)
{
	ifstream fin(filename);
	if (!fin.is_open())
	{
		std::cout << "can't open ports file" << endl;
	}

	string first_line;
	getline(fin, first_line);
	std::cout << first_line << endl;

	map<string, Port> M;
	vector<Port> portSet;

	//double loadcost, discharge, transshopment;
	int portID;
	string port;
	Port tempPort;
	int portCount = 0;
	int region = -1;
	/*bool whether_Trans;
	int Group;*/
	while (!fin.eof())
	{
		fin >> portID >> port >> region;
		tempPort.setPortID(portID);
		tempPort.setPort(port);
		tempPort.setPortIndex(portCount++);
		tempPort.SetRegion(region);
		//tempPort.loadingCost = loadcost;
		//tempPort.dischargeCost = discharge;
		//tempPort.transshipmentCost = transshopment;
		M.emplace(make_pair(port, tempPort));
		portSet.push_back(tempPort);
	}
	this->setPortSet(portSet);
	this->setPortSet(M);

	//output ports information
	for (auto& x : M)
	{
		std::cout << x.second.getPortID() << '\t' << x.second.getPort() << endl;
	}

	return M;
}

vector<vector<Vessel>> DroData::inputVesselData(string filename)
{
	ifstream fin(filename);
	if (!fin.is_open())
	{
		std::cout << "can't open ports file" << endl;
	}

	string first_line;
	getline(fin, first_line);
	std::cout << first_line << endl;

	vector<vector<Vessel>> vesselsSet = vector<vector<Vessel>>(this->GetRouteSet().size(), vector<Vessel>());
	vector<Vessel> vesselSet;

	int ID;
	int num;
	int capacity;
	int routeID;
	double operation_cost;

	int tempID = 0;

	while (!fin.eof())
	{
		fin >> ID >> capacity >> operation_cost >> routeID >> num;

		if (tempID != 0 && tempID != routeID)
		{
			vesselsSet.push_back(vesselSet);
			vesselSet.clear();
			tempID = routeID;
		}
		if (tempID == 0)
		{
			tempID = routeID;
		}
		vesselSet.push_back(Vessel(ID, routeID, capacity, num, operation_cost * 1e6));
	}
	//vesselSet.pop_back();
	vesselsSet.push_back(vesselSet);
	this->SetVesselSet(vesselsSet[0]);
	this->SetVesselsSet(vesselsSet);

	//output ports information
	for (auto& x : vesselsSet)
	{
		for (auto& y : x)
		{
			y.ShowVessel();
		}
	}

	return this->getVesselsSet();
}

vector<Route> DroData::inputRouteData(string filename)
{
	ifstream fin(filename);
	if (!fin.is_open())
	{
		std::cout << "can't open ship routes file" << endl;
	}

	map<string, Port> M = this->getPortMap();
	vector<Route> RouteSet;

	string first_line;
	getline(fin, first_line);
	std::cout << first_line << endl;

	int routeID, call, arrivalTime;
	string port;
	int tempID = 0;
	vector<Port> portSequence;
	vector<int> arrivalTimeSequence;
	while (!fin.eof())
	{
		fin >> routeID >> call >> port >> arrivalTime;
		cout << routeID << "\t" << call << "\t" << port << "\t" << arrivalTime << "\n";

		if (tempID != 0)
		{
			if (routeID != tempID)
			{
				// before start read the next route : storage the current route, clear the temp port sequence set and temp arrival time sequence
				RouteSet.push_back(Route(routeID, arrivalTime - arrivalTimeSequence[0], portSequence, arrivalTimeSequence));
				portSequence.clear();
				arrivalTimeSequence.clear();
				tempID = routeID;
			}
			portSequence.push_back(M[port]);
			arrivalTimeSequence.push_back(arrivalTime);
		}
		else
		{
			// tempID = 0 : means start read first call of the first route
			tempID = routeID;
			portSequence.push_back(M[port]);
			arrivalTimeSequence.push_back(arrivalTime);
		}
	}
	RouteSet.push_back(Route(routeID, arrivalTime - arrivalTimeSequence[0], portSequence, arrivalTimeSequence));

	// only need information of one route
	// here we choose the first route (we can change next)
	this->setRoute(RouteSet[0]);
	this->SetTau(RouteSet[0].getRoundTripTime());

	//output ship routes
	std::cout << "output ship routes" << endl;
	for (auto& x : RouteSet)
	{
		x.showRoute();
	}

	return RouteSet;
}

void DroData::SetAuxiliaryParameter()
{
	map<string, Port> M = this->getPortMap();
	vector<Port> P = this->getPortSet();
	vector<vector<OD>> Wod = this->getWod();

	vector<vector<bool>> 	theta = vector<vector<bool>>(M.size(),
		vector<bool>(P.size(), bool()));
	int m_count = 0;
	for (auto& m : M)
	{
		for (int p = 0; p < P.size(); p++)
		{
			if (m.first == P[p].getPort())
			{
				theta[m_count][p] = 1;
			}
			else
			{
				theta[m_count][p] = 0;
			}
		}
		m_count++;
	}

	this->SetThetaMp(theta);



	// delta p_od
	vector<vector <vector< bool >> > delta_p_od = vector<vector <vector< bool >> >(P.size(),
		vector <vector< bool >>(Wod.size(), vector<bool>(Wod[0].size())));
	for (size_t p = 0; p < P.size(); p++)
	{
		for (size_t o = 0; o < Wod.size(); o++)
		{
			for (size_t d = 0; d < Wod[o].size(); d++)
			{
				int o_call_index = Wod[o][d].GetOriginCallIndex();
				int d_call_index = Wod[o][d].GetDestinationCallIndex();

				if (P[p].getPortIndex() >= o_call_index && P[p].getPortIndex() < d_call_index)
				{
					delta_p_od[p][o][d] = 1;
				}
				else
				{
					delta_p_od[p][o][d] = 0;
				}
			}

		}
	}

	this->SetDeltaPOd(delta_p_od);
}

void DroData::SetDefaultParameter()
{
	map<string, Port> M = this->getPortMap();
	vector<Port> P = this->getPortSet();
	vector<vector<OD>> Wod = this->getWod();
	vector<vector<bool>> theta = this->GetThetaMp();

	// set turnover Time
	for (auto& x : M)
	{
		x.second.setTurnoverTime(standardTurnoverTime);
	}

	// set initial empty containers for each port in M
	for (auto& x : M)
	{
		int initial_empty_container_at_p = 0;
		for (size_t o = 0; o < Wod.size(); o++)
		{
			for (size_t d = 0; d < Wod[o].size(); d++)
			{
				if (o == d)
					continue;
				int o_index = Wod[o][d].GetOriginCallIndex();
				initial_empty_container_at_p += theta[x.second.getPortIndex()][o_index];
			}
		}
		x.second.setInitialEmptyContainer(initial_empty_container_at_p);
	}

	for (size_t p = 0; p < P.size(); p++)
	{
		P[p].setTurnoverTime(M[P[p].getPort()].getTurnoverTime());
		P[p].setInitialEmptyContainer(M[P[p].getPort()].getInitialEmptyContainer());
	}

	this->setPortSet(M);
	this->setPortSet(P);
}

void DroData::CalculateLUBound()
{
	double epsilon1 = this->GetEpsilon1();
	pair<double, double> epsilon2 = this->GetEpsilon2();

	size_t num_request = this->RequestSet.size();

	vector<int> Lodt1 = vector<int>(num_request);
	vector<int> Lodt2 = vector<int>(num_request);
	vector<int> Uodt1 = vector<int>(num_request);
	vector<int> Uodt2 = vector<int>(num_request);

	vector<double> means = this->GetMeanDemand();
	vector<double> variance = this->GetVarianceDemand();

	for (size_t od = 0; od < num_request; od++)
	{
		Lodt1[od] = (int)(means[od] * (1 - epsilon1));
		Uodt1[od] = (int)(means[od] * (1 + epsilon1));
		Lodt2[od] = (int)(variance[od] * epsilon2.first);
		Uodt2[od] = (int)(variance[od] * epsilon2.second);
	}

	this->setLodt1(Lodt1);
	this->setUodt1(Uodt1);
	this->setLodt2(Lodt2);
	this->setUodt2(Uodt2);
}

void DroData::GenerateUnitCost()
{
	GenerateLoadingCost();
	GenerateRentalCost();
	GeneratePenaltyCost();
}

void DroData::GenerateLoadingCost()
{
	vector<Port>& P = this->getPortSet();
	map<string, Port>& M = this->getPortMap();
	for (size_t p = 0; p < P.size(); p++)
	{
		P[p].setUnitLoadingCost(25);
		M[P[p].getPort()].setUnitLoadingCost(25);
		P[p].setUnitDischargeCost(25);
		M[P[p].getPort()].setUnitDischargeCost(25);
	}
	this->setPortSet(P);
	this->setPortSet(M);
}

void DroData::GenerateRentalCost()
{
	vector<vector<OD>> Wod = this->getWod();
	vector<vector<int>> time_adj = this->GetTransitTime();
	for (size_t o = 0; o < Wod.size(); o++)
	{
		for (size_t d = 0; d < Wod[o].size(); d++)
		{
			if (o == d)
				continue;
			int o_index = Wod[o][d].GetOriginCallIndex();
			int d_index = Wod[o][d].GetDestinationCallIndex();
			Wod[o][d].SetUnitRentalCost(unitRentalPrice * time_adj[o_index][d_index]);
		}
	}

	this->setWod(Wod);
}

void DroData::GeneratePenaltyCost()
{
	vector<vector<OD>> Wod = this->getWod();
	vector<vector<pair<int, int>>> freight_bound = this->GetFreightBound();

	for (size_t o = 0; o < Wod.size(); o++)
	{
		for (size_t d = 0; d < Wod[o].size(); d++)
		{
			if (o == d)
				continue;
			int o_region = Wod[o][d].getOriginPort().GetRegion() - 1;
			int d_region = Wod[o][d].getDestinationPort().GetRegion() - 1;
			int left = freight_bound[o_region][d_region].first;
			int right = freight_bound[o_region][d_region].second;
			Wod[o][d].SetUnitPenaltyCost(left + rand() % (right - left + 1));
		}
	}

	this->setWod(Wod);
}

void DroData::UpdatePortSetting()
{
	vector<Port> portSeq = this->route.getPortSequence();
	for (size_t p = 0; p < portSeq.size(); p++)
	{
		portSeq[p] = M[portSeq[p].getPort()];
	}
	route.setPortSequence(portSeq);


	for (size_t c = 0; c < CallNodeSet.size(); c++)
	{
		CallNodeSet[c].setPort(M[CallNodeSet[c].get_port()]);
	}
}

void DroData::calculateCapacity(vector<vector<int>>& VesselDecison)
{
	vector<Vessel>& VesselSet = this->getVesselSet();

	capacitySet = vector<int>(SegmentSet.size());
	for (size_t seg = 0; seg < SegmentSet.size(); seg++)
	{
		int p_index = SegmentSet[seg].first.getShipPathIndex();
		for (size_t h = 0; h < VesselSet.size(); h++)
		{
			if (VesselDecison[h][p_index] == 1)
			{
				capacitySet[seg] = VesselSet[h].GetCapacity();
				break;
			}
		}
	}
}
