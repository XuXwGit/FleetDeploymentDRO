/*****************************************************************//**
 *  @file					: DroData.h
 *  @namespace		: DRO
 *  @brief				: Data class : the imput data for model buliding and algorithm, which includes : the shipping path network, the sample support set, e.t.c
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/28
 *********************************************************************/

#ifndef _DRO_DATA_H_
#define _DRO_DATA_H_

#include "include.h"
#include "vessel.h"
#include "OD.h"
#include "route.h"
#include "CallNode.h"
#include "Request.h"

using namespace std;

class DroData
{
public:
	// segment
	typedef pair<CallNode, CallNode> SegMent;
public:
	DroData() {}
	DroData(string path, int T, int S)
		: filepath(path)
	{
		ReadBasicData(path);
		GenerateDroData(T, S);
	}

	inline void ReadBasicData(string path)
	{
		inputPortData(path + "ports.txt");
		inputRouteData(path + "routes.txt");
		inputVesselData(path + "vessels.txt");
	}
	map<string, Port> inputPortData(string filename);
	vector<vector<Vessel>> inputVesselData(string filename);
	vector<Route> inputRouteData(string filename);

	inline void GenerateDroData(int planning_horizon, int sample_size) {
		this->setPlanningHorizon(planning_horizon);
		this->setSampleSize(sample_size);

		int num1 = ceil((planningHorizon + 1 - route.getArrivalTimeSequence()[0] - roundTripTime) / 7);
		int num2 = ceil(roundTripTime / 7);
		int numShip = num1 > num2 ? num2 : num1;
		this->setNumShipPath(numShip);

		// set travel time matrix
		GenerateTravelTimeAdj();
		GenerateODpairs();

		// Set Parameters for each port		
		GenerateUnitCost();
		SetAuxiliaryParameter();
		SetDefaultParameter();

		// set port call node
		GenerateTimePointSet();
		GeneratePortCallSet();
		UpdatePortSetting();

		// set od request
		GenerateDemand();
		CalculateLUBound();
	}

public:

	void GenerateTimePointSet();
	void GeneratePortCallSet();
	void GenerateRequestSet();
	void GenerateODpairs();
	void GenerateTravelTimeAdj();
	void GenerateDemand();

	void RequestDataMap();

	void SetAuxiliaryParameter();
	 
	void SetDefaultParameter();

	void CalculateLUBound();

	void GenerateUnitCost();
	void GenerateLoadingCost();
	void GenerateRentalCost();
	void GeneratePenaltyCost();

	void UpdatePortSetting();

private:
	// T
	int planningHorizon;
	// Tr
	int roundTripTime;
	// N
	int maxVesselNum;
	// S
	int sample_size;
	// Nr
	int num_ship_path;

private:
	string filepath;

	// N
	int sumNumVessels;
	Route route;
	vector<Route> route_set;

	vector<int> numRotation;

	// Call Node Set
	vector<CallNode> CallNodeSet;
	// Request Set
	vector<Request> RequestSet;
	vector<SegMent> SegmentSet;
	// capacity on each segment
	vector<int> capacitySet;

	Eigen::MatrixXd demandSet;
	Eigen::MatrixXd demandFirstMoment;
	Eigen::MatrixXd demandSecondMoment;
	Eigen::MatrixXd demandCovarianceMatrix;

	// Port Set
	// M
	vector<Port> PortSet;
	map<string, Port> M;
	// P
	vector<int> portCallSet;

	// Vessel
	// H
	vector<int> vesselIndexSet;
	vector<Vessel> VesselSet;
	vector<vector<Vessel>> vessels_set;

	//t[p][p']
	vector<vector<int>> transitTime;

	// pi[i][p]: the depature time of ith round-trip from the pth port of call
	vector<vector<int>> pi;

	// W[o][d]	o \in P, d \in P
	vector<vector<int>> ODpairs;
	vector<vector<OD>> Wod;

	// time point set
	vector<int> time_point_set;

private:
	// delta_od_t
	vector<vector<double>> demandSampleSet;
	vector<double> meanDemand;
	vector<double> varianceDemand;
	vector<vector<double>> covarianceDemand;

	// region
	vector<int> region = { 1, 2 };
	vector<vector<pair<int, int>>> demand_bound = { {{0,0}, {100, 200}}, {{50, 150}, {0,0}} };
	vector<vector<pair<int, int>>> freight_bound = { {{0,0}, {3300, 9300}}, {{3500, 8500}, {0,0}} };
	//vector<vector<pair<int, int>>> freight_bound = { {{0,0}, {0, 0}}, {{0, 0}, {0,0}} };

	// unit price
	int containerPrice = 0;
	int unitRentalPrice = 75;

	// standard turnover time
	int standardTurnoverTime = 12;

	// interval of the empirical mean
	// scaling parameter
	double epsilon1 = 0.05;
	pair<double, double> epsilon2 = { 0.75, 1.25 };

	vector<int> L_odt1;
	vector<int> L_odt2;
	vector<int> U_odt1;
	vector<int> U_odt2;

	vector<vector <vector< bool >> > delta_p_od;
	vector<vector<bool>> theta_mp;

public:
	inline int getContainerPrice() {
		return containerPrice;
	}
	inline string getFilepath() {
		return filepath;
	}
	inline vector<vector<double>>& getDemandSet() {
		return demandSampleSet;
	}
	inline vector<CallNode>& getCallNodeSet() {
		return CallNodeSet;
	}
	inline void setSegMent(vector<SegMent>& segSet) {
		SegmentSet = segSet;
	}
	inline vector<SegMent>& getSegmentSet() {
		return SegmentSet;
	}
	inline vector<int>& getNumRotation() {
		return numRotation;
	}
	inline int GetSampleSize() const
	{
		return sample_size;
	}
	inline int getNumShipPath() const {
		return num_ship_path;
	}
	inline void SetSampleSize(const int sample_size)
	{
		this->sample_size = sample_size;
	}
	inline const vector<double>& GetMeanDemand() 
	{
		return meanDemand;
	}
	inline void SetMeanDemand(const vector<double>& mean_demand)
	{
		meanDemand = mean_demand;
	}
	inline int getPlanningHorizon() { return planningHorizon; }
	inline const int getRoundTripTime() { return roundTripTime; }
	inline Route& getRoute() { return route; }
	inline vector <vector< OD >>& getWod() {
		return Wod;
	}
	inline vector<vector<int>>& getTransitTime() {
		return transitTime;
	}
	inline vector<Port>& getPortSet() {
		return PortSet;
	}
	inline map<string, Port>& getPortMap() {
		return M;
	}
	inline vector<int>& getPortCallSet() {
		return portCallSet;
	}
	inline vector<Vessel>& getVesselSet() {
		return VesselSet;
	}
	inline vector<vector<Vessel>>& getVesselsSet() {
		return vessels_set;
	}
	inline vector<vector<double>>& getSupportSet() {
		return demandSampleSet;
	}
	inline vector<vector <vector< bool >> >& getP_OD() {
		return delta_p_od;
	}
	inline vector<vector<bool>>& getCallwithPort()
	{
		return theta_mp;
	}
	inline vector<vector <vector< bool >> >& GetDeltaPOd()
	{
		return delta_p_od;
	}
	inline vector<int>& getLodt1()
	{
		return L_odt1;
	}
	inline vector<int>& getLodt2()
	{
		return L_odt2;
	}
	inline vector<int>& getUodt1()
	{
		return U_odt1;
	}
	inline vector<int>& getUodt2()
	{
		return U_odt2;
	}
	inline int getTotalVesselNum() const
	{
		int totalNum = 0;
		for (size_t h = 0; h < VesselSet.size(); h++)
		{
			totalNum += VesselSet[h].getMaxNum();
		}

		return totalNum;
	}
	inline int GetTau() const
	{
		return roundTripTime;
	}
	inline void SetTau(const int tau)
	{
		this->roundTripTime = tau;
	}
	inline vector<Route> GetRouteSet() const
	{
		return route_set;
	}
	inline void SetRouteSet(const vector<Route> route_set)
	{
		this->route_set = route_set;
	}
	inline void SetVesselSet(const vector<Vessel> vessel_set)
	{
		VesselSet = vessel_set;
	}
	inline void SetVesselsSet(const vector<vector<Vessel>> vessels_set)
	{
		this->vessels_set = vessels_set;
	}
	inline void setWod(vector<vector<OD>> Wod)
	{
		this->Wod = Wod;
	}
	inline void setPlanningHorizon(int timeHorizon) {
		this->planningHorizon = timeHorizon;
	}
	inline void setNumShipPath(int NumShipPath) {
		this->num_ship_path = NumShipPath;
	}
	inline void setSampleSize(int S) {
		this->sample_size = S;
	}
	inline void setPortSet(map<string, Port> M) {
		this->M = M;
	}
	inline void setPortSet(vector<Port> portSet) {
		this->PortSet = portSet;
	}
	inline void setPortCallSet(vector<int> portCallSet) {
		this->portCallSet = portCallSet;
	}
	inline void setRoute(Route route) {
		this->route = route;
	}
	inline void setLodt1(vector<int> L_odt1)
	{
		this->L_odt1 = L_odt1;
	}
	inline void setLodt2(vector<int> L_odt2)
	{
		this->L_odt2 = L_odt2;
	}
	inline void setUodt1(vector<int> U_odt1)
	{
		this->U_odt1 = U_odt1;
	}
	inline void setUodt2(vector<int> U_odt2)
	{
		this->U_odt2 = U_odt2;
	}
	inline void SetDeltaPOd(const vector<vector <vector< bool >> > delta_p_od)
	{
		this->delta_p_od = delta_p_od;
	}
	inline vector<int> GetRegion() const
	{
		return region;
	}
	inline vector<vector<pair<int, int>>> getDetermineDemandBound() const
	{
		return demand_bound;
	}
	inline void SetRegion(const vector<int> region)
	{
		this->region = region;
	}
	inline void SetDemandBound(const vector<vector<pair<int, int>>> demand_bound)
	{
		this->demand_bound = demand_bound;
	}
	inline vector<vector<pair<int, int>>> GetFreightBound() const
	{
		return freight_bound;
	}
	inline void SetFreightBound(const vector<vector<pair<int, int>>> freight_bound)
	{
		this->freight_bound = freight_bound;
	}
	inline vector<vector<int>> GetTransitTime() const
	{
		return transitTime;
	}
	inline void SetTransitTime(const vector<vector<int>> transit_time)
	{
		transitTime = transit_time;
	}
	inline const vector<vector<bool>>& GetThetaMp()
	{
		return theta_mp;
	}
	inline void SetThetaMp(const vector<vector<bool>>& theta_mp)
	{
		this->theta_mp = theta_mp;
	}
	inline double GetEpsilon1()
	{
		return epsilon1;
	}
	inline void SetEpsilon1(const double epsilon1)
	{
		this->epsilon1 = epsilon1;
	}
	inline pair<double, double> GetEpsilon2()
	{
		return epsilon2;
	}
	inline void SetEpsilon2(const pair<double, double>& epsilon2)
	{
		this->epsilon2 = epsilon2;
	}
	inline vector<double> GetVarianceDemand() const
	{
		return varianceDemand;
	}
	inline void SetVarianceDemand(const vector<double>& variance_demand)
	{
		varianceDemand = variance_demand;
	}
	inline vector<double> GetMeanRequest() const
	{
		return meanDemand;
	}
	inline void SetMeanRequest(const vector<double>& mean_request)
	{
		this->meanDemand = mean_request;
	}
	inline vector<vector< double >> GetCovarianceRequest() const
	{
		return covarianceDemand;
	}
	inline void SetCovarianceRequest(const vector<vector<double>> covariance)
	{
		this->covarianceDemand = covariance;
	}
	inline vector<int>& GetTimePointSet() 
	{
		return time_point_set;
	}
	inline vector<Request>& GetRequestSet()  {
		return RequestSet;
	}
	inline void SetTimePointSet(const vector<int>& time_point_set)
	{
		this->time_point_set = time_point_set;
	}
	inline void SetCallNodeSet(const vector<CallNode>& CallNodeSet)
	{
		this->CallNodeSet = CallNodeSet;
	}
	inline vector<int>& GetCapacity() { return capacitySet; }
	void calculateCapacity(vector<vector<int>>& VesselDecison);
	};

#endif // !_DRO_DATA_H_
