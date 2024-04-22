/*****************************************************************//**
 *  @file					: Request.h
 *  @namespace		: DRO
 *  @brief				: the demand class
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/28
 *********************************************************************/

#ifndef FDP_DRO_REQUEST_H_
#define FDP_DRO_REQUEST_H_

#include "CallNode.h"

class Request
{
public:

private:
	int requestID;
	CallNode origin;
	CallNode destination;
	CallNode releaseNode;
	int meanDemand;
	bool whetherGenerateEmptyContainer;
	vector<int> uncertainDemandSet;
public:

	Request(int requestID, CallNode& origin, CallNode& destination)
		: requestID(requestID), origin(origin), destination(destination), releaseNode(destination), meanDemand(0), whetherGenerateEmptyContainer(false), uncertainDemandSet(vector<int>())
	{
	}

	Request(int requestID, CallNode& origin, CallNode& destination, int meanDemand, vector<int>& uncertainDemandSet)
		: requestID(requestID), origin(origin), destination(destination), releaseNode(destination), meanDemand(meanDemand), whetherGenerateEmptyContainer(false), uncertainDemandSet(uncertainDemandSet)
	{
	}

	Request(int requestID, CallNode& origin, CallNode& destination, CallNode& releaseNode, int meanDemand, vector<int>& uncertainDemandSet)
		: requestID(requestID), origin(origin), destination(destination), releaseNode(releaseNode), meanDemand(meanDemand), whetherGenerateEmptyContainer(false), uncertainDemandSet(uncertainDemandSet)
	{
	}

	Request(int requestID, CallNode& origin, CallNode& destination, CallNode& releaseNode, int meanDemand, bool whetherGenerateEmptyContainer, vector<int>& uncertainDemandSet)
		: requestID(requestID), origin(origin), destination(destination), releaseNode(releaseNode), meanDemand(meanDemand), whetherGenerateEmptyContainer(whetherGenerateEmptyContainer), uncertainDemandSet(uncertainDemandSet)
	{
	}

	inline  CallNode& getOrigin() {
		return origin;
	}
	inline  CallNode& getDestination() {
		return destination;
	}
	inline CallNode& getReleaseNode() {
		return releaseNode;
	}
	inline const int getMeanDemand() {
		return meanDemand;
	}
	inline vector<int>& getUncertainDemandSet() {
		return this->uncertainDemandSet;
	}
	inline void setMeanDemand(int demand) {
		this->meanDemand = demand;
	}
	inline void setUncertainDemandSet(vector<int>& demandSet) {
		this->uncertainDemandSet = demandSet;
	}
	inline bool getWhetherEmptyContainer() {
		return whetherGenerateEmptyContainer;
	}
};

#endif // !FDP_DRO_REQUEST_H_
