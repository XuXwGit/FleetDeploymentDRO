/*****************************************************************//**
 *  @file					: vessel.h
 *  @namespace		: DRO
 *  @brief				: vessel class, which includes : capacity, fixed operation cost
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/28
 *********************************************************************/

#ifndef _VESSEL_H_
#define _VESSEL_H_

#include <iomanip>

class Vessel
{
public:
	Vessel()
		: vesselIndex(-1),
		routeID(-1),
		capacity(-1),
		maxNum(-1),
		operationCost(-1)
	{
	}

	inline Vessel(const int index, const int routeID, const int capacity, const int maxNum,
		const double operationCost)
		: vesselIndex(index),
		routeID(routeID),
		capacity(capacity),
		maxNum(maxNum),
		operationCost(operationCost)
	{
	}

private:
	int vesselIndex;
	int capacity;
	int maxNum;
	int routeID;
	double operationCost;

public:
	int GetVesselIndex() const
	{
		return vesselIndex;
	}

	void SetVesselIndex(const int vessel_index)
	{
		vesselIndex = vessel_index;
	}

	int GetCapacity() const
	{
		return capacity;
	}

	void SetCapacity(const int capacity)
	{
		this->capacity = capacity;
	}

	int getMaxNum() const
	{
		return maxNum;
	}

	void SetMaxNum(const int max_number)
	{
		maxNum = max_number;
	}

	int GetRouteId() const
	{
		return routeID;
	}

	void SetRouteId(const int route_id)
	{
		routeID = route_id;
	}

	double GetOperationCost() const
	{
		return operationCost;
	}

	void SetOperationCost(const double operation_cost)
	{
		operationCost = operation_cost;
	}

	void ShowVessel()
	{
		cout << fixed << setprecision(2);
		cout << "(" << routeID << ")"
		<< vesselIndex << "\t"
		<< capacity << "\t"
		<< operationCost <<"\t"
		<< routeID<<"\t"
		<< maxNum<< endl;
	}
};

#endif // !_VESSEL_H_
