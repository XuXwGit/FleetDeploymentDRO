/*****************************************************************//**
 *  @file					: OD.h
 *  @namespace		: DRO
 *  @brief				: information about transportation from one port (origin) to another port (destination)
 * 
 *  @author				: XuXw
 *  @version			: 1.0.0
 *  @date					: 2023/7/28
 *********************************************************************/

#ifndef DRO_OD_H_
#define DRO_OD_H_

#include "include.h"
#include "port.h"

class OD
{
public:
	OD() {
	}

	inline OD(int o_call, int d_call,Port& o, Port& d)
		:	origin_call_index(o_call),
			destination_call_index(d_call),
			originPortID(o.getPortID()),
			destinationPortID(d.getPortID()),
			origin(o.getPort()),
			destination(d.getPort()),
			originPort(o),
			destiantionPort(d),
			demand(-1)			
	{
	}

	int GetOriginCallIndex() const
	{
		return origin_call_index;
	}

	void SetOriginCallIndex(const int origin_call_index)
	{
		this->origin_call_index = origin_call_index;
	}

	int GetDestinationCallIndex() const
	{
		return destination_call_index;
	}

	void SetDestinationCallIndex(const int destination_call_index)
	{
		this->destination_call_index = destination_call_index;
	}

private:
	int origin_call_index;
	int destination_call_index;
	int originPortID;
	int destinationPortID;
	string origin;
	string destination;
	double unitRentalCost;
	double unitPenaltyCost;
	int demand;
	Port originPort;
	Port destiantionPort;

public:
	int GetOriginPortId() const
	{
		return originPortID;
	}

	void SetOriginPortId(const int origin_port_id)
	{
		originPortID = origin_port_id;
	}

	int GetDestinationPortId() const
	{
		return destinationPortID;
	}

	void SetDestinationPortId(const int destination_port_id)
	{
		destinationPortID = destination_port_id;
	}

	string GetOrigin() const
	{
		return origin;
	}

	void SetOrigin(const string& origin)
	{
		this->origin = origin;
	}

	string GetDestination() const
	{
		return destination;
	}

	void SetDestination(const string& destination)
	{
		this->destination = destination;
	}

	double GetUnitRentalCost() const
	{
		return unitRentalCost;
	}

	void SetUnitRentalCost(const double unit_rental_cost)
	{
		unitRentalCost = unit_rental_cost;
	}

	double GetUnitPenaltyCost() const
	{
		return unitPenaltyCost;
	}

	void SetUnitPenaltyCost(const double unit_penalty_cost)
	{
		unitPenaltyCost = unit_penalty_cost;
	}

	int getDetermineDemand() const
	{
		return demand;
	}

	void SetDemand(const int demand)
	{
		this->demand = demand;
	}

	Port GetOriginPort() const
	{
		return originPort;
	}

	void SetOriginPort(const Port& origin_port)
	{
		originPort = origin_port;
	}

	Port GetDestiantionPort() const
	{
		return destiantionPort;
	}

	void SetDestiantionPort(const Port& destiantion_port)
	{
		destiantionPort = destiantion_port;
	}


public:
	inline Port& getOriginPort() { return this->originPort; }
	inline Port& getDestinationPort() { return this->destiantionPort; }
	inline double getUnitRentalCost() { return this->unitRentalCost; }
	inline double getUnitPenaltyCost() { return this->unitPenaltyCost; }
	inline int getDetermineDemand()
	{
		return demand;
	}
};


#endif // !DRO_OD_H_
