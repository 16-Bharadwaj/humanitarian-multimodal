# %%
import gurobipy as gp
from gurobipy import GRB

from dataclasses import dataclass, field
from typing import List, Dict, Tuple

import signal


# Global flag to handle interruption
interrupted = False


def signal_handler(sig, frame):
    global interrupted
    interrupted = True
    print("\nInterrupt received! Stopping optimization...")


# Register the signal handler for KeyboardInterrupt (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)


def my_callback(model, where):
    global interrupted
    if where == GRB.Callback.MIPNODE:
        if interrupted:
            model.terminate()  # Stop optimization safely


@dataclass(frozen=True)
class Vehicle:
    vehicle_name: str
    is_truck: bool = False
    capacity: int = 0
    speed: int = 0
    range: int = 0
    cost: float = 0.0


# Maybe reincorporate later
# @dataclass(frozen=True)
# class AircraftAssignment:
#     vehicle: Vehcile = field(default_factory=Vehcile)
#     # If negative 1, then assume it is a truck
#     number: int = -1

@dataclass(frozen=True)
class Facility:
    name: str
    supply: int = 0
    demand: int = 0

# A truck is basically an aircraft with a lower parameters
@dataclass
class Route:
    origin: Facility
    destination: Facility
    distance: int = 0
    # Dictionary of aircraft type to number of aircraft assigned
    aircraft_assigned: Dict[str, int] = field(default_factory=dict)

    @property
    def endpoints(self) -> Tuple[str, str]:
        return (self.origin.name, self.destination.name)


@dataclass(frozen=True)
class Scenario:
    name: str
    probability: float
    demand: int


@dataclass
class BuildData:
    facilities: List[Facility] = field(default_factory=list)
    
    scenarios: List[Scenario] = field(default_factory=list)
    aircraft_types: List[Vehicle] = field(default_factory=list)

    # Assume that the routes are subsets with size = 2 of the facilities
    routes: List[Route] = field(default_factory=list)
    
    truck_pool: int = 10000
    truck_capacity: int = 1000
    truck_cost: float = 10.0
    truck_speed: float = 10.0
    truck_range: float = 1000.0


# %%
class CapacityReservationModel:
    def __init__(self, data: BuildData):
        self.data: BuildData = data

        # Higher alpha can be interpreted as valuaing the revenue loss more than the demand
        # Initially set to 0.5 to signify equal weighting of both objectives
        self.alpha: float = 0.5  # Weighting factor for the objective function

    # We assume a constant unit of loss for each unit of demand that is not met
    loss_constant: float = 5000000000

    # NOTE: These dictionaries are temporary, will be replaced with new data structure
    # Aircraft stationed at each airport with respect to each aircraft type
    aircraft_at_airport: dict = {'A': {'1': 2, '2': 0, '3': 0},
                                'B': {'1': 0, '2': 0, '3': 0},
                                'C': {'1': 0, '2': 0, '3': 0}}
    
    # Capacity with respect to each aircraft type
    aircraft_capacity: dict = {'1': 200, '2': 200, '3': 300}

    scenario_probability: dict = {'alpha': 0.5, 'beta': 0.5}
    scenario_demand: dict = {'alpha': 200, 'beta': 200}    

    def model(self):
        model = gp.Model("Supply-Demand Optimization")

        # Decision Variables
        # Capacity reserved for use in an aircraft at an airport node
        k = model.addVars(((od, a) for od in self.aircraft_at_airport.keys() for a in self.aircraft_at_airport[od]),
            vtype=GRB.INTEGER, lb=0, name="reserved_capacity")
        
        # k = model.addVars(((od.origin_destination, a.aircraft.aircraft_type) for od in self.data.routes for a in od.aircraft_assigned),
        #     vtype=GRB.INTEGER, lb=0, name="reserved_capacity")
        
        lost_revenue = model.addVars(k.keys(), vtype=GRB.CONTINUOUS, name="lost_revenue")

        # Objective Functions
        model.setObjective(lost_revenue.sum(), GRB.MINIMIZE)

        # Constraints
        model.addConstrs((k[od, a] <= self.aircraft_capacity[a]
            for od in self.aircraft_at_airport.keys() for a in self.aircraft_at_airport[od]), "Capacity_Constraint")
        # for od in self.data.routes:
        #     for a in od.aircraft_assigned:
        #         model.addConstr(k[od.origin_destination, a.aircraft.aircraft_type] <= a.aircraft.capacity, "Capacity_Constraint")

        # Unless lost revenue is scaled in a non-linear way, demand will be allocated to the first available aircraft (greedy approach)
        model.addConstrs(((1 - self.alpha) * lost_revenue[od, a] >= self.alpha * k[od, a] for od in self.aircraft_at_airport.keys() for a in self.aircraft_at_airport[od]), "Lost_Revenue_Constraint")

        # This constraint estimates the lower bound on demand that must be met
        # Assume that it doesn't matter how much demand is met, as long as it is met
        # model.addConstr(lost_revenue >= sum(self.scenario_demand[scenario] for scenario in self.scenario_probability.keys()) - 
        #     gp.quicksum(k[(od, a)] for od in self.aircraft_at_airport.keys() for a in self.aircraft_at_airport[od]), "Demand_Constraint")
        model.addConstr(
            self.alpha * gp.quicksum(lost_revenue[od, a] 
                        for od in self.aircraft_at_airport.keys() 
                        for a in self.aircraft_at_airport[od]) 
            >= (1 - self.alpha) * (sum(self.scenario_demand[scenario] * self.scenario_probability[scenario] for scenario in self.scenario_probability.keys()) 
            - gp.quicksum(k[od, a] 
                        for od in self.aircraft_at_airport.keys() 
                        for a in self.aircraft_at_airport[od])), 
            "Demand_Constraint"
        )

        model.optimize(my_callback)

        if model.status == GRB.INFEASIBLE:
            model.computeIIS()
            model.write("model.ilp") 
            raise Exception("Model is infeasible. Check the model.ilp file for details.")
        elif model.status == GRB.OPTIMAL or model.status == GRB.INTERRUPTED:
            print("Model solved successfully.")
            print(f"Objective value: {model.ObjVal}")
            for v in model.getVars():
                if v.X > 0:
                    print(f"{v.VarName}: {v.X}")
        else:
            raise Exception("Model optimization failed.")
        
        return model, k

    
    def routing_model(self,
                      value_of_time: float = 1.0,
                      transfer_time: float = 0.0,
                      cost_of_unmet_demand: float = 10000.0): 
        model = gp.Model("Routing Optimization")

        # Decision Variables
        # NOTE: This is strictly a middle-mile analysis
        """
        Questions to answer:
        - Do we assume that the trucks are pre-assigned to a route or do we assign them to a route from a given pool?
            - Trucks come from a pool
        - Do we assume that there is insufficient capacity in the trucks to meet demand?
            - Relate via sensitivity analysis
            - Single trip/period initially -> add in slack variables
        - If so, do we strictly take a greedy approach? -> no
        - Else, do we want to consider cost via time or explicitly via dollars? -> vot
        """
        # Define mode as the integer total number of vehicles assigned to each route with respect to either truck or aircraft
        trucks_assigned = model.addVars((od.endpoints for od in self.data.routes), vtype=GRB.INTEGER, lb=0, name="trucks_assigned")
        aircraft_used = {}
        for od in self.data.routes:
            for veh in od.aircraft_assigned:
                key = od.endpoints + (veh.vehicle_name,)
                aircraft_used[key] = model.addVar(
                    vtype=GRB.INTEGER,
                    lb=0,
                    ub=od.aircraft_assigned[veh],
                    name=f"aircraft_used_{od.origin.name}_{od.destination.name}_{veh.vehicle_name}"
                )


        # Define where node d gets its supply from (x_d with respect to node o)
        x = model.addVars(((s.name, d.name) for s in self.data.facilities for d in self.data.facilities), vtype=GRB.CONTINUOUS, lb=0, name="flow_allocation")

        # Define z such that z_ij^d defines the fraction of flow going to node d via i,j
        z = model.addVars(((od.endpoints + (f.name,)) for od in self.data.routes for f in self.data.facilities), vtype=GRB.CONTINUOUS, lb=0, name="flow_routing")

        excess_supply = model.addVars((s.name for s in self.data.facilities), name="excess_supply", lb=0)

        excess_demand = model.addVars((s.name for s in self.data.facilities), name="excess_demand", lb=0)


        # Objective Function
        # We want to maximize demand met via a minimum amount of cost (time)
        model.setObjective(gp.quicksum(trucks_assigned[od.endpoints] * (od.distance * value_of_time * self.data.truck_speed + self.data.truck_cost) for od in self.data.routes)
                            + gp.quicksum(aircraft_used[(od.endpoints + (type.vehicle_name,))] * (od.distance * value_of_time * type.speed + type.cost) for od in self.data.routes for type in od.aircraft_assigned)
                            + gp.quicksum(excess_demand[s.name] * cost_of_unmet_demand for s in self.data.facilities), 
                            GRB.MINIMIZE)

        # Constraints
        # The total number of trucks assigned to routes must be less than or equal to the truck pool (i.e. the number of trucks available)
        model.addConstr((gp.quicksum(trucks_assigned[od.endpoints] for od in self.data.routes) <= self.data.truck_pool), name="Truck_Pool_Constraint")

        # All of the outgoing supply from node s must be less than or equal to its own supply minus its own demand
        for s in self.data.facilities:
            # Total outflow ≤ supply
            model.addConstr(
                gp.quicksum(x[(s.name, j.name)] for j in self.data.facilities)
                + excess_supply[s.name]
                == s.supply,
                name=f"Total_Supply_Balance_{s.name}"
            )

            # Total inflow ≥ demand
            model.addConstr(
                gp.quicksum(x[(i.name, s.name)] for i in self.data.facilities)
                + excess_demand[s.name]
                == s.demand,
                name=f"Total_Demand_Balance_{s.name}"
            )


        # # Ensure flow conservation
        # # NOTE In this initial formulation we allow for multiple transfers (all-stop configuration) with the goal of adding a penalty term later
        for o, d in x.keys():
            if o!= d:
                for i in self.data.facilities:
                    i_name = i.name
                    inflow = gp.quicksum(z[(u.name, i_name, d)] for u in self.data.facilities if u.name != i_name and (u.name, i_name, d) in z)
                    outflow = gp.quicksum(z[(i_name, j.name, d)] for j in self.data.facilities if j.name != i_name and (i_name, j.name, d) in z)

                    if i_name == o:
                        rhs = x[(o, d)]
                        # print(f'in rhs, {o,d}')
                    elif i_name == d:
                        rhs = -x[(o, d)]
                        # print(f'in out, {o,d}')
                    else:
                        rhs = 0

                    model.addConstr(outflow - inflow == rhs, name=f"Flow_Conservation_{o}_{d}_{i_name}")

        # # Vehicle sufficiency constraint
        for  od in self.data.routes:
            model.addConstr((gp.quicksum(z[od.endpoints + (f.name,)] for f in self.data.facilities) 
                              <= gp.quicksum(trucks_assigned[od.endpoints] * self.data.truck_capacity + aircraft_used[(od.endpoints + (type.vehicle_name,))] * type.capacity for type in od.aircraft_assigned)),
                              name=f"Vehicle_Sufficiency_Constraint_{od.origin.name}_{od.destination.name}")

        # Supplies inside a given vehicle must be less than or equal to the capacity of the vehicle
        # total cost to service demand must be less than or equal to the budget
        # NOTE for now not explicitly implemented

        model.optimize(my_callback)

        if model.status == GRB.INFEASIBLE:
            model.computeIIS()
            model.write("model.ilp") 
            raise Exception("Model is infeasible. Check the model.ilp file for details.")
        elif model.status == GRB.OPTIMAL or model.status == GRB.INTERRUPTED:
            print("Model solved successfully.")
            print(f"Objective value: {model.ObjVal}")
            for v in model.getVars():
                if v.X > 0:
                    print(f"{v.VarName}: {v.X}")
        else:
            raise Exception("Model optimization failed.")
        
        return model


if __name__ == '__main__':
    data = BuildData()
    data.scenarios = [Scenario('alpha', 0.5, 200), Scenario('beta', 0.5, 0)]
    data.aircraft_types = [Vehicle('1', 100), Vehicle('2', 200), Vehicle('3', 300)]

    data.facilities = [Facility('A', 100, 200), Facility('B', 200, 100)]

    data.routes = [
        Route(data.facilities[0], data.facilities[1], 100, {data.aircraft_types[0]: 1}),
        Route(data.facilities[1], data.facilities[0], 100, {data.aircraft_types[0]: 1})
    ]
    # for i in data.aircraft_types:
    #     data.routes.append(Route('A', 'B', [AircraftAssignment(i, 1)]))
    #     data.routes.append(Route('B', 'C', [AircraftAssignment(i, 1)]))
    #     data.routes.append(Route('C', 'A', [AircraftAssignment(i, 1)]))

    o1 = CapacityReservationModel(data)
    # model, k = o1.model()
    model = o1.routing_model()
# %%
