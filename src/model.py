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
class Aircraft:
    aircraft_type: str
    capacity: int
    speed: int = 0
    range: int = 0


@dataclass(frozen=True)
class AircraftAssignment:
    aircraft: Aircraft
    number: int


@dataclass(frozen=True)
class Route:
    origin_destination: Tuple[str, str]
    distance: int = 0
    aircraft_assigned: List[AircraftAssignment]


@dataclass(frozen=True)
class Scenario:
    name: str
    probability: float
    demand: int


@dataclass
class BuildData:
    scenarios: List[Scenario] = field(default_factory=list)
    aircraft_types: List[Aircraft] = field(default_factory=list)
    routes: List[Route] = field(default_factory=list)


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

    
    def routing_model(self): 
        model = gp.Model("Routing Optimization")

        # Decision Variables
        # NOTE: This is strictly a middle-mile analysis
        """
        Questions to answer:
        - Do we assume that the trucks are pre-assigned to a route or do we assign them to a route from a given pool?
        - Do we assume that there is insufficient capacity in the trucks to meet demand?
        - If so, do we strictly take a greedy approach? 
        - Else, do we want to consider cost via time or explicitly via dollars?
        """

        # Objective Function
        # We want to maximize demand met via a minimum amount of cost (time)
        model.setObjective(0, GRB.MINIMIZE)

        # Constraints
        # Supplies inside a given vehicle must be less than or equal to the capacity of the vehicle

        # total cost to service demand must be less than or equal to the budget





if __name__ == '__main__':
    data = BuildData()
    data.scenarios = [Scenario('alpha', 0.5, 200), Scenario('beta', 0.5, 0)]
    data.aircraft_types = [Aircraft('1', 100), Aircraft('2', 200), Aircraft('3', 300)]

    for i in data.aircraft_types:
        data.routes.append(Route(('A', 'B'), [AircraftAssignment(i, 1)]))
        data.routes.append(Route(('B', 'C'), [AircraftAssignment(i, 1)]))
        data.routes.append(Route(('C', 'A'), [AircraftAssignment(i, 1)]))

    o1 = CapacityReservationModel(data)
    model, k = o1.model()
# %%
