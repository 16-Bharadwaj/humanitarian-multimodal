# %%
import gurobipy as gp
from gurobipy import GRB

from dataclasses import dataclass, field
from typing import List, Dict, Tuple

import signal

import matplotlib.pyplot as plt
import numpy as np
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)

# %%
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

# %%
@dataclass(frozen=True)
class Vehicle:
    vehicle_name: str
    is_truck: bool = False
    capacity: int = 0
    speed: int = 1
    range: int = 1
    cost: float = 1.0


# Maybe reincorporate later
# @dataclass(frozen=True)
# class AircraftAssignment:
#     vehicle: Vehcile = field(default_factory=Vehcile)
#     # If negative 1, then assume it is a truck
#     number: int = -1

@dataclass(frozen=True)
class Facility:
    name: str
    is_airport: bool = False
    supply: int = 0
    demand: int = 0

@dataclass
class Route:
    origin: Facility
    destination: Facility
    distance: int = 0
    aircraft_assigned: Dict[str, int] = field(default_factory=dict)  # Use vehicle name as key

    @property
    def is_air_route(self) -> bool:
        return self.origin.is_airport and self.destination.is_airport

    @property
    def endpoints(self) -> Tuple[str, str]:
        return (self.origin.name, self.destination.name)

@dataclass
class Scenario:
    name: str
    probability: float
    demand: float


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
class ReservationModel:
    def __init__(self, data: BuildData):
        self.data: BuildData = data

        # Higher alpha can be interpreted as valuaing the revenue loss more than the demand
        # Initially set to 0.5 to signify equal weighting of both objectives
        self.alpha: float = 0.5  # Weighting factor for the objective function
        self.cost_of_unmet_demand: float = 100.0  # Cost of unmet demand, can be adjusted based on scenario
        self.cost_lost_revenue: float = 100.0  # Cost of lost revenue, can be adjusted based on scenario

    def capacity_reservation_model(self):
        model = gp.Model("Supply-Demand Optimization")

        # Decision Variables
        # Capacity reserved for use in an aircraft at an airport node
        aircraft_by_route = [
            (route.endpoints[0], route.endpoints[1], aircraft_name, i)
            for route in self.data.routes if route.is_air_route
            for aircraft_name, count in route.aircraft_assigned.items()
            for i in range(count)
        ]
       
        k = model.addVars(aircraft_by_route, vtype=GRB.INTEGER, lb=0, name="reserved_capacity")        
        loss = model.addVars(k.keys(), vtype=GRB.CONTINUOUS, name="lost_value")

        # Objective Functions
        model.setObjective(loss.sum(), GRB.MINIMIZE)

        # Constraints
        aircraft_dict = {v.vehicle_name: v for v in self.data.aircraft_types}
        model.addConstrs((k[o, d, a, i] <= aircraft_dict[a].capacity for o, d, a, i in k.keys()), "Capacity_Constraint")

        # Unless lost revenue is scaled in a non-linear way, demand will be allocated to the first available aircraft (greedy approach)
        model.addConstrs(((1 - self.alpha) * loss[o, d, a, i] >= self.cost_lost_revenue * self.alpha * k[o, d, a, i] for o, d, a, i in k.keys()), "Lost_Revenue_Constraint")

        # This constraint estimates the lower bound on demand that must be met
        # Assume that it doesn't matter how much demand is met, as long as it is met
        # model.addConstr(lost_revenue >= sum(self.scenario_demand[scenario] for scenario in self.scenario_probability.keys()) - 
        #     gp.quicksum(k[(od, a)] for od in self.aircraft_at_airport.keys() for a in self.aircraft_at_airport[od]), "Demand_Constraint")
        model.addConstr(
            self.alpha * gp.quicksum(loss[o, d, a, i] for o, d, a, i in k.keys())
            >= self.cost_of_unmet_demand * (1 - self.alpha) * (sum(self.data.scenarios[scenario].demand * self.data.scenarios[scenario].probability for scenario in range(len(self.data.scenarios)))
            - gp.quicksum(k[o, d, a, i] for o, d, a, i in k.keys())),
            name="Demand_Constraint"
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
        
        return model, k, loss

    
    # def routing_model(self,
    #                   value_of_time: float = 1.0,
    #                   transfer_time: float = 0.0,
    #                   cost_of_unmet_demand: float = 10000.0): 
    #     model = gp.Model("Routing Optimization")

    #     # Decision Variables
    #     # NOTE: This is strictly a middle-mile analysis
    #     """
    #     Questions to answer:
    #     - Do we assume that the trucks are pre-assigned to a route or do we assign them to a route from a given pool?
    #         - Trucks come from a pool
    #     - Do we assume that there is insufficient capacity in the trucks to meet demand?
    #         - Relate via sensitivity analysis
    #         - Single trip/period initially -> add in slack variables
    #     - If so, do we strictly take a greedy approach? -> no
    #     - Else, do we want to consider cost via time or explicitly via dollars? -> vot
    #     """
    #     # Define mode as the integer total number of vehicles assigned to each route with respect to either truck or aircraft
    #     trucks_assigned = model.addVars((od.endpoints for od in self.data.routes), vtype=GRB.INTEGER, lb=0, name="trucks_assigned")
    #     aircraft_used = {}
    #     for od in self.data.routes:
    #         for veh in od.aircraft_assigned:
    #             key = od.endpoints + (veh.vehicle_name,)
    #             aircraft_used[key] = model.addVar(
    #                 vtype=GRB.INTEGER,
    #                 lb=0,
    #                 ub=od.aircraft_assigned[veh],
    #                 name=f"aircraft_used_{od.origin.name}_{od.destination.name}_{veh.vehicle_name}"
    #             )


    #     # Define where node d gets its supply from (x_d with respect to node o)
    #     x = model.addVars(((s.name, d.name) for s in self.data.facilities for d in self.data.facilities), vtype=GRB.CONTINUOUS, lb=0, name="flow_allocation")

    #     # Define z such that z_ij^d defines the fraction of flow going to node d via i,j
    #     z = model.addVars(((od.endpoints + (f.name,)) for od in self.data.routes for f in self.data.facilities), vtype=GRB.CONTINUOUS, lb=0, name="flow_routing")

    #     excess_supply = model.addVars((s.name for s in self.data.facilities), name="excess_supply", lb=0)

    #     excess_demand = model.addVars((s.name for s in self.data.facilities), name="excess_demand", lb=0)


    #     # Objective Function
    #     # We want to maximize demand met via a minimum amount of cost (time)
    #     model.setObjective(gp.quicksum(trucks_assigned[od.endpoints] * (od.distance * value_of_time / self.data.truck_speed + self.data.truck_cost) for od in self.data.routes)
    #                         + gp.quicksum(aircraft_used[(od.endpoints + (type.vehicle_name,))] * (od.distance * value_of_time / type.speed + type.cost) for od in self.data.routes for type in od.aircraft_assigned)
    #                         + gp.quicksum(excess_demand[s.name] * cost_of_unmet_demand for s in self.data.facilities), 
    #                         GRB.MINIMIZE)

    #     # Constraints
    #     # The total number of trucks assigned to routes must be less than or equal to the truck pool (i.e. the number of trucks available)
    #     model.addConstr((gp.quicksum(trucks_assigned[od.endpoints] for od in self.data.routes) <= self.data.truck_pool), name="Truck_Pool_Constraint")

    #     # All of the outgoing supply from node s must be less than or equal to its own supply minus its own demand
    #     for s in self.data.facilities:
    #         # Total outflow ≤ supply
    #         model.addConstr(
    #             gp.quicksum(x[(s.name, j.name)] for j in self.data.facilities)
    #             + excess_supply[s.name]
    #             == s.supply,
    #             name=f"Total_Supply_Balance_{s.name}"
    #         )

    #         # Total inflow ≥ demand
    #         model.addConstr(
    #             gp.quicksum(x[(i.name, s.name)] for i in self.data.facilities)
    #             + excess_demand[s.name]
    #             == s.demand,
    #             name=f"Total_Demand_Balance_{s.name}"
    #         )


    #     # # Ensure flow conservation
    #     # # NOTE In this initial formulation we allow for multiple transfers (all-stop configuration) with the goal of adding a penalty term later
    #     for o, d in x.keys():
    #         if o!= d:
    #             for i in self.data.facilities:
    #                 i_name = i.name
    #                 inflow = gp.quicksum(z[(u.name, i_name, d)] for u in self.data.facilities if u.name != i_name and (u.name, i_name, d) in z)
    #                 outflow = gp.quicksum(z[(i_name, j.name, d)] for j in self.data.facilities if j.name != i_name and (i_name, j.name, d) in z)

    #                 if i_name == o:
    #                     rhs = x[(o, d)]
    #                     # print(f'in rhs, {o,d}')
    #                 elif i_name == d:
    #                     rhs = -x[(o, d)]
    #                     # print(f'in out, {o,d}')
    #                 else:
    #                     rhs = 0

    #                 model.addConstr(outflow - inflow == rhs, name=f"Flow_Conservation_{o}_{d}_{i_name}")

    #     # # Vehicle sufficiency constraint
    #     # it should be fine to consider all aircraft as if there is no reserved_capacity it is cut off due to unnecessary cost
    #     # NOTE: This is a temporary solution, we will need to add in the reserved capacity later
    #     for  od in self.data.routes:
    #         model.addConstr((gp.quicksum(z[od.endpoints + (f.name,)] for f in self.data.facilities) 
    #                           <= gp.quicksum(trucks_assigned[od.endpoints] * self.data.truck_capacity + aircraft_used[(od.endpoints + (type.vehicle_name,))] * type.reserved_capacity for type in od.aircraft_assigned)),
    #                           name=f"Vehicle_Sufficiency_Constraint_{od.origin.name}_{od.destination.name}")

    #     # Supplies inside a given vehicle must be less than or equal to the capacity of the vehicle
    #     # total cost to service demand must be less than or equal to the budget
    #     # NOTE for now not explicitly implemented

    #     model.optimize(my_callback)

    #     if model.status == GRB.INFEASIBLE:
    #         model.computeIIS()
    #         model.write("model.ilp") 
    #         raise Exception("Model is infeasible. Check the model.ilp file for details.")
    #     elif model.status == GRB.OPTIMAL or model.status == GRB.INTERRUPTED:
    #         print("Model solved successfully.")
    #         print(f"Objective value: {model.ObjVal}")
    #         for v in model.getVars():
    #             if v.X > 0:
    #                 print(f"{v.VarName}: {v.X}")
    #     else:
    #         raise Exception("Model optimization failed.")
        
    #     return model


def flood_case_study() -> BuildData:
    data = BuildData()

    # Aircraft
    data.aircraft_types = [
        Vehicle("ATR72_Freighter", is_truck=False, capacity=75, speed=5, range=1500, cost=1800.0),
        Vehicle("B737_800BCF", is_truck=False, capacity=180, speed=8, range=3700, cost=3000.0),
        Vehicle("TwinOtter_DHC6", is_truck=False, capacity=30, speed=3, range=1000, cost=900.0),

        Vehicle("Isuzu_MediumTruck", is_truck=True, capacity=40, speed=2, range=700, cost=500.0),
        Vehicle("Mercedes_Axor_1840", is_truck=True, capacity=80, speed=2, range=1000, cost=700.0)
    ]

    # Facilities
    # Perhaps include field hubs later?
    data.facilities = [
        Facility(name="Warehouse_1", is_airport=False, supply=300),
        Facility(name="Warehouse_2", is_airport=False, supply=250),
        Facility(name="Dest_1", is_airport=False, demand=200),
        Facility(name="Dest_2", is_airport=False, demand=230),
        Facility(name="Airport_A", is_airport=True, supply=0, demand=0),
        Facility(name="Airport_B", is_airport=True, supply=0, demand=0),
        Facility(name="Field_Hub_1", is_airport=False, supply=0, demand=0),
        Facility(name="Field_Hub_2", is_airport=False, supply=0, demand=0)
    ]

    # Routes with preassigned aircraft on air routes
    # Create a quick lookup dict for facilities by name
    facility_dict = {f.name: f for f in data.facilities}

    # Routes with preassigned aircraft on air routes
    data.routes = [
        # Warehouse to airport by truck
        Route(facility_dict["Warehouse_1"], facility_dict["Airport_A"], 110),
        Route(facility_dict["Warehouse_2"], facility_dict["Airport_B"], 130),

        # Air routes with preassigned aircraft
        Route(facility_dict["Airport_A"], facility_dict["Airport_B"], 950, aircraft_assigned={
            "ATR72_Freighter": 2,
            "B737_800BCF": 1
        }),
        Route(facility_dict["Airport_A"], facility_dict["Field_Hub_1"], 600, aircraft_assigned={
            "TwinOtter_DHC6": 3
        }),
        Route(facility_dict["Airport_B"], facility_dict["Field_Hub_2"], 580, aircraft_assigned={
            "ATR72_Freighter": 1
        }),

        # Airport to Field Hub by truck (backup/overland)
        Route(facility_dict["Airport_A"], facility_dict["Field_Hub_1"], 1000),
        Route(facility_dict["Airport_B"], facility_dict["Field_Hub_2"], 900),

        # # Field hub to final destination (last mile, truck)
        Route(facility_dict["Field_Hub_1"], facility_dict["Dest_1"], 40),
        Route(facility_dict["Field_Hub_2"], facility_dict["Dest_2"], 35),
    ]

    return data

# def sensitivity_analysis():
data = flood_case_study()
total_demand = sum(f.demand for f in data.facilities if f.demand > 0)
# things to permute:
# - Demand levels (e.g., increase/decrease by 10%, 20%)
# - Probabilities of scenarios (e.g., increase/decrease by 5%, 10%)
# - Costs of unmet demand and lost revenue (e.g., increase/decrease by 10%, 20%)
# - Alpha weighting factor (e.g., increase/decrease by 0.1, 0.2)

scenarios = [
    Scenario(
        name="Expected_Demand",
        probability=0.5, 
        demand=1.0 * total_demand
    ),  # Baseline scenario: normal demand levels

    # Modify demand levels
    Scenario(
        name="Expected_Demand+5%",
        probability=0.5, 
        demand=1.05 * total_demand
    ),

    Scenario(
        name="Expected_Demand+10%",
        probability=0.5, 
        demand=1.1 * total_demand
    ),

    Scenario(
        name="Expected_Demand+20%",
        probability=0.5, 
        demand=1.2 * total_demand
    ),

    Scenario(
        name="Expected_Demand-5%",
        probability=0.5, 
        demand=0.95 * total_demand
    ),

    Scenario(
        name="Expected_Demand-10%",
        probability=0.5, 
        demand=0.9 * total_demand
    ),

    Scenario(
        name="Expected_Demand-20%",
        probability=0.5, 
        demand=0.8 * total_demand
    ),

    # Modify probabilities of scenarios
    Scenario(
        name="Expected_Demand-p=0.55",
        probability=0.55,
        demand=1.0 * total_demand
    ),

    Scenario(
        name="Expected_Demand-p=0.65",
        probability=0.65,
        demand=1.0 * total_demand
    ),

    Scenario(
        name="Expected_Demand-p=0.7",
        probability=0.7,
        demand=1.0 * total_demand
    ),

    Scenario(
        name="Expected_Demand-p=0.45",
        probability=0.45,
        demand=1.0 * total_demand
    ),

    Scenario(
        name="Expected_Demand-p=0.35",
        probability=0.35,
        demand=1.0 * total_demand
    ),

    Scenario(
        name="Expected_Demand-p=0.3",
        probability=0.3,
        demand=1.0 * total_demand
    ),

    # Scenario(
    #     name="", 
    #     probability=0.25, 
    #     demand=1.5 * total_demand
    # ),  # Peak demand or emergency scenario with 50% higher demand

    # Scenario(
    #     name="Low_Demand", 
    #     probability=0.1, 
    #     demand=0.7 * total_demand
    # ),  # Off-season or reduced demand scenario with 30% lower demand

    # Scenario(
    #     name="Best_Case", 
    #     probability=0.05, 
    #     demand=0.0 * total_demand
    # ),  # Ideal scenario with no demand, no disaster happens
            
    # Airport_A closed or disrupted, resulting in zero demand served through it
]

res = ReservationModel(data)
demand_outputs = []
prob_outputs = []

for scenario in scenarios:
    res.data.scenarios = [scenario]
    _, k, loss = res.capacity_reservation_model()

    total_reserved = sum(val.X for val in k.values())
    total_loss = sum(val.X for val in loss.values())

    if 'p=' in scenario.name:
        prob_outputs.append((scenario.probability, total_reserved, total_loss))
    else:
        demand_outputs.append((scenario.demand, total_reserved, total_loss))

# Sort results
prob_outputs.sort()
demand_outputs.sort()

# %%
# Plot reserved capacity and loss vs probability
if prob_outputs:
    probs, reserved_k, losses = zip(*prob_outputs)
    probs = list(probs)  # Convert to list for plotting
    fig, ax1 = plt.subplots()
    ax1.plot(probs, reserved_k, 'b-o', label='Reserved Capacity')
    ax1.set_xlabel("Scenario Probability")
    ax1.set_ylabel("Reserved Capacity", color='b')
    ax2 = ax1.twinx()
    ax2.plot(probs, losses, 'r-s', label='Loss')
    ax2.set_ylabel("Loss", color='r')
    plt.title("Sensitivity: Reserved Capacity and Loss vs Probability")
    fig.tight_layout()
    plt.grid(True)
    plt.savefig("../output/figures/sensitivity_probabilities.pdf", bbox_inches='tight', dpi=300)
    plt.show()

# Plot reserved capacity and loss vs demand
if demand_outputs:
    demands, reserved_k, losses = zip(*demand_outputs)
    fig, ax1 = plt.subplots()
    ax1.plot(demands, reserved_k, 'b-o', label='Reserved Capacity')
    ax1.set_xlabel("Scenario Demand")
    ax1.set_ylabel("Reserved Capacity", color='b')
    ax2 = ax1.twinx()
    ax2.plot(demands, losses, 'r-s', label='Loss')
    ax2.set_ylabel("Loss", color='r')
    plt.title("Sensitivity: Reserved Capacity and Loss vs Demand")
    fig.tight_layout()
    plt.grid(True)
    plt.savefig("../output/figures/sensitivity_demands.pdf", bbox_inches='tight', dpi=300)
    plt.show()

# %%
value_set = [0.3, 0.35, 0.45, 0.5, 0.55, 0.65, 0.7]

res.data.scenarios = [scenarios[0]]

alpha_outputs = []
cost_unmet_outputs = []
cost_lost_outputs = []

for v in value_set:
    res.cost_of_unmet_demand = 100.0  # Reset to default for next analysis
    res.cost_lost_revenue = 100.0  # Reset to default for next analysis
    res.alpha = v
    _, k, loss = res.capacity_reservation_model()

    total_reserved = sum(val.X for val in k.values())
    total_loss = sum(val.X for val in loss.values())

    alpha_outputs.append((v, total_reserved, total_loss))

    #####
    res.alpha = 0.5  # Reset to default for next analysis
    res.cost_lost_revenue = 100.0  # Reset to default for next analysis
    res.cost_of_unmet_demand = v * 100.0  # Scale cost of unmet demand
    _, k, loss = res.capacity_reservation_model()

    total_reserved = sum(val.X for val in k.values())
    total_loss = sum(val.X for val in loss.values())

    cost_unmet_outputs.append((v*100, total_reserved, total_loss))

    #####
    res.cost_of_unmet_demand = 100.0  # Reset to default for next analysis
    res.cost_lost_revenue = v * 100.0  # Scale cost of lost revenue

    _, k, loss = res.capacity_reservation_model()
    total_reserved = sum(val.X for val in k.values())
    total_loss = sum(val.X for val in loss.values())

    cost_lost_outputs.append((v*100, total_reserved, total_loss))


# Plot alpha sensitivity
if alpha_outputs:
    alphas, reserved_k, losses = zip(*alpha_outputs)
    fig, ax1 = plt.subplots()
    ax1.plot(alphas, reserved_k, 'b-o', label='Reserved Capacity')
    ax1.set_xlabel("Alpha Value")
    ax1.set_ylabel("Reserved Capacity", color='b')
    ax2 = ax1.twinx()
    ax2.plot(alphas, losses, 'r-s', label='Loss')
    ax2.set_ylabel("Loss", color='r')
    plt.title("Sensitivity: Reserved Capacity and Loss vs Alpha")
    fig.tight_layout()
    plt.grid(True)
    plt.savefig("../output/figures/sensitivity_alpha.pdf", bbox_inches='tight', dpi=300)
    plt.show()

# Plot cost of unmet demand sensitivity
if cost_unmet_outputs:
    costs, reserved_k, losses = zip(*cost_unmet_outputs)
    fig, ax1 = plt.subplots()
    ax1.plot(costs, reserved_k, 'b-o', label='Reserved Capacity')
    ax1.set_xlabel("Cost of Unmet Demand")
    ax1.set_ylabel("Reserved Capacity", color='b')
    ax2 = ax1.twinx()
    ax2.plot(costs, losses, 'r-s', label='Loss')
    ax2.set_ylabel("Loss", color='r')
    plt.title("Sensitivity: Reserved Capacity and Loss vs Cost of Unmet Demand")
    fig.tight_layout()
    plt.grid(True)
    plt.savefig("../output/figures/sensitivity_cost_unmet.pdf", bbox_inches='tight', dpi=300)
    plt.show()

# Plot cost of lost revenue sensitivity
if cost_lost_outputs:
    costs, reserved_k, losses = zip(*cost_lost_outputs)
    fig, ax1 = plt.subplots()
    ax1.plot(costs, reserved_k, 'b-o', label='Reserved Capacity')
    ax1.set_xlabel("Cost of Lost Revenue")
    ax1.set_ylabel("Reserved Capacity", color='b')
    ax2 = ax1.twinx()
    ax2.plot(costs, losses, 'r-s', label='Loss')
    ax2.set_ylabel("Loss", color='r')
    plt.title("Sensitivity: Reserved Capacity and Loss vs Cost of Lost Revenue")
    fig.tight_layout()
    plt.grid(True)
    plt.savefig("../output/figures/sensitivity_cost_lost.pdf", bbox_inches='tight', dpi=300)
    plt.show()

# if __name__ == '__main__':
#     sensitivity_analysis()
    # model = o1.routing_model()
# %%
