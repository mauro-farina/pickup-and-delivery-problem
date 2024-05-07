import gurobipy as gb

from itertools import product

from abstract_model import AbstractModel
from graph import Graph, NodeType
from request import Request
from vehicle import Vehicle


class Lyu(AbstractModel):

    def __init__(self, graph: Graph, vehicles: set[Vehicle], requests: set[Request]):

        super().__init__()
        model = gb.Model('Lyu', env=gb.Env(params={'OutputFlag': 0}))
        model.modelSense = gb.GRB.MINIMIZE

        model.setParam('OutputFlag', 0)
        model.setParam('TimeLimit', 3600)

        transfer_stations = {t for t in graph.nodes if t.type is NodeType.TRANSFER_STATION}
        depot_nodes = {n for n in graph.nodes
                       if n.type is NodeType.ORIGIN_DEPOT or n.type is NodeType.DESTINATION_DEPOT}

        # x_k_i_j = 1 if vehicle k travels through arc (i,j)
        x = model.addVars(
            [(arc.src.index, arc.dst.index, k.index)
             for arc in graph.arcs
             for k in vehicles],
            lb=0, ub=1, vtype=gb.GRB.BINARY
        )

        # y_r_k_i_j = 1 if request r is transported by vehicle k through arc (i,j)
        y = model.addVars(
            [(arc.src.index, arc.dst.index, k.index, r.index)
             for arc in graph.arcs
             for k in vehicles
             for r in requests],
            lb=0, ub=1, vtype=gb.GRB.BINARY
        )

        # s_t_r_k1_k2
        s = model.addVars(
            [(t.index, r.index, k1.index, k2.index)
             for t in transfer_stations
             for r in requests
             for k1 in vehicles
             for k2 in vehicles if k2 != k1],
            lb=0, ub=1, vtype=gb.GRB.BINARY
        )

        # a_k_i represent the arrival time for vehicle k at location i
        a = model.addVars(
            [(n.index, k.index)
             for n in graph.nodes
             for k in vehicles],
            lb=0, ub=float('inf'), vtype=gb.GRB.CONTINUOUS
        )

        # b_k_i represent the departure time for vehicle k at location i
        b = model.addVars(
            [(n.index, k.index)
             for n in graph.nodes
             for k in vehicles],
            lb=0, ub=float('inf'), vtype=gb.GRB.CONTINUOUS
        )

        model.setObjective(
            gb.quicksum(
                arc.cost * x[arc.src.index, arc.dst.index, k.index] * k.travel_unit_cost
                for arc in graph.arcs
                for k in vehicles)
        )

        # (4) ∑∈K ∑(i,j)∈A y_k_r_i_j = 1 ∀r ∈ R, i = p(r)
        for r in requests:
            model.addConstr(
                gb.quicksum(
                    y[arc.src.index, arc.dst.index, k.index, r.index]
                    for k in vehicles
                    for arc in graph.arcs if arc.src == r.pickup
                )
                == 1,
                '(4)'
            )

        # (5) ∑∈K ∑(j,i)∈A y_k_r_j_i = 1 ∀r ∈ R, i = d(r)
        for r in requests:
            model.addConstr(
                gb.quicksum(
                    y[arc.src.index, arc.dst.index, k.index, r.index]
                    for k in vehicles
                    for arc in graph.arcs if arc.dst == r.destination
                )
                == 1,
                '(5)'
            )

        # (6) ∑k∈K ∑(i,j)∈A y_k_r_i_j − ∑k∈K ∑(j,i)∈A y_k_r_j_i = 0 ∀r ∈ R, ∀i ∈ T
        for r, i in product(requests, transfer_stations):
            model.addConstr(
                gb.quicksum(
                    y[arc.src.index, arc.dst.index, k.index, r.index]
                    for k in vehicles
                    for arc in graph.arcs if arc.src == i
                )
                - gb.quicksum(
                    y[arc.src.index, arc.dst.index, k.index, r.index]
                    for k in vehicles
                    for arc in graph.arcs if arc.dst == i
                )
                == 0,
                '(6)'
            )

        # (8) y_k_r_i_j ≤ x_k_i_j ∀(i,j) ∈ A, ∀k ∈ K, ∀r ∈ R
        for arc, k, r in product(graph.arcs, vehicles, requests):
            model.addConstr(
                y[arc.src.index, arc.dst.index, k.index, r.index]
                <= x[arc.src.index, arc.dst.index, k.index],
                '(8)'
            )

        # (9) ∑r∈R q_r y_k_r_i_j ≤ u_k x_k_i_j ∀(i,j) ∈ A, ∀k ∈ K
        for arc, k in product(graph.arcs, vehicles):
            model.addConstr(
                gb.quicksum(
                    r.load * y[arc.src.index, arc.dst.index, k.index, r.index]
                    for r in requests
                )
                <= k.capacity * x[arc.src.index, arc.dst.index, k.index],
                '(9)'
            )

        # (16) ∑(i,j)∈A y_k_r_i_j − ∑(j,i)∈A y_k_r_j_i = 0 ∀k ∈ K, ∀r ∈ R, ∀i ∈ N\{T ∪ {p(r),d(r)}}
        for k, r, i in product(vehicles, requests, graph.nodes - transfer_stations):
            if i == r.pickup or i == r.destination:
                continue
            model.addConstr(
                gb.quicksum(
                    y[arc.src.index, arc.dst.index, k.index, r.index]
                    for arc in graph.arcs if arc.src == i
                )
                - gb.quicksum(
                    y[arc.src.index, arc.dst.index, k.index, r.index]
                    for arc in graph.arcs if arc.dst == i
                )
                == 0,
                '(16)'
            )

        # (21) ∑(j,t)∈A y_k1_r_j_t + ∑(t,j)∈A y_k2_r_t_j ≤ s_k1_k2_t_r + 1 ∀r ∈ R, ∀t ∈ T , ∀k1 , k2 ∈ K, k1 != k2
        for r, t, k1, k2 in product(requests, transfer_stations, vehicles, vehicles):
            if k1 == k2:
                continue
            model.addConstr(
                gb.quicksum(
                    y[arc.src.index, arc.dst.index, k1.index, r.index]
                    for arc in graph.arcs if arc.dst == t
                )
                + gb.quicksum(
                    y[arc.src.index, arc.dst.index, k2.index, r.index]
                    for arc in graph.arcs if arc.src == t
                )
                <= s[t.index, r.index, k1.index, k2.index] + 1,
                '(21)'
            )

        # (25) ∑(i,j)∈A x_k_i_j = 1 ∀k ∈ K, i = o(k)
        for k in vehicles:
            model.addConstr(
                gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.src == k.origin
                )
                == 1,
                '(25)'
            )

        # (27) ∑(i,j)∈A x_k_i_j − ∑(j,i)∈A x_k_j_i = 0 ∀k ∈ K, ∀i ∈ P ∪ D ∪ T
        for k, i in product(vehicles, graph.nodes):
            if i.type not in [NodeType.PICKUP, NodeType.DELIVERY, NodeType.TRANSFER_STATION]:
                continue
            model.addConstr(
                gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.src == i
                )
                - gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.dst == i
                )
                == 0,
                '(27)'
            )

        # (40) ∑(j,i)∈A x_k_j_i = 0 ∀k ∈ K, i = o(k)
        for k in vehicles:
            model.addConstr(
                gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.dst == k.origin
                )
                == 0,
                '(40)'
            )

        # (41) ∑(i,j)∈A x_k_i_j = 0 ∀k ∈ K, ∀i ∈ O ∪ O′, i != o(k)
        for k, i in product(vehicles, depot_nodes):
            if i == k.origin:
                continue
            model.addConstr(
                gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.src == i
                )
                == 0,
                '(41)'
            )

        # (42) ∑(j,i)∈A x_k_j_i = 1 ∀k ∈ K, i = o'(k)
        for k in vehicles:
            model.addConstr(
                gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.dst == k.dest
                )
                == 1,
                '(42)'
            )

        # (43) ∑(i,j)∈A x_k_j_i = 0 ∀k ∈ K, i = o'(k)
        for k in vehicles:
            model.addConstr(
                gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.src == k.dest
                )
                == 0,
                '(43)'
            )

        # (44) ∑(i,j)∈A x_k_i_j ≤ 1 ∀k ∈ K, ∀i ∈ T
        for k, i in product(vehicles, transfer_stations):
            model.addConstr(
                gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.src == i
                )
                <= 1,
                '(44)'
            )

        # (45) ∑(i,j)∈A ∑k∈K x_k_i_j = 1 ∀i ∈ P ∪ D
        for i in graph.nodes - transfer_stations - depot_nodes:
            model.addConstr(
                gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.src == i
                    for k in vehicles
                )
                == 1,
                '(45)'
            )

        # (46) ∑(i,j)∈A ∑k∈K y_k_r_i_j = 0 ∀r ∈ R, j = p(r)
        for r in requests:
            model.addConstr(
                gb.quicksum(
                    y[arc.src.index, arc.dst.index, k.index, r.index]
                    for arc in graph.arcs if arc.dst == r.pickup
                    for k in vehicles
                )
                == 0,
                '(46)'
            )

        # (47) ∑(i,j)∈A y_k_r_i_j = 0 ∀r ∈ R, ∀k ∈ K, ∀i ∈ O ∪ O′, i != o(k), i != o′ (k)
        for r, k, i in product(requests, vehicles, depot_nodes):
            if i == k.origin or i == k.dest:
                continue
            model.addConstr(
                gb.quicksum(
                    y[arc.src.index, arc.dst.index, k.index, r.index]
                    for arc in graph.arcs if arc.src == i
                )
                == 0,
                '(47)'
            )

        # (48) a_k1_t − b_k2_t ≤ M(1 − s_k1_k2_t_r) ∀r ∈ R, t ∈ T, k1,k2 ∈ K, k1 != k2
        for r, t, k1, k2 in product(requests, transfer_stations, vehicles, vehicles):
            if k1 == k2:
                continue
            M = t.latest_time - t.earliest_time
            model.addConstr(
                a[t.index, k1.index] - b[t.index, k2.index]
                <= M * (1 - s[t.index, r.index, k1.index, k2.index])
            )

        # (49) b_k_i + τ_k_i_j − a_k_j ≤ M(1 − x_k_i_j) ∀(i,j) ∈ A, ∀k ∈ K
        for arc, k in product(graph.arcs, vehicles):
            M = max(0, arc.src.latest_time + arc.cost - arc.dst.earliest_time)
            model.addConstr(
                b[arc.src.index, k.index] + arc.cost - a[arc.dst.index, k.index]
                <= M * (1 - x[arc.src.index, arc.dst.index, k.index])
            )

        # (50) a_k_i ≥ Ei , bk_i ≤ Li ∀i ∈ N, ∀k ∈ K
        for i, k in product(graph.nodes, vehicles):
            model.addConstr(
                a[i.index, k.index] >= i.earliest_time
            )
            model.addConstr(
                b[i.index, k.index] <= i.latest_time
            )

        # (51) a_k_i ≤ b_k_i ∀i ∈ N, ∀k ∈ K
        for i, k in product(graph.nodes, vehicles):
            model.addConstr(
                a[i.index, k.index] <= b[i.index, k.index]
            )

        self.model = model
