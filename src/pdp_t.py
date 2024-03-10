import gurobipy as gb

from itertools import product

from src.graph import Graph, NodeType
from src.request import Request
from src.vehicle import Vehicle


class PDP_T:

    def __init__(self, graph: Graph, vehicles: set[Vehicle], requests: set[Request]):
        pdp_t = gb.Model('pdp_t')
        pdp_t.modelSense = gb.GRB.MINIMIZE

        pdp_t.setParam('OutputFlag', 0)

        transfer_stations = {t for t in graph.nodes if t.type is NodeType.TRANSFER_STATION}
        non_depot_nodes = {n for n in graph.nodes
                           if n.type is not NodeType.ORIGIN_DEPOT and n.type is not NodeType.DESTINATION_DEPOT}

        # x_k_i_j = 1 if vehicle k travels through arc (i,j)
        x = pdp_t.addVars(
            [(arc.src.index, arc.dst.index, k.index)
             for arc in graph.arcs
             for k in vehicles],
            vtype=gb.GRB.BINARY
        )

        # y_r_k_i_j = 1 if request r is transported by vehicle k through arc (i,j)
        y = pdp_t.addVars(
            [(arc.src.index, arc.dst.index, k.index, r.index)
             for arc in graph.arcs
             for k in vehicles
             for r in requests],
            vtype=gb.GRB.BINARY
        )

        # z_k_i_j = 1 if node i precedes node j for vehicle k
        z = pdp_t.addVars(
            [(i.index, j.index, k.index)
             for i in graph.nodes
             for j in graph.nodes if j != i
             for k in vehicles],
            vtype=gb.GRB.BINARY
        )

        pdp_t.setObjective(
            gb.quicksum(
                arc.cost * x[arc.src.index, arc.dst.index, k.index] * k.travel_unit_cost
                for arc in graph.arcs
                for k in vehicles)
        )

        # (1) ∑(i,j)∈A x_k_i_j ≤ 1 ∀k ∈ K, i = o(k)
        for k in vehicles:
            pdp_t.addConstr(
                gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.src == k.origin
                )
                <= 1,
                '(1)'
            )

        # (2) ∑(i,j)∈A x_k_i_j = ∑(j,l)∈A x_k_j_l ∀k ∈ K, i = o(k), l = o′ (k )
        for k in vehicles:
            pdp_t.addConstr(
                gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.src == k.origin
                )
                == gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.dst == k.dest
                ),
                '(2)'
            )

        # (3) ∑(i,j)∈A x_k_i_j − ∑(j,i)∈A x_k_i_j = 0 ∀k ∈ K, ∀i ∈ N\{o(k), o′(k)}
        for k, i in product(vehicles, non_depot_nodes):
            pdp_t.addConstr(
                #  ∑(i,j)∈A x_k_i_j
                gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.src == i
                )
                #  ∑(j,i)∈A x_k_i_j
                - gb.quicksum(
                    x[arc.src.index, arc.dst.index, k.index]
                    for arc in graph.arcs if arc.dst == i
                )
                == 0,
                '(3)'
            )

        # (4) ∑∈K ∑(i,j)∈A y_k_r_i_j = 1 ∀r ∈ R, i = p(r)
        for r in requests:
            pdp_t.addConstr(
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
            pdp_t.addConstr(
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
            pdp_t.addConstr(
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

        # (7) ∑(i,j)∈A y_k_r_i_j − ∑(j,i)∈A y_k_r_j_i = 0 ∀k ∈ K, ∀r ∈ R, ∀i ∈ N\T
        for k, r, i in product(vehicles, requests, graph.nodes - transfer_stations):
            pdp_t.addConstr(
                gb.quicksum(
                    y[arc.src.index, arc.dst.index, k.index, r.index]
                    for arc in graph.arcs if arc.src == i
                )
                - gb.quicksum(
                    y[arc.src.index, arc.dst.index, k.index, r.index]
                    for arc in graph.arcs if arc.dst == i
                )
                == 0,
                '(7)'
            )

        # (8) y_k_r_i_j ≤ x_k_i_j ∀(i,j) ∈ A, ∀k ∈ K, ∀r ∈ R
        for arc, k, r in product(graph.arcs, vehicles, requests):
            pdp_t.addConstr(
                y[arc.src.index, arc.dst.index, k.index, r.index]
                <= x[arc.src.index, arc.dst.index, k.index],
                '(8)'
            )

        # (9) ∑r∈R q_r y_k_r_i_j ≤ u_k x_k_i_j ∀(i,j) ∈ A, ∀k ∈ K
        for arc, k in product(graph.arcs, vehicles):
            pdp_t.addConstr(
                gb.quicksum(
                    r.load * y[arc.src.index, arc.dst.index, k.index, r.index]
                    for r in requests
                )
                <= k.capacity * x[arc.src.index, arc.dst.index, k.index],
                '(9)'
            ),

        # (10) x_k_i_j ≤ z_k_i_j ∀i,j ∈ N, ∀k ∈ K, i != o(k), j != o′(k)
        for i, j, k in product(graph.nodes, graph.nodes, vehicles):
            if i == j or i == k.origin or j == k.dest:
                continue
            pdp_t.addConstr(
                x[i.index, j.index, k.index] <= z[i.index, j.index, k.index],
                '(10)'
            )

        # (11) z_k_i_j + z_k_j_i = 1 ∀i,j ∈ N, ∀k ∈ K, i != o(k), j != o′(k)
        for i, j, k in product(graph.nodes, graph.nodes, vehicles):
            if i == j or i == k.origin or j == k.dest:
                continue
            pdp_t.addConstr(
                z[i.index, j.index, k.index]
                + z[j.index, i.index, k.index]
                == 1,
                '(11)'
            )

        # makes model infeasible
        # (12) z_k_i_j + z_k_j_l + z_k_l_i ≤ 2 ∀i,j,l ∈ N, ∀k ∈ K, i, j != o(k), l != o′(k)
        for i, j, l, k in product(graph.nodes, graph.nodes, graph.nodes, vehicles):
            if i == j or i == l or j == l:
                continue
            if i == k.origin or j == k.origin or l == k.dest:
                continue
            pdp_t.addConstr(
                z[i.index, j.index, k.index]
                + z[j.index, l.index, k.index]
                + z[l.index, i.index, k.index]
                <= 2,
                '(12)'
            )

        self.pdp_t = pdp_t

    def optimize(self):
        self.pdp_t.optimize()

    def print_result(self):
        if self.pdp_t.status == gb.GRB.OPTIMAL:
            print(f"Optimal solution found!")
            print(f"Objective value: {self.pdp_t.objVal}")
        else:
            print("No solution found")
