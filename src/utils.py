from itertools import product

from src.request import Request
from src.vehicle import Vehicle
from src.graph import NodeType
from src.graph import Node
from src.graph import Arc
from src.graph import Graph

from math import floor, ceil
from pathlib import Path

import pandas as pd


_N_INSTANCES_PER_CONFIG = 10
PDPT_PAPER_RESULTS_PATH = '../data/Results/Results-PDPT.txt'
PDPTWT_PAPER_RESULTS_PATH = '../data/Results/Results-PDPTWT.txt'
PDPT_VEHICLES_PAPER_RESULTS_PATH = '../data/Results/Results-PDPT-vehicle.txt'


def _get_euclidean_distance(node1: Node, node2: Node) -> float:
    """
    Returns the Euclidean distance between two nodes
    """
    x1, y1 = node1.coordinates
    x2, y2 = node2.coordinates
    return ((x2 - x1)**2 + (y2 - y1)**2) ** 0.5


def get_instance_data(filepath: Path, *, sampaio: bool = False) -> tuple[Graph, set[Vehicle], set[Request]]:
    """
    Returns the data required to initialize a model
    :param filepath: Path of the .txt file containing the instance's parameters
    :param sampaio: Set to true to obtain the instance data for the Sampaio model (for each vehicle, the origin depot
    coincides with the destination depot)
    :return: a tuple containing the graph, the set of vehicles and the set of requests
    """
    requests = set()
    nodes = set()
    vehicles = set()
    arcs = set()

    with open(filepath, 'r') as f:
        params_names = f.readline().replace('\n', '').split('\t')
        params_values = f.readline().replace('\n', '').split('\t')
        params = dict(zip(params_names, params_values))
        f.readline()  # skip blank line
        instance_data = f.readline().replace('\n', '').split('\t')
        n_nodes = 2 * int(params['nr']) + 2 * int(params['nv']) + int(params['nt'])

        pickup_nodes = dict()
        origin_depot_nodes = dict()
        i_requests = 0
        i_vehicles = 0

        for i in range(n_nodes):
            line_data = f.readline().replace('\n', '').split('\t')
            node_data = dict(zip(instance_data, line_data))

            node_type = NodeType(node_data['node'][0])
            coords = (int(node_data['x']), int(node_data['y']))
            earliest_time = int(node_data['a'])
            latest_time = int(node_data['b'])

            if sampaio and node_type == NodeType.DESTINATION_DEPOT:
                continue

            node = Node(i, node_type, coords, earliest_time, latest_time)
            nodes.add(node)

            if node_type == NodeType.PICKUP:
                pickup_nodes[node_data['node']] = node
            if node_type == NodeType.DELIVERY:
                pickup_node = pickup_nodes['p' + node_data['node'][1:]]
                load = abs(int(node_data['load']))
                requests.add(Request(i_requests, pickup_node, node, load))
                i_requests += 1

            if node_type == NodeType.ORIGIN_DEPOT:
                if sampaio:
                    vehicles.add(Vehicle(i_vehicles, node, node, 100))
                    i_vehicles += 1
                else:
                    origin_depot_nodes[node_data['node']] = node
            if node_type == NodeType.DESTINATION_DEPOT:
                origin_node = origin_depot_nodes['o' + node_data['node'][1:]]
                vehicles.add(Vehicle(i_vehicles, origin_node, node, 100))
                i_vehicles += 1

    for n1, n2 in product(nodes, nodes):
        if n1 == n2:
            continue
        cost = _get_euclidean_distance(n1, n2)
        arcs.add(Arc(n1, n2, cost))

    return Graph(nodes, arcs), vehicles, requests


def _pick_median_instances(df: pd.DataFrame, k: int, skip: list[str] = None) -> list[str]:
    """
    Pick k instances around the median value w.r.t. 'Time' column
    :param df: the pandas DataFrame from which to pick instances
    :param k: the number of instances to pick
    :param skip: a list of strings that identify the configurations to skip
    :return: a list of instances names
    """
    median_instances = list()

    df = df.sort_values(by='Time').reset_index(drop=True)

    if skip is None:
        skip = []

    for j in range(-ceil(k / 2), floor(k / 2)):
        t = _N_INSTANCES_PER_CONFIG / 2 + j

        for stopword in skip:
            if stopword in df['Instance'].loc[t]:
                return []

        median_instances.append(df['Instance'].loc[t])

    return median_instances


def pick_pdpt_instances(n: int, k: int, model: str, skip: list[str] = None) -> list[str]:
    """
    Pick k instances for each of the first n configurations around the median value w.r.t. 'Time'
    :param n: number of parameters configurations to test
    :param k:number of instances per configuration
    :param model: reference model for the results: either 'Rais' or 'Lyu'
    :param skip: a list of strings that identify configurations to skip
    :return: list of instances names
    """
    n_rows = _N_INSTANCES_PER_CONFIG * 2 * n
    k = min(_N_INSTANCES_PER_CONFIG, k)

    df = pd.read_csv(PDPT_PAPER_RESULTS_PATH, sep='\t', nrows=n_rows, comment='#')
    df = df[df['Model'] == model]

    pdpt_instances = list()

    for i in range(1, n + 1):
        sub_df = df.head(_N_INSTANCES_PER_CONFIG * i).tail(_N_INSTANCES_PER_CONFIG)
        pdpt_instances.extend(_pick_median_instances(sub_df, k, skip))

    return pdpt_instances


def pick_pdpt_vehicles_instances(n: int, k: int, model: str, skip: list[str] = None) -> list[str]:
    """
    Pick k instances for each of the first n configurations around the median value w.r.t. 'Time'
    :param n: number of parameters configurations to test
    :param k:number of instances per configuration
    :param model: reference model for the results: either 'Rais' or 'Lyu'
    :param skip: a list of strings that identify configurations to skip
    :return: list of instances names
    """
    n_rows = _N_INSTANCES_PER_CONFIG * 2 * n
    k = min(_N_INSTANCES_PER_CONFIG, k)

    df = pd.read_csv(PDPT_VEHICLES_PAPER_RESULTS_PATH, sep='\t', nrows=n_rows, comment='#')
    df = df[df['Model'] == model]

    pdpt_v_instances = list()

    for i in range(1, n + 1):
        sub_df = df.head(_N_INSTANCES_PER_CONFIG * i).tail(_N_INSTANCES_PER_CONFIG)
        pdpt_v_instances.extend(_pick_median_instances(sub_df, k, skip))

    return pdpt_v_instances


def pick_pdptwt_instances(k: int, model: str, skip: list[str] = None) -> list[str]:
    """
    Pick k instances for each configuration around the median value w.r.t. 'Time'
    :param k:number of instances per configuration
    :param model: reference model for the results: either 'Sampaio' or 'Lyu'
    :param skip: a list of strings that identify configurations to skip
    :return: list of instances names
    """
    k = min(_N_INSTANCES_PER_CONFIG, k)

    df = pd.read_csv(PDPTWT_PAPER_RESULTS_PATH, sep='\t')
    df = df[df['Model'] == model]

    pdptw_instances = list()

    for i in range(1, len(df) // 10 + 1):
        sub_df = df.head(_N_INSTANCES_PER_CONFIG * i).tail(_N_INSTANCES_PER_CONFIG).reset_index(drop=True)
        pdptw_instances.extend(_pick_median_instances(sub_df, k, skip))

    return pdptw_instances


def log_result(problem: str, model: str, instance_name: str, result: tuple[str, float, float, float]) -> None:
    """
    Log the result of Gurobi's model optimization
    :param problem: problem solved: either 'PDPT' or 'PDPTW'
    :param model: model used: either 'Rais', 'Lyu' or 'Sampaio'
    :param instance_name: name of solved instance, e.g. PDPT-R5-K2-T1-Q100-0.txt
    :param result:a tuple of status, objective, gap and time
    :return: nothing
    """
    problem = problem.upper()

    results_file = Path(f"../results/{problem}.csv")
    results_file.parent.mkdir(parents=True, exist_ok=True)

    if not results_file.exists():
        with open(results_file, 'w') as f:
            f.write('Instance,Status,Objective,Gap,Time,Model\n')

    with open(f"../results/{problem}.csv", 'a') as f:
        f.write(f"{instance_name},{result[0]},{result[1]},{result[2]},{result[3]},{model}\n")
