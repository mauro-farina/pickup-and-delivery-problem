# Pickup and Delivery Problem with Transshipments

- Project for the Mathematical Optimization course at University of Trieste;
- Implementation of the three MILP models discussed in
[The pickup and delivery problem with transshipments: Critical review of two existing models and a new formulation](https://www.sciencedirect.com/science/article/pii/S0377221722004556).

## Structure
- `data` contains the files obtained from the [Mendeley Data](https://data.mendeley.com/datasets/w925jygjct/4)
repository managed by the authors of the paper. Minor changes have been made to the files in `data/Results` to solve 
inconsistent spelling;
- `results` contains the results obtained by the Gurobi solver with a time limit of 1 hour;
- `figures` contains plots made to analyze the results;
- `src` is the python package containing all the code. In particular:
  - `rais.py`, `sampaio.py` and `lyu.py`: Gurobi MILP models;
  - `graph.py`, `request.py` and `vehicle.py`: network elements of the PDP-T;
  - `computations.ipynb`: run bulk computations over multiple instances;
  - `run.py`: quickly test the performance of a model on a given instance from CLI.

## Python
- The repository has been tested with Python 3.9 and 3.12

## Setup
1. Close repository
    ```
    git clone https://github.com/mauro-farina/pickup-and-delivery-problem.git
    ```
2. Move to cloned directory
    ```
    cd pickup-and-delivery-problem
    ```
3. Install dependencies
    ```
   pip install -r requirements.txt
   ```
4. Launch `run.py` to solve a given instance with a certain model; 
usage: `python src/run.py [instance_name] [model_name]`
where *instance_name* is the name of a file containing the instance data
and *model_name* is one of Rais, Sampaio or Lyu. For example:
    ```
   python src/run.py PDPT-R5-K2-T1-Q100-5.txt Rais
   ```
