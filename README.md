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
  - `src/models`: Gurobi MILP models;
  - `src/network`: network elements of the PDPTW-T;
  - `src/computations.ipynb`: run bulk computations over multiple instances;
  - `src/quicktest.py`: quickly test the models on a given instance.

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