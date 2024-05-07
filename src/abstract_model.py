from abc import ABC


class AbstractModel(ABC):

    def __init__(self):
        self.model = None

    def optimize(self):
        self.model.optimize()

    def get_result(self) -> tuple[str, float, float, float]:
        return self.get_status(), round(self.model.ObjVal, 7), self.model.MIPGap, self.model.Runtime
    
    def get_status(self):
        code_to_keyword_map = {
            1: 'LOADED',
            2: 'OPTIMAL',
            3: 'INFEASIBLE',
            4: 'INF_OR_UNBD',
            5: 'UNBOUNDED',
            6: 'CUTOFF',
            7: 'ITERATION_LIMIT',
            8: 'NODE_LIMIT',
            9: 'TIME_LIMIT',
            10: 'SOLUTION_LIMIT',
            11: 'INTERRUPTED',
            12: 'NUMERIC',
            13: 'SUBOPTIMAL',
            14: 'INPROGRESS',
            15: 'USER_OBJ_LIMIT',
            16: 'WORK_LIMIT',
            17: 'MEM_LIMIT'
        }
        return code_to_keyword_map[self.model.Status]
