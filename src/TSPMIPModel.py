import mip
from TSPWDData import TSPWDData
from TSPWDSolution import TSPWDSolution


class TSPMIPModel:
    def __init__(self, instance: TSPWDData):
        self.instance = instance
        self.__algorithm = "MIP"
        self.model = mip.Model(name="TSP", sense=mip.MINIMIZE)
        pass
        
        #Variables de décision
        # x[i][j]   = 1 si on passe sur l'arc (i,j), avec i=0,...,n et j=0,...,n
        #           = 0 sinon

        self.x = [
                    [
                        self.model.add_var(name=f"x_{i}_{j}", var_type=mip.BINARY) for j in range(instance.graph.number_of_nodes())
                    ]
                    for i in range(instance.graph.number_of_nodes())
                ]

        # y[i]  = 1 si le noeud i a ete visite, avec i=0,...,n
        #       = 0 sinon

        self.y = [
            self.model.add_var(name=f"y_{i}", var_type=mip.BINARY) for i in range(instance.graph.number_of_nodes())
        ]

        #Fonction objectif

        self.model.objective = mip.xsum(
            mip.xsum(instance.time_matrix[i][j] * self.x[i][j] for i in range(len(instance.demands_nodes)) ) for j in range(len(instance.demands_nodes))
        )

        #Les contraintes
        
        #Cstr1
        for j in instance.demands_nodes :
            self.model += (
                mip.xsum(self.x[i][j] for i in range(instance.graph.number_of_nodes())) == self.y[j]
            )

        #Cstr2
        for j in instance.demands_nodes :
            self.model += (
                self.y[j] == 1
            )

        #Cstr 3
        for j in range(instance.graph.number_of_nodes()) :
            self.model += (
                mip.xsum(self.x[i][j] for i in range(instance.graph.number_of_nodes())) == 1
            )

        #Cstr4
        for j in range(instance.graph.number_of_nodes()) :
            self.model += (
                mip.xsum(self.x[j][i] for i in range(instance.graph.number_of_nodes())) == 1
            )

    def _create_solution(self):
        x = [
                [ 
                    self.x[i][j].x for j in range(len(self.instance.nodes))
                ]
                for i in range(len(self.instance.nodes))
            ]


        y = [
                self.y[i].x for i in range(len(self.instance.nodes))
            ]

        return x, y

    def solve(
        self,
        solver_name: str = "GRB",
        verbose: bool = True,
        time_limit: int = 600,
        max_gap: float = 0.0001,
        nb_threads: int = -1,
    ) -> TSPWDSolution:
        self.model.solver_name = solver_name
        self.model.verbose = int(verbose)
        self.model.max_seconds = time_limit
        self.model.max_mip_gap = max_gap
        self.model.threads = nb_threads

        # Solve model
        _status = self.model.optimize()

        # Create solution
        _x, _y = self._create_solution()

        solution = [self.instance.demands_nodes[0]]
        for i in range (len(solution)):
            for j in range(len(self.instance.nodes)) :
                if _x[solution[i]][j] == 1 and _y[j] == 1 :
                    solution.append(j)

        #_runtime = self.model.search_progress_log.log[-1][0]
        _runtime = 4

        # Get solution
        if _status == mip.OptimizationStatus.OPTIMAL:
            print(
                f"Optimal Result: runtime={_runtime:.2f}sec; objective={int(self.model.objective_value)}; gap={self.model.gap:.4f}%"
            )
            return  TSPWDSolution(
                self.instance,
                self.__algorithm,
                int(self.model.objective_value),
                solution,
            )
        elif _status == mip.OptimizationStatus.FEASIBLE:
            print(
                f"Result: runtime={_runtime:.2f}sec; objective={int(self.model.objective_value)}; gap={100*self.model.gap:.4f}%"
            )
            return  TSPWDSolution(
                self.instance,
                self.__algorithm,
                int(self.model.objective_value),
                _x,
                _y
            )
        else:
            print(f"No solution found in {time_limit} seconds!")

    def __str__(self):
        return f" TSPWDMIPModel(instance={self.instance})"

    def __repr__(self):
        return self.__str__()