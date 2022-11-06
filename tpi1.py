from tree_search import *
from cidades import *
from blocksworld import *
# Made by Inês Moreira nmec: 100084
# Discussed with: 
# Bárbara Moreira nmec: 104056
# Ricardo Machado nmec: 102737

def func_branching(connections,coordinates):
    all = 0
    for a in coordinates:
        count = 0
        for b, c, d in connections: 
            if a == b or a == c or a == d: 
                count += 1
        all += count 
    
    coord = len (coordinates)
    average = all/coord-1 

    return average 

class MyCities(Cidades):
    def __init__(self,connections,coordinates):
        super().__init__(connections,coordinates)
        self.branching = func_branching(connections, coordinates)
       

class MySTRIPS(STRIPS):
    def __init__(self,optimize=False):
        super().__init__(optimize)

    def simulate_plan(self,state,plan):
        for action in plan:
            state= self.result(state, action)
            return state


# Nós de uma árvore de pesquisa 
class MyNode(SearchNode):
    def __init__(self, state, parent, depth, cost, heuristic):
        super().__init__(state,parent)
        self.depth= depth
        self.cost= cost
        self.heuristic= heuristic 


class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',optimize=0,keep=0.25): 
        super().__init__(problem,strategy)
        self.optimize = optimize
        root = MyNode(problem.initial, None, 0, 0, 0)
        self.all_nodes = [root]
        self.open_nodes = [0]
        self.keep = keep

        if optimize == 0:
            root = MyNode(problem.initial, None, 0, problem.domain.heuristic(problem.initial, problem.goal), 0)
        elif optimize == 1:
            root = (problem.initial, None, 0, problem.domain.heuristic(problem.initial, problem.goal), 0)
        else:
            root = (problem[1], None, 0, problem[0][3](problem[1], problem[2]), 0)
    

    def astar_add_to_open(self,lnewnodes):
        if self.strategy == 'breadth':
            self.open_nodes.extend(lnewnodes)

        elif self.strategy == 'depth': 
            self.open_nodes[:0] = lnewnodes 

        elif self.strategy == 'a*':
            self.open_nodes.extend(lnewnodes)
            self.open_nodes.sort(key= lambda node: node.heuristic + node.cost)

    # remove a fraction of open (terminal) nodes
    # with lowest evaluation function
    # (used in Incrementally Bounded A*)
    def forget_worst_terminals(self):
        #IMPLEMENT HERE
        pass

    # procurar a solucao
    def search2(self):
        while self.open_nodes != []:
            nodeID = self.open_nodes.pop(0)
            node = self.all_nodes[nodeID]

            if self.optimize == 0:
                if self.problem.goal_test(node.state):
                    self.solution = node
                    self.terminals = len(self.open_nodes)+1
                    return self.get_path(node)

            elif self.optimize == 1:
                if self.problem.goal_test(node[0]):
                    self.solution = node
                    self.terminals = len(self.open_nodes)+1
                    return self.get_p(node)

            else:
                if self.problem[0][4](node[0],self.problem[2]):
                    self.solution = node
                    self.terminals = len(self.open_nodes)+1
                    return self.get_path2(node)

            lnewnodes = []
            self.non_terminals += 1

            if self.optimize == 0:
                for a in self.problem.domain.actions(node.state):
                    newstate = self.problem.domain.result(node.state,a)
                    # store
                    if newstate not in self.get_path(node):
                        newnode = MyNode(newstate,nodeID, node.depth + 1, node.cost + self.problem.domain.cost(node.state, a), self.problem.domain.heuristic(newstate, self.problem.goal))
                        lnewnodes.append(len(self.all_nodes))
                        self.all_nodes.append(newnode)
        
            elif self.optimize == 1:
                for a in self.problem.domain.actions(node[0]):
                    newstate = self.problem.domain.result(node[0], a)
                    if newstate not in self.get_path2(node):
                        if self.optimize:
                            newnode = (newstate, nodeID, node[4] + 1, node[2] + self.problem.domain.cost(node[0], a), self.problem.domain.heuristic(newstate, self.problem.goal)) 
                            lnewnodes.append(len(self.all_nodes))
                            self.all_nodes.append(newnode)

            elif self.optimize == 2: 
                 for a in self.problem[0][0](node[0]):
                        newstate = self.problem[0][1](node[0],a)
                        if newstate not in self.get_path2(node):
                            if self.optimize:
                                newnode = (newstate, nodeID, node[4] + 1, node[2] + self.problem[0][2](node[0], a), self.problem[0][3](newstate, self.problem[2])) 
                                lnewnodes.append(len(self.all_nodes))
                                self.all_nodes.append(newnode)

            self.add_to_open(lnewnodes)
        return None    

    def get_p(self,node):
        if node[1] == None:
            return [node[0]]
        path = self.get_p(self.all_nodes[node[1]])
        path += [node[0]]

        return(path)




