## Agents and Problem-Solving:

**Question 1:** Definition of agent, agent function and agent program.

An agent is defined as a entity that perceives the environment through sensors and acts on the environment through actuators.

The agent function is defined as  $f : P^* \to A$ where $P^*$ is the perceptions history and $A$ is the set of actions. An agent is completely specified by the agent function.

The agent program is defined as the program that is used to implement the agent function. The input is the current perception and the output is the action to be execute. The agent program runs on a PC.

**Question 2:** What is a rational agent?

A rational agent is an agent that acts to achieve the best outcome or the best expected outcome where there is uncertainty.

**Question 3:** What is an agent?

See **Question 1**.

**Question 4:** What is the main difference between agent function and agent program?

The main difference is that the agent function receives as input the entire perceptions history, instead the agent program receives only the current perception.

**Question 5:** What are the five components that define formally a search problem? Describe these components.

The main five components that define a search problem are:

1. States: a state space of all possible states. We need to define an initial state ad one or more goal states.
2. Actions: given a state $s$, the possible actions are the ones returned by Action($s$).
3. Transition model: given a state $s$ and an action $a \in$ Action(), the function call Result($s$, $a$) returns the new state that we reach after performing $a$ on $s$. This new state is called successor of $s$.
4. Path cost: each transition is associated to a cost, the cost of a path is defined as the sum of all the cost from the initial state to the current state.
5. Solution: the solution is defined as a path that starts from the initial node and reaches a goal state. An optimal solution is a solution with minimum cost.

**Question 6:** Provide the formal definition of a solution in a search problem.

The solution is defined as a path that starts from the initial state and reaches a goal state.

**Question 7:** In a problem-solving agent a problem can be defined formally by five components. Please, indicate these components and provide the definition of goal test.

See **Question 5** for the 5 components.

A goal test is a function that returns true if the state passed as argument is a goal state.

**Question 8:** Provide the formal definition of problem, solutions, and optimal solution in a search problem.

See **Question 5**.

**Question 9:** What is the transition model?

See **Question 5**.

**Question 10:** Describe the simple problem-solving agent providing both pseudocode and an informal description.

```
Problem-Solving-Agent(percept):
	state = update-state(state, percept)
	if seq is empty:
		goal = formulate-goal(state)
		problem = formulate-problem(goal)
		seq = search(problem)
	action = first(sequence)
	seq = rest(seq)
	return action
```

---

## Search Algorithms:

**Question 11:** Describe algorithm A* and the notion of admissible and consistent heuristic.

The A* algorithm is derived from the best-first search algorithm. The evaluation function used to choose which node should be expanded next is defined as:

$$
f(n) = g(n) + h(n).
$$
Where $g(n)$ gives the cost of the path from the initial node to node $n$ and $h(n)$ is an heuristic function that gives an estimate of the cost from node $n$ to a goal state. Therefore $f(n$) gives the estimated cost of a solution that passes through node $n$.

Admissible heuristic: $h(n)$ is admissible iff $h(n) \leq h^*(n)$ for each node $n$, where $h^*(n)$ is the true cost from node $n$ to the goal.

Consistent heuristic: $h(n)$ is consistent iff $h(n) \leq c(n, a, n') + h(n')$, where $n'$ is a successor of $n$ that we reach with action $a$. The relation must hold true for each successor $n'$ of $n$.

**Question 12:** Provide the definition of admissible heuristic.

See **Question 11**.

**Question 13:** Provide the definition of consistent heuristic.

See **Question 11**.

**Question 14:** What is the difference between A* and the greedy algorithm?

The main difference between the two algorithm is the evaluation function used. In A* we use $f(n) = g(n) + h(n)$, instead in the greedy search we use $f(n) = h(n)$.

Note also that A* is optimal (if $h(n)$ is admissible), instead greedy search it is not.

**Question 15:** Describe Breadth-first search and Depth-first search. What are the main differences between them?

- BFS: each parent node is visited before each of its children; is complete and optimal; $O(b^d)$ in time and space.
- DFS:  each childeren is visited before its parent; isn't neither complete or optimal; is $O(b^m)$ in time but linear in space.

**Question 16:** Describe how breadth-first search works.

See **Question 15**.

**Question 17:** Describe greedy best-first search.

See **Question 14**.

**Question 18:** Describe iterative deepening search.

The iterative deepening search is a depth-limited search in which the maximum depth is increased iteration by iteration.

**Question 19:** Describe the local beam search algorithm.

Local beam search algorithm uses $k$ current states instead of just one. At each iteration it generates the successors of each state: if any of them is a goal state, then the algorithm stops; otherwise the $k$ best successors are selected and the rule is repeated.

**Question 20:** Describe Local Beam search.

See **Question 20**.

**Question 21:** Describe the algorithm hill climbing. Explain for which reasons often hill climbing gets stuck.

Hill climbing selects the neighbor that has the highest value. The algorithm stops if no neighbor has an higher value w.r.t. the current state.

Since it only allows up-hill moves, the algorithm gets stuck on local maxima or plateaux.

**Question 22:** Describe hill climbing search providing both the pseudocode and an informal description.

Description: see **Question 21**.

```
Hill-Climbing-Search(initial):
	current = initial
	loop:
		n = get-highest-neighbor(current)
		if value(n) <= value(current):
			return current
		else:
			current = n
```

**Question 23:** Describe first-choice hill climbing.

The first-choice hill climbing algorithm is a hill climbing algorithm in which the neighbor of the current state that is selected is the first one that has an higher value.

---

## Constraint Satisfaction Problems (CSP):

**Question 24:** Definition of a constraint in a CSP.

CSP stands for Constraint Satisfaction Problem. A CSP is defined by
- A set o variables $X=\{X_1, \ldots, X_n\}$.
- A set of domains $D=\{D_1, \ldots, D_n\}$ where $X_i \in D_i\quad \forall i$.
- A set of constraints $C = \{c_i = (\text{scope}_i, \text{rel}_i), \quad \forall i\}$ where the scope is a subset of $X$ and the relation (rel) contains the allowed assignments of the variables in the scope.

A solution of a CSP must be both complete (all variables must be fixed to a value) and consistent  (meaning that the assignments satisfy all the constraints).

**Question 25:** Provide the formal definition of constraint satisfaction problem.

See **Question 24**.

**Question 26:** Provide the formal definition of a solution in a constraint satisfaction problem.

See **Question 24**.

**Question 27:** Describe the possible outcomes of the arc consistency algorithm applied to a CSP. Explain how these outcomes are related to the set of the solutions.

Arc consistency can be applied as a pre-processing step and/or within the search algorithm. Arc consistency is used to reduce the possible values that each variable can assume and to do this it exploits the binary constraints of our CSP. The new CSP produced share the same solutions of the starting one but it is easier to solve. Indeed arc consistency is a constraint propagation technique that it is used to speed up the searching process.

If we apply arc consistency to our CSP and each domains has only one value, then we have found a solution. If it exists a domain with 0 values, then the CSP doesn't admit a solution. If it exists a domain with more than one value, then nothing can be said, we need to apply an algorithm and try to solve the CSP.

**Question 28:** Consider the case where the arc consistency algorithm applied to a CSP terminates and some domains have multiple values. Is there guaranteed to be a solution? Justify the answer.

No, there is no guaranteed that a solution exists. Indeed if we have a problem with $X_1, X_2, X_3$ where $D_1=D_2=D_3 = \{1, 2\}$ and the constraints $X_1 \neq X_2$, $X_1 \neq X_3$ and $X_2\neq X_3$, then arc consistency doesn't detect failure but the problem doesn't admit a solution.

**Question 29:** Provide the formal definition of directed arc consistency.

A CSP is said to be Direct Arc Consistent (DAC) if we have a complete ordering of the variables $X_1,\ldots, X_n$ and if $X_i$ is arc consistent with respect to $X_j$ for all $j>i$ and for all $i = 1,\ldots, n$.

**Question 30:** What does it mean that a two-variable set {Xi, Xj} in a CSP is path-consistent with respect to a third variable Xm?

By definition, a two-variable set $\{X_i, X_j\}$ is path-consistent w.r.t. $X_m$ if:
- for each assignment of $\{X_i=a, X_j=b\}$ consistent to $\{X_i, X_j\}$
- there exist at least one value for $X_m$ such that
	- it is consistent to $\{X_i, X_m\}$ and
	- it is consistent to $\{X_m, X_j\}$.

**Question 31:** What is a global constraint? Provide an example.

A global constraint is a constraint that involves all the variables of the CSP. An example is the constraint All-Different: the values assigned to the variables must be different from each other.

**Question 32:** Describe the degree heuristic.

The degree heuristic is used to choose which variable we should consider next, during a step of the backtracking algorithm.

The degree heuristic chooses the (unassigned) variable that is involved in the most constraints (i.e. the most constraining variable).

**Question 33:** When is the constraint graph a tree?

Choose the root node. Then to verify that a constraint graph is a tree, we need to be sure that each node has exactly one parent (with exception to the root node).

Recall that, in order to solve a tree-structured CSP, we need also to verify that the Direct Arc Consistency property hods true. Otherwise we need to make the tree DAC.

**Question 34:** What is the tree width of a tree decomposition?

It is defined as the number $s-1$ where $s$ is the size of the largest sub-problem.

**Question 35:** Provide an example of a constraint satisfaction problem which is arc consistent but not consistent.

See **Question 28**.

**Question 36:** Define a map coloring problem with 5 variables where each variable can have values in D = red, blue, green. Apply backtracking with forward checking on this problem. Apply arc consistency.

...

**Question 37:** Constraints involving more than three variables are more expressive than constraints involving only two variables? Motivate your answer.

...

---

## Soft Constraints and Weighted Constraints:

**Question 38:** What is a soft constraint satisfaction problem (SCSP)? What is an optimal solution of an SCSP?

A soft-CSP is essentially a CSP in which we have soft-constraints. Soft-constraints are "normal" constraints that in addition have a specified preference value.

A soft-CSP is completely defined by its c-semiring which is defined as a tuple $\langle A, +, \times, \pmb 0, \pmb 1 \rangle$ where: ...(explain each component)...

The $+$ operator induces an ordering on the solution, it is used to state if a solution is better than another one. In general state if a solution is better than another one is an easy task, instead finding an optimal solution is not. To find an optimal solution we can use the Branc-and-Bound algorithm.

The definition of an optimal solution strictly depends on the c-semiring that specifies the soft-CSP that we're considering. For example if we consider a fuzzy-CSP, then the criteria specified by the c-semiring is to
*maximize the minimum preference*.

**Question 39:** Provide the formal definition of soft constraint.

A soft constraint is defined as $c = \langle con, f\rangle$, where $con \subseteq X$ and $f$ is a function $f: D_1 \times \ldots \times D_n \to A$ where $A$ is the set of admissible preferences values.

**Question 40:** What kind of preferences can model soft constraints?

Soft constraints can model quantitative and unconditional preferences.

---

## Stable Matching and Gale-Shapley Algorithm:

**Question 44:** What's a stable matching problem? What's a matching? What does it mean that a matching is stable?

In a stable matching problem we have two distinct sets of agent, each agent totally order all the agents that belong to the other set. The goal is to find a stable matching between the two groups of agents.

A matching is a one to one relation between two agents belonging to different sets. In the original formulation (where ties are no allowed and preferences lists cannot be incomplete), a matching is stable if there aren't any blocking pairs.

**Question 45:** Provide formal definition of blocking pair.

In the original formulation (see **Question 45**) we say that the pair $(m, w)$ is blocking iff $m$ and $w$ are not married by the prefer to be together rather than the partner they are currently with.

Formally, we have that $m$ is married with $w'$ and $m'$ is married with $w$, but $m$ prefers $w$ to $w'$ and $w$ prefers $m$ to $m'$.

**Question 46:** Describe Gale-Shapley algorithm.

GS algorithm is used to solve the stable marriage problem. It ties are not allowed and preferences cannot be incomplete, then it always finds a solution (since it always exists at least one stable marriage). Also the solution is optimal for men and pessimal for women, in other words, one set of agents is always preferred.


---

## Voting Theory:

**Question 48:** Describe how Plurality rule works.

Each agent chooses its most preferred candidates. The winner is the candidates that receives the most votes.

**Question 49:** Describe how Borda rule works.

Each agent totally rank all the candidates. For each candidate we can compute the score by counting, for each ranking list, how many candidates it dominates. The candidate with the highest score is the winner.

**Question 50:** Describe how Borda rule works. Is it possible to manipulate it?

From Arrow's theorem we know that if we have totally order preferences, than it is impossible to avoid non-dictatorship and non-manipulability. Therefore it is possible to manipulate Borda rule.

---

## Bayesian Networks:

**Question 51:** Describe the syntax and the semantics of a Bayesian network.

BN are network models used to reason under uncertainty. They use a graph notation that represents dependencies among random variables. They can represent full joint distribution in a very compact manner by using Conditional Probability Tables (CPTs).

Each random variable is expressed as a node. A direct edge that connects $A \to B$ means that $A$ directly influences $B$. Each node as an associated CPT that express the Conditional Probability Distribution $P(X_i \mid \text{Parents}(X_i))$.

In order to compute the full joint distribution, we can use the following formula:
$$
P(x_1,\ldots, x_n) = \prod_{i=1}^n P(X_i \mid \text{Parents}(X_i))
$$

**Question 52:** What is the syntax of a Bayesian network (BN)?

See **Question 51**.

**Question 53:** Describe the semantics of a Bayesian network (BN).

See **Question 51**.

**Question 54:** What is a conditional probability table in a Bayesian network?

In a BN each node is associated to a CPT. The CPT contains the information about the Conditional Probability Distribution $P(X_i \mid \text{Parents}(X_i))$.

---

## CP-nets (Conditional Preference Networks):

**Question 56:** What are the similarities and the differences between Bayesian networks and CP-nets?

Both BNs and CP-nets use graphs and both express relations using direct edges. The difference between the two is that, BNs are used to reason under uncertainty and to represent full joint distributions in a compact manner; instead CP-nets are used to model preferences.

BNs use Conditional Probability Tables, CP-nets use Conditional Preferences Tables.

**Question 57:** What kind of preferences can model CP-nets?

CP-nets can model qualitative and conditional preferences.

**Question 58:** What are the main differences between soft constraint formalism and CP-nets?

See **Question 56**.

**Question 59:** Describe the algorithm to find an optimal solution of an acyclic CP-net.

Steps:
1. Consider independent variables and assign them their most preferred value.
2. Consider dependent variables that directly depend on the assigned variables, and assign to them their most preferred value based on the value assigned to their parent.
3. Repeat from step (2) considering the remaining dependent variables.

**Question 60:** What is a conditional preference table in a CP-net?

CPT are used to express conditional preferences. 

For instance consider that we have a graph in which we have two variables, namely $A,B$ and the edge $A\to B$.

In this case $A$ is an independent variable and in its CPT we can find the preferences among its possible values. Instead $B$ is a dependent variable and in its CPT we can find the preferences among its possible values, that are based on the value assumed by $A$.

...(Example)...

---

## Lab Part

**Question 61:** Describe the components to design an agent based on a hierarchical architecture (i.e., 3 layers).

We have three layers: Top Layer, Mid Layer and Low Layer. Each handles information at different level of abstraction. Low Layer is the fastest one, Top Layer is the slowest.

In details:
1. Top Layer: planning tasks for the agents. The original plan can be re-adapted using the info provided my Mid Layer.
2. Mid Layer: determines agent's behaviors based on planned tasks (from Top Layer) and perceived context (from Low Layer).
3. Low Layer: implements low level commands using actuators and perceives through sensors.

**Question 62:** Consider a Roomba robot in the next figure that can vacuum and wash the floor. It is equipped with a camera to detect the obstacles and the dirt. It can automatically reach the charging base. Hypothesize to use a hierarchical architecture for designing the agent. Represent and explain the flow of the information among the three layers considering the three functionalities: vacuum, wash and charge. 

Architecture:
1. Top Layer: high level planning tasks to vacuum, wash and charge. The actions are provided as input to the Mid Layer and the original planning can be re-adapted based on feedback provided (e.g. failure or success). 
2. Mid Layer: has to determine specific agent's behaviors based on the planned tasks and perceived context (from Low Layer). Provides routines to reach targets, to wash and to vacuum.
3. Low Layer: implements low-level commands (from Mid Layer) using actuators (steering commands) and perceives through sensors (obstacles, dirt, battery level).

**Question 63:** Report and explain the pseudocode to implement the Gale-Shapley algorithm based on a graph.

We can create a complete bipartite graph where in one side we have all the nodes representing men an on the other hand the notes representing women. 

...(draw the graph)...

Then we can create a dictionary in which we store the pairs.

...(write pseudocode)...

In the end we can highlight the edges associated to the solution found by the algorithm in order to have a (useless) visual effect (that no one cares about).

**Question 64:** Consider the Roomba robot (Figure 1) that has to clean a flat composed of five main rooms (kitchen, dining room, bathroom, bedroom, living room) and two corridors. The flat planimetry is reported below. Draw a graph to represent the flat (i.e, topological map) and motivate your choices.

We can add 5 nodes that represent the 5 main rooms. We connect two nodes with an edge if we have direct access from one room to the other.

**Question 65:** Complete the following code to start implementing the graph representing the topological map of the flat in Python using the library networkx.

```python
import networkx as nx

G = nx.Graph()
G.________([‘Kitchen’, ‘DiningRoom’, ‘LivingRoom’, ‘Bedroom’, ’Bathroom’, ‘CorridorA’, ‘CorridorB’])

G.________(‘Kitchen’, ‘DiningRoom’)
```

We need to call:
- `G.add_nodes_from()` to add the nodes to the graph.
- `G.add_edge()` to add an edge between two nodes.

**Question 66:** Does it exist a path connecting the Kitchen with the Living Room? How is it possible to verify that via code?

Yes it exists. To see that we can just call the function `G.has_path()`.

**Question 67:** Explain a strategy based on A* to find the best path to go to the kitchen starting from the bedroom.

We need to specify the evaluation function $f(n) = g(n) + h(n)$. We can consider $g(n)$ the function that returns the distance between the initial room and room $n$; instead $h(n)$ returns the straight line distance between room $n$ and the goal room. 

```
A*(G, start, goal):
	f_start = h(start)
	frontier = [(start, f_start)]
	
	while frontier is not empty:
		current = frontier.pop()
		
		if current is goal:
			return solution
		
		for each neighbor of current:
			current g_scores += w(current, neighbor)
			
		add neigbors nodes if they're not in the frontier
		
		for each node in frontier:
			if g(node) + h(node) < f(n):
				update f_score for node
				
	return failure
```

**Question 68:** Report and explain the pseudocode to implement the Genetic algorithm.

Three main components:
1. Fitness function: tells how good is the current solution.
2. Mutation: operator that causes small change in a gene at random.
3. Cross-over:randomly selects a breakpoint and then combines two genes
4. Selection: randomly selects N parents from the initial population, then selects M of the N parents with the best fitness.

```
genetic-alg(population, fitness_func):
	repeat:
		init new_population as empty
		
		for i=1 to size(population):
			x = random-select(population, fitness_func)
			y = random-select(population, fitness_func)
			
			child = cross-over(x, y)
			mutate(child)
			
			new_population.add(child)
		
		population = new_population
		
	until some individual is fit enough

	return best invididual in population wrt fitness_func
```

**Question 69:** Explain how to solve the problem of the N-queens based on the Hill-climbing search.

Recall: no two queens can stay on the same row, column and diagonal.

We define:
- Goal: minimize the number of queens under attack. This is the metrics used to evaluate each state.
- Optimal sol: no queens under attack.

If the algorithm finds multiple optimal solution, then it chooses at random one of them.

**Question 70:** Report and explain the pseudocode to implement the backtracking algorithm.

```
backtracking(assignment, csp):

	if assignment is complete:
		return assignment

	var = select_variable(csp)
	ordered_values = var.get_ordered_values()
	
	for value in ordered_values:
	
		if value is consistent with csp:
			var = value
			assignment.add(var)
			
			result = backtracking(assignment, csp)
			
			if result != failure:
				return result
			
			assignment.remove(var)

	return failure
```


**Question 71:** Use CSP to assign equal values to equal letters, and different values to different letters, in a way that satisfies the following sum:

```
SEIS +
SEIS
------
DOZE
```

1. Explain the formulation of the problem with CSP with particular attention to the definition of the constraints.
2. Which functions of the library python constraint library are useful to model and solve the problem above?

Formulation of the CSP:
- Variables: $X = \{S, I, D, O, Z, E\}$.
- Constraints:
	1. Each variable must be between 0 and 9.
	2. Each variable must have a different value.
	3. $S \neq D \neq 0$.
	4. Constraint on the equation:

$$
2 \cdot (S \cdot 10^3 + E \cdot 10^2 + I \cdot 10 + S) = D\cdot 10^3 + O \cdot 10^2 + Z \cdot 10 + E
$$

The python library that one can use is `python constraints`.

```python
def solve():
	# Create a problem.
	problem = Problem()
	# Add variables: specify names and domains.
	problem.addVariables("seidoz", range(10))

	# Add the constraints.
	problem.addConstraint(
		lambda s, e, i, d, o, z: 
			2 * (s * 1000 + e * 100 + i * 10 + s) 
			== d * 1000 + o * 100 + z * 10 + e, "seidoz",
	)
	problem.addConstraint(lambda s: s != 0, "s")
	problem.addConstraint(lambda d: d != 0, "d")

	solutions = problem.getSolutions()
return solutions
```

**Question 72:** Suppose that the voters V1, V2, V3 and V4 express the preferences reported in Figure 3. Compute the corresponding score using the borda criterion.

...(just do the thing)...

**Question 73:** Describe how to implement a voting system based on plurality in Python to choose the winner based on the data of the following figure. 

...(just use some counters)...

**Question 74:** Considering the Bayesian Network reported in Figure, Fill the portion of the code below in order to implement it using `pgmpy`.

```python
diet_model = BayesianNetwork([
	______
])

cpd_overweight = TabularCPD(
	variable="BeOverweight",
	variable_card=2, 
	values=____
)

cpd_sport = TabularCPD(
	variable="PractiseSport",
	variable_card=2,
	values=_____,
	evidence=["BeOverweight"],
	evidence_card=[2],
)

cpd_diet = TabularCPD(
	variable="Diet",
	variable_card=2,
	values=____,
	evidence=["BeOverweight"],
	evidence_card=[2],
)

cpd_fit = TabularCPD(variable="BeFit",
	variable_card=2,
	values=___,
	evidence=["PractiseSport","Diet"],
	evidence_card=[2,2]
)
```

Solution:
```python
diet_model = BayesianNetwork([
	# Parent       -> Child
	("BeOverweight",  "PractiseSport"),
	("BeOverweight",  "Diet"),
	("PractiseSport", "BeFit"),
	("Diet",          "BeFit"),
])

cpd_overweight = TabularCPD(
	variable="BeOverweight",
	variable_card=2, 
	values=[0.5, 0.5]
)

cpd_sport = TabularCPD(
	variable="PractiseSport",
	variable_card=2,
	values=[[0.9, 0.3],[0.1, 0.7]],
	evidence=["BeOverweight"],
	evidence_card=[2],
)

cpd_diet = TabularCPD(
	variable="Diet",
	variable_card=2,
	values=[[0.2, 0.55], [0.8, 0.45]],
	evidence=["BeOverweight"],
	evidence_card=[2],
)

cpd_fit = TabularCPD(
	variable="BeFit",
	variable_card=2,
	values=values=[
		[0.2, 0.25, 0.10, 0.05], #B=0
		[0.80, 0.75, 0.90, 0.95] #B=1
	],
	evidence=["PractiseSport","Diet"],
	evidence_card=[2,2]
)
```

**Question 75:** Considering the goal of cleaning the floor. We suppose that the floor can be cleaned by a person or by the Roomba robot shown in Figure 1. The person cleans the floor only when she/he is not tired. Otherwise, the robot cleans it for her/him. The person is tired with a probability equal to 0.75. Model the situation described above with a Bayesian Network.

Random variables:
- Tiredness represents whether the person is tired or not.
- Person's Action represents whether the person cleans or not.
- Robot's Action represents whether the robot cleans or not.
- Cleaned Floor represents whether the floor is clean or not

The cleaned floor depends on the Person's Action and the Robot's Action which are influenced by the Tiredness of the person.

**Question 76:** Explain how it is possible to infer the structure of a Bayesian Network based on the data and using the library `pgmpy`.

Inside the `pgmpy` library, it is possible to infer the structure of a Bayesian Network using an estimator for instance via `ExhaustiveSearch` or HillClimbing Search based on the data and a score (e.g., K2Score, BDeuScore, BicScore). However, the main requirement in order to achieve an accurate and reliable estimated model is to have enough data.