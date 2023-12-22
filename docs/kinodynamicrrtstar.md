# Kinodynamic RRT*
This article introduces an incremental sampling-based approach designed for asymptotically optimal motion planning, specifically tailored for robots characterized by linear differential constraints. Building upon the foundation of RRT*—originally devised for holonomic robots—the proposed approach extends its capabilities by incorporating a fixed-final-state-free-final-time controller. This controller precisely and optimally connects any pair of states, considering a cost function that strikes a balance between trajectory duration and control effort.

The efficacy of this approach is demonstrated through the computation of asymptotically optimal trajectories in three distinct motion planning scenarios. These scenarios include a planar robot with a 4-dimensional state space and double integrator dynamics, an aerial vehicle with a 10-dimensional state space and linearized quadrotor dynamics, and a car-like robot with a 5-dimensional state space and non-linear dynamics.

The sampling-based planner employed in this work utilizes probabilistic roadmaps and rapid-exploring random trees. Additionally, the paper delves into the computational complexity theory, categorizing the problem at hand as PSPACE-hard—a class of decision problems solvable by a Turing machine with polynomial space.

While PRM (Probabilistic Roadmaps) and RRT (Rapidly-exploring Random Trees) are asymptotically complete, the article acknowledges the practical success of RRT* but highlights its limitation to systems with simple dynamics. RRT* relies on the ability to connect any pair of states with an optimal trajectory, making it primarily suitable for holonomic robots.

In contrast, the proposed algorithm in this paper addresses the limitation by designing a method capable of optimally connecting states in the state-space, accommodating control inputs and boundary conditions. The algorithm showcases its efficiency, particularly for holonomic robots, and its potential applicability to kinodynamic systems by linearizing nonlinear dynamics.

The paper emphasizes the importance of considering differential constraints in kinodynamic systems, where straight-line connections between states are typically invalid trajectories. It introduces a two-point boundary value problem for these systems.

In summary, this article contributes a novel algorithm for optimal trajectory generation, considering state-space dynamics, collision avoidance, and adherence to dynamic constraints. The proposed method exhibits efficiency and applicability to various robotic systems, showcasing its potential impact on kinodynamic planning in complex scenarios.


# Kinodynamic RRT* Algorithm

#### Input:
- $ Q_{\text{init}} $: Initial configuration of the robot.
- $ Q_{\text{goal}} $: Goal configuration of the robot.
- \\(\text{ControlSpace}\\): Set of feasible control inputs for the robot.
- \\(\text{MaxIter}\\): Maximum number of iterations.
- $ \Delta t $: Time step.
- $ \epsilon $: Radius of the ball around a node for cost evaluation.

#### Output:
A tree of configurations rooted at $ Q_{\text{init}} $ and connected towards $ Q_{\text{goal}} $. An optimized path with minimized cost.

#### Procedure:
1. **Initialization:**
   - Create an empty tree $ T $ with $ Q_{\text{init}} $ as the root.
2. **for** $ i = 1 $ to \\(\text{MaxIter}\\):
   - $ Q_{\text{rand}} $ = RandomConfiguration()
   - $ q_{\text{near}} $ = NearestNeighbor($ Q_{\text{rand}}, T $)
   - $ U $ = ControlInputs($ q_{\text{near}}, $ Q_{\text{rand}} $)
   - **for each** $ u $ in $ U $ **do**:
      - $ q_{\text{new}} $ = KinodynamicExtend($ q_{\text{near}}, u, $ \Delta t $)
      - **if** $ q_{\text{new}} $ is collision-free **then**:
         - $ \text{NearNodes} $ = Near($ T, $ q_{\text{new}}, $ \epsilon $)
         - $ q_{\text{min}} $ = ChooseParent($ T, $ q_{\text{new}}, $ \text{NearNodes}, $ U $)
         - Add $ q_{\text{new}} $ to $ T $ with $ q_{\text{min}} $ as its parent.
         - RewireTree($ T, $ q_{\text{new}}, $ \text{NearNodes}, $ U $)
   - **end for**
3. **end for**
4. **ExtractPath:**
   - If $ Q_{\text{goal}} $ is in the tree, trace the path from $ Q_{\text{goal}} $ to the root $ Q_{\text{init}} $. Choose the path with the minimum cost.

#### Subroutines:
- **RandomConfiguration():**
   - Generate a random configuration in the configuration space.
- **NearestNeighbor($ Q, T $):**
   - Find the nearest node in the tree $ T $ to the given configuration $ Q $.
- **ControlInputs($ q_{\text{near}}, Q_{\text{rand}} $):**
   - Generate a set of feasible control inputs to move from $ q_{\text{near}} $ towards $ Q_{\text{rand}} $.
- **KinodynamicExtend($ q_{\text{near}}, u, $ \Delta t $):**
   - Propagate the system dynamics from $ q_{\text{near}} $ using control input $ u $ for a time step $ \Delta t $.
- **Near($ T, $ q, $ \epsilon $):**
   - Return the set of nodes in the tree $ T $ that are within a ball of radius $ \epsilon $ around $ q $.
- **ChooseParent($ T, $ q, $ \text{NearNodes}, $ U $):**
   - Find the parent node in $ T $ that minimizes the cost considering the new node $ q $, the set of near nodes $ \text{NearNodes} $, and the set of control inputs $ U $.
- **RewireTree($ T, $ q, $ \text{NearNodes}, $ U $):**
   - Rewire the tree $ T $ to improve the tree structure based on the new node $ q $, the set of near nodes $ \text{NearNodes} $, and the set of control inputs $ U $.


![](img/kinopsuedocode.png)
![](img/kinorrtx.png)

## Kinodynamic Planning

In the realm of robotics and motion planning, kinodynamic planning constitutes a class of challenges where stringent constraints related to velocity, acceleration, and force/torque must be adhered to. The objective of kinodynamic planning revolves around navigating a robot from an initial state to a designated goal region, all while circumventing obstacles and adhering to both kinematic and dynamic constraints. These constraints dictate the intricate relationship between a robot's controls and its motion. The term "kinodynamic" was first introduced in 1993, marking a pivotal moment in the evolution of planning methodologies. Within kinodynamic planning, a vehicle's dynamics are treated differentially, providing a nuanced understanding of its motion characteristics.

## References

1.<https://stanfordasl.github.io/wp-content/papercite-data/pdf/Schmerling.Pavone.EOR19.pdf>
2.<https://en.wikipedia.org/wiki/Kinodynamic_planning>
3.<https://arxiv.org/abs/1205.5088>
4.<https://ieeexplore.ieee.org/abstract/document/6631299>