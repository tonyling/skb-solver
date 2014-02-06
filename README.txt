Author: Tony Ling
README file describing how to compile and run the program as well as 
implementation details.

Heuristics resources:
  http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.164.9379&rep=rep1&type=pdf
  http://fragfrog.nl/papers/solving_the_sokoban_problem.pd

Program compiled and tested on Ubuntu 12.04 LTS using GCC.
  To compile type:        make
  To run type:            ./skb <your_text_file>.txt
  To remove files type:   make clean

Sokoban level character key:
     (empty)      Empty floor
  #  (hash)       Wall 
  .  (period)     Empty goal 
  @  (at)         Player on floor 
  +  (plus)       Player on goal 
  $  (dollar)     Box on floor 
  *  (asterisk)   Box on goal 

Example level:
7
####
# .#
#  ###
#*@  #
#  $ #
#    #
######

*NOTE* The program does not check for incorrect levels.  It is assumed
       that levels only have 1 player at any given time, and that for
       each box in the level, there are reachable goals for those boxes.
       The program also assumes that the level is surrounded by walls.
       Explored lists are used to prevent infinite loops in some cases 
       of incorrect levels.

Sokoban Solver Implementation:

Citations:
 Takes, Frank.
  Sokoban: Reversed Solving. N.p.: n.p., 20 June 2008. PDF.
 Dorst, Matthijs, Alejandro Marzinotto, Marin Gerontini, and Radu-Mihai Pana-Talpeanu.
  Solving the Sokoban Problem. N.p.: n.p., 14 Oct. 2011. PDF.

Breadth First Search:
  This search algoritm uses a queue for the states to explore, and an explored
  list to keep track of previous states already explored to prevent looping.
  It uses a standard algorithm.  Take a state from front of the queue, and exit
  if it is a goal state.  Otherwise, generate all possible valid successor 
  states and push back to the end of the queue.
  
Depth First Search:
  Although a standard algorithm for depth first search does not keep track of
  explored states, an explored list is used for this solver.  This is because
  the depth of a sokoban level is infinite (IE the player can just go up, then 
  down, then up, then down... forever), it would not work without one.  This
  algorithm uses a list to store all states to explore.  It takes the first node
  from the list, and exit if it is a goal state.  Otherwise it generates all
  valid states and adds it to the front of the list.

Uniform Cost Search:
  The uniform cost search uses a priority queue to keep track of the total
  cost from the initial state to the node and chooses the one with the lowest
  cost.  It takes a state from the front of the queue and exits if goal state.
  All valid states generated are placed in the queue based off of total costs.
  An explored list is used to keep track of states to preventing looping.
  Under the parameters of this assignment, the results seen from a UCS algorithm
  gives similar answers to a BFS search, yet takes longer and explores more
  nodes.  If the move and push costs of this algorithm were more drastic, I
  would expect to see more variance to the solutions given by this algorithm.

Greedy Best First Search:
  A greedy best first search uses a priority queue.  It takes the first state
  from the queue, and exits if it is the goal state.  Otherwise, it generates
  all valid states and place them in the queue based off of their heuristics
  function.  An explored list is used to prevent infinite looping.  All valid
  states are placed in the queue based on a score.  New nodes with score found
  to be identical to those in the queue are placed at the end of the list of
  nodes that have the same score, effectively appending to nodes with the same
  score.
  
A* Search:
   This search algorithm is exactly the same as Greedy Best First Search,
   however the total cost from the initial state to the node is added onto
   heuristics function score.

Heuristics Function 1:
  As described on page 3 of "Solving the Sokoban Problem", the heuristics used
  is one where it checks the distance between all goals and all boxes, the 
  distance between a player and the nearest box, and whether or not there are
  boxes already on goals.  The scores for this function starts are 0.
  Since priority queues used prioritizes lower scores, boxes already on goals
  add a -1000 score.  This values states with non empty goal states,
  however would not be adequate for sokoban levels where boxes have to be
  pushed away from a goal.  All the distances are added to the score, which 
  means larger distances reducing a state priority.  This implementation is not
  admissible as seen from the results.
  
Heuristics Function 2:
  The heuristics used here is based off of the heuristics described in 
  "Sokoban": Reversed Solving" on page 3.  Instead of taking into account
  the distances of the boxes, goals and player, this function instead ranks
  a state depending on what the boxes are next to.  The score starts at 0
  as well, and checks to see if any box is in an "unsafe position."  An unsafe
  position would be corners with no goal states, and along unbroken walls of
  2 corners that have no goal state along the wall.  An box in a corner would
  give a positive +1000 score, while any unsafe position along a wall also gives
  a +1000 score.  The lowest possible score is a 0, which means no unsafe
  positions are detected at all.  While this sounds like deadlock detection, it
  is only so with corners.  This heuristics does not consider any the box to be
  in a unsafe position if it is not next to a wall, which means that even though
  the box cannot reach the goal, the heuristics does not see take into account
  the whole level.  This heuristics can be thought of as giving lower priority
  to states that are unfavorable, compared to prioritizing favorable states in
  heuristics such as function 1.
  
*NOTE* Greedy Best First Search and A* search w/ Heuristics Function 2 Results:
  The greedy best first search with heuristics function 2 is able to produce 
  optimal results because it is similar to a breadth first search.  The
  heuristics function gives lowest scores to states where boxes are in unsafe
  positions, but the algorithm pushs all safe states in front of the unsafe 
  states.  This is essentially pushing back all states except for the ones
  that are unsafe, compared to a breadth first search that does not 
  differentiate between safe and unsafe states.  Due to the parameters of
  this assignment, the results of both the greedy best first search and A* 
  search with heuristics function 2 are identical.  Since both pushes and moves
  are both a cost of 1, each sucessor node of a safe state would always be at a
  cost of +1 of total path in the A* search.  Even though a priority queue is
  used, since each succesor is always +1, it is estentially appended to the end
  of the queue of safe states, since the priority queue is ordered from lowest
  cost to greatest.  Therefore when the push cost and the move cost is the same,
  there is not difference between any successor node and all are appended in
  front of unsafe states, the same as how the greedy best first search with
  appending same costs nodes to the front of the unsafe states.
