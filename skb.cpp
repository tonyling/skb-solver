/*  Author: Tony Ling
 *  Summary: HW2 for W4701 AI Fall 2013 @ Columbia University
 * 
 *  Notes: 9/9/2013  - File created.  Created generation of initial level/state
 *         9/10/2013 - BFS started. Gen_valid_states implemented.
 *         9/11/2013 - BFS implemented. SearchStat struct used for statistics.
 *         9/12/2013 - DFS implemented. UCS implemented.
 *         9/15/2013 - Heuristics function 1 implemented for greedy best first
 *                     search and A* search. enum search_mode added.
 *         9/16/2013 - Heuristics function 2 implemented for greedy best first
 *                     search and A* search.
 * 
 *  Sokoban level character key:
 *   	(empty) 	Empty floor
 * 	#	(hash)  	Wall 
 * 	.	(period)	Empty goal 
 * 	@	(at)    	Player on floor 
 * 	+	(plus)		Player on goal 
 * 	$	(dollar)	Box on floor 
 * 	*	(asterisk)	Box on goal 
 * 
 *  Resources: 
 *    Runtime:
 *      http://pubs.opengroup.org/onlinepubs/7908799/xsh/systime.h.html
 *    Heuristics:
 *      http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.164.9379&rep=rep1&type=pdf
 *      http://fragfrog.nl/papers/solving_the_sokoban_problem.pdf
 */
#include <sys/time.h>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <queue>
#include <deque>
#include <string>
#include <fstream>

enum search_mode {NONE, UCS, GBFSH1, GBFSH2, ASH1, ASH2, BFS, DFS};

/* States contain the level, how many moves/pushes and move list to get
 * to that state, and how deep that state is in node terms
 */
struct State
{
	std::string state_str;
	std::string move_list;
	int depth;
	int moves;
	int pushes;
	int total_cost;
	int hscore;
}; //struct State

/* SearchStat used to count search algorithm information, such as amount
 * of nodes, duplicate nodes, fringe nodes and explored nodes.
 */
struct SearchStat
{
	State node;
	int node_count;
	int rep_node_count;
	int fringe_node;
	int explored_count;
}; //struct SearchStat

/* Function used to check if state is the goal state. Goal state is a
 *  state with no empty goals, nor player on goal. Assumes that there
 *  exists at least 1 empty goal, ie a level with at least 1 empty goal
 *  to fill, with the same amount of goals as boxes, and only 1 player.
 * 
 * Preconditions: State object representing current state in agent prog
 * Postconditions: Returns true if goal state is found, otherwise false
 */
bool is_goal(State &cur_state)
{
	//std::cout << cur_state.state_str<<std::endl;
	bool goal = false;
	//if there are no empty goals
	if ((cur_state.state_str.find_first_of('.')) == std::string::npos)
	{
		//if there are no players standing on an empty goal,
		//then we are in goal state
		if ((cur_state.state_str.find_first_of('+')) == std::string::npos)
		{
			//and there are no boxes not on a goal
			if ((cur_state.state_str.find_first_of('$')) == std::string::npos)
				goal = true;
		}
	}
	return goal;
} //bool is_goal(State &cur_state)

/* Function used to print out a level state if that state is stored in a
 *  vector of vector<char>.  It iterattes through the vector, printing
 *  out each char of each vector.
 * 
 * Preconditions: vector<vector<char>> object
 * Postcoditions: prints out each element in the vector of vectors, with
 *  new line seperated each vector
 */
void print_level(std::vector< std::vector<char> > &map)
{
	for (int i = 0; i < map.size(); i++)
	{
		std::vector<char> vec = map[i];
		std::vector<char>::iterator itr;
		for (itr = vec.begin(); itr != vec.end(); itr++)
			std::cout << *itr;
		std::cout << std::endl;
	}
} //void print_level(std::vector< std::vector<char> > &map)

/* Heuristics function uses Manhattan distance between player and
 *  nearest box, and between the boxes and the goals.
 *  Assumes only 1 player on level.
 * 
 * Preconditions: State object
 * Postcoditions: int return representing heuristics score
 */
int h1(const State &cur_state)
{
	std::stringstream ss(cur_state.state_str);
	std::string line;
	int x, y, counter = 0;
	std::vector< std::vector<int> > box_list;
	std::vector< std::vector<int> > goal_list;
	int score = 0;
	int playerdist = 1000;

	while (getline(ss,line, '\n'))
	{
		for (int i = 0; i <line.length() ; i++)
		{
			//assumes only 1 player exists in any given state
			//if found user, set x, y positions
			if (line[i] == '@')
			{
				x = i;
				y = counter;
			}
			//if player is on an empty goal, pos of player
			//is the same as an empty goal
			else if (line[i] == '+' )
			{
				x = i;
				y = counter;
				std::vector<int> goal_loc;
				goal_loc.push_back(i);
				goal_loc.push_back(counter);
				goal_list.push_back(goal_loc);
			}
			//if box on floor, add to box list
			else if (line[i] == '$' )
			{
				std::vector<int> box_loc;
				box_loc.push_back(i);
				box_loc.push_back(counter);
				box_list.push_back(box_loc);
			}
			//if empty goal, add to goal list
			else if (line[i] == '.' )
			{
				std::vector<int> goal_loc;
				goal_loc.push_back(i);
				goal_loc.push_back(counter);
				goal_list.push_back(goal_loc);
			}
			//if there is a box on a goal, then it should take
			//precidence over non fill goals in case there are multiple
			//boxes and goals.  we want the program to let the box remain
			//on the goal.  to this end, -1000 is added to the score
			else if(line[i] == '*')
			{
				score -= 1000;
			}
		}
		counter++;
	}

	//for each box on the floor, calculate the distance to each empty goal
	for (int i = 0; i < box_list.size(); i++)
	{
		for (int j = 0; j < goal_list.size(); j++)
		{
			//calculate distance of x and y cords
			int xdist = box_list[i][0] - goal_list[j][0];
			int ydist = box_list[i][1] - goal_list[j][1];
			//take absolute value of distance
			if (xdist < 0)
				xdist *= -1;
			if (ydist < 0)
				ydist *= -1;
			//add distance to score, lower score = better
			score += xdist;
			score += ydist;
		}
		//calculate player to box distances
		int p_to_box_x = box_list[i][0] - x;
		int p_to_box_y = box_list[i][1] - y;
		//take absolute value of distance
		if (p_to_box_x < 0)
			p_to_box_x *= -1;
		if (p_to_box_y < 0)
			p_to_box_y *= -1;
		//stores shortest distance to any box
		//NOTE: This only works if shortest distance is within 1000 tiles
		if (playerdist > (p_to_box_y + p_to_box_x))
			playerdist = (p_to_box_y + p_to_box_x);
	}
	score += playerdist;
	return score;
} //int h1(const State &cur_state)

/* Heuristics function checks to see if boxes not on goals are in any
 *  'unsafe' position.  Unsafe positions are corners with no goals, and
 *  tiles along unbroken walls of 2 unsafe corners with no goals along
 *  the wall.  Unlike heuristics function 1, this function calculates
 *  no distances between any objects, nor prioritizes boxes already on
 *  goals.
 * 
 * Preconditions: State object
 * Postcoditions: int return representing heuristics score
 */
int h2(const State &cur_state)
{
	std::stringstream ss(cur_state.state_str);
	std::vector< std::vector<int> > box_list;
	std::vector< std::vector<char> > level_map;
	std::string line;
	int score = 0, counter = 0;

	//generates level map and marks where boxes are level map stored as
	//(y,x) cordinates since reads lines by line first (y plane) and
	//then char by char (x plane)
	while (getline(ss,line, '\n'))
	{
		std::vector<char> temp;
		level_map.push_back(temp);
		for (int i = 0; i <line.length() ; i++)
		{
			//if box on floor, add to box list
			//ignores boxes on goals since they cannot
			//be unsafe positions
			if (line[i] == '$' )
			{
				std::vector<int> box_loc;
				box_loc.push_back(i);
				box_loc.push_back(counter);
				box_list.push_back(box_loc);
			}
			level_map[counter].push_back(line[i]);
		}
		counter++;
	}

	//for each box on the floor, check if it is adjacent to a wall
	//if adjacent against 2 walls that are next to each other,
	//then  it is a corner
	for (int i = 0; i < box_list.size(); i++)
	{
		bool N_wall = false;
		bool E_wall = false;
		bool S_wall = false;
		bool W_wall = false;
		bool in_corner = false;
		
		int cur_box_x = box_list[i][0];
		int cur_box_y = box_list[i][1];
		
		//check if there is a wall north of the box
		if (level_map[cur_box_y - 1][cur_box_x] == '#')
			N_wall = true;
		//check if there is a wall east of the box
		if (level_map[cur_box_y][cur_box_x + 1] == '#')
			E_wall = true;
		//check if there is a wall south of the box
		if (level_map[cur_box_y + 1][cur_box_x] == '#')
			S_wall = true;
		//check if there is a wall west of the box
		if (level_map[cur_box_y][cur_box_x - 1] == '#')
			W_wall = true;
		
		//first check if box is in a corner
		//check if box in NE corner
		if (N_wall && E_wall)
		{
			in_corner = true;
		}
		//check if box in NW corner
		else if (N_wall && W_wall)
		{
			in_corner = true;
		}
		//check if box in SE corner
		else if (S_wall && E_wall)
		{
			in_corner = true;
		}
		//check if box in SW corner
		else if (S_wall && W_wall)
		{
			in_corner = true;
		}
		
		//if box is ever in a corner, then box is in a deadlock position
		if (in_corner)
		{
			score += 1000;
		}
		//if box is next to a wall, check to see if wall is unbroken with
		//2 unsafe corners and no goals along the wall
		else
		{
			//if wall north of box, search for and east-most and west-most wall
			if (N_wall)
			{
				bool safe = false;
				bool corner_E = false;
				bool corner_W = false;
				
				//search east to see if there are accessible tiles along unbroken
				//north walls until a corner is found.  boxes and players are ignored
				//and considered accessible tiles since they can move
				for (int i = cur_box_x + 1; i < level_map[cur_box_y].size(); i++)
				{
					//if goal is found along the way then it cannot be an unsafe position
					if ((level_map[cur_box_y][i] == '.') ||
					 (level_map[cur_box_y][i] == '*') || (level_map[cur_box_y][i] == '+'))
					{
						safe = true;
						break;
					}
					
					//if north tile is not a wall, then it is safe
					if (level_map[cur_box_y - 1][i] != '#')
					{
						safe = true;
						break;
					}
					
					//if NE corner
					if ((level_map[cur_box_y][i] == '#') && (level_map[cur_box_y - 1][i] == '#'))
					{
						corner_E = true;
						break;
					}
				}
				
				//search west to see if there are accessible tiles along unbroken
				//north walls until a corner is found.  boxes and players are ignored
				//and considered accessible tiles since they can move
				for (int i = cur_box_x - 1; i >= 0 ; i--)
				{
					//if goal is found along the way then it cannot be an unsafe position
					if ((level_map[cur_box_y][i] == '.') ||
					 (level_map[cur_box_y][i] == '*') || (level_map[cur_box_y][i] == '+'))
					{
						safe = true;
						break;
					}
					
					//if north tile is not a wall, then it is safe
					if (level_map[cur_box_y - 1][i] != '#')
					{
						safe = true;
						break;
					}
					
					//if NW corner
					if ((level_map[cur_box_y][i] == '#') && (level_map[cur_box_y - 1][i] == '#'))
					{
						corner_W = true;
						break;
					}
				}
				
				//if unbroken wall along path east and west of the box to corners with any goal
				//increment score to signify unsafe position
				if (!safe)
				{
					if(corner_E && corner_W)
						score += 1000;
				}	
			}
			
			//if wall north of box, search for and north-most and south-most wall
			if (E_wall)
			{
				bool safe = false;
				bool corner_N = false;
				bool corner_S = false;
				
				//search north to see if there are accessible tiles along unbroken
				//east walls until a corner is found.  boxes and players are ignored
				//and considered accessible tiles since they can move
				for (int i = cur_box_y - 1; i >=0; i--)
				{
					//if goal is found along the way then it cannot be an unsafe position
					if ((level_map[i][cur_box_x] == '.') ||
					 (level_map[i][cur_box_x] == '*') || (level_map[i][cur_box_x] == '+'))
					{
						safe = true;
						break;
					}
					
					//if east tile is not a wall, then it is safe
					if (level_map[i][cur_box_x + 1] != '#')
					{
						safe = true;
						break;
					}
					
					//if NE corner
					if ((level_map[i][cur_box_x] == '#') && (level_map[i][cur_box_x + 1] == '#'))
					{
						corner_N = true;
						break;
					}
				}
				
				//search south to see if there are accessible tiles along unbroken
				//east walls until a corner is found.  boxes and players are ignored
				//and considered accessible tiles since they can move
				for (int i = cur_box_y + 1; i < level_map.size(); i++)
				{
					//if goal is found along the way then it cannot be an unsafe position
					if ((level_map[i][cur_box_x] == '.') ||
					 (level_map[i][cur_box_x] == '*') || (level_map[i][cur_box_x] == '+'))
					{
						safe = true;
						break;
					}
					
					//if east tile is not a wall, then it is safe
					if (level_map[i][cur_box_x + 1] != '#')
					{
						safe = true;
						break;
					}
					
					//if SE corner
					if ((level_map[i][cur_box_x] == '#') && (level_map[i][cur_box_x + 1] == '#'))
					{
						corner_S = true;
						break;
					}
				}
				
				//if unbroken wall along path east and west of the box to corners with any goal
				//increment score to signify unsafe position
				if (!safe)
				{
					if(corner_N && corner_S)
						score += 1000;
				}	
			}
			
			//if wall south of box, search for and east-most and west-most wall
			if (S_wall)
			{
				bool safe = false;
				bool corner_E = false;
				bool corner_W = false;
				
				//search east to see if there are accessible tiles along unbroken
				//south walls until a corner is found.  boxes and players are ignored
				//and considered accessible tiles since they can move
				for (int i = cur_box_x + 1; i < level_map[cur_box_y].size(); i++)
				{
					//if goal is found along the way then it cannot be an unsafe position
					if ((level_map[cur_box_y][i] == '.') ||
					 (level_map[cur_box_y][i] == '*') || (level_map[cur_box_y][i] == '+'))
					{
						safe = true;
						break;
					}
					
					//if south tile is not a wall, then it is safe
					if (level_map[cur_box_y + 1][i] != '#')
					{
						safe = true;
						break;
					}
					
					//if SE corner
					if ((level_map[cur_box_y][i] == '#') && (level_map[cur_box_y + 1][i] == '#'))
					{
						corner_E = true;
						break;
					}
				}
				
				//search west to see if there are accessible tiles along unbroken
				//south walls until a corner is found.  boxes and players are ignored
				//and considered accessible tiles since they can move
				for (int i = cur_box_x - 1; i >= 0 ; i--)
				{
					//if goal is found along the way then it cannot be an unsafe position
					if ((level_map[cur_box_y][i] == '.') ||
					 (level_map[cur_box_y][i] == '*') || (level_map[cur_box_y][i] == '+'))
					{
						safe = true;
						break;
					}
					
					//if south tile is not a wall, then it is safe
					if (level_map[cur_box_y + 1][i] != '#')
					{
						safe = true;
						break;
					}
					
					//if SW corner
					if ((level_map[cur_box_y][i] == '#') && (level_map[cur_box_y + 1][i] == '#'))
					{
						corner_W = true;
						break;
					}
				}
				
				//if unbroken wall along path east and west of the box to corners with any goal
				//increment score to signify unsafe position
				if (!safe)
				{
					if(corner_E && corner_W)
						score += 1000;
				}	
			}
			
			//if wall north of box, search for and north-most and south-most wall
			if (W_wall)
			{
				bool safe = false;
				bool corner_N = false;
				bool corner_S = false;
				
				//search north to see if there are accessible tiles along unbroken
				//east walls until a corner is found.  boxes and players are ignored
				//and considered accessible tiles since they can move
				for (int i = cur_box_y - 1; i >=0; i--)
				{
					//if goal is found along the way then it cannot be an unsafe position
					if ((level_map[i][cur_box_x] == '.') ||
					 (level_map[i][cur_box_x] == '*') || (level_map[i][cur_box_x] == '+'))
					{
						safe = true;
						break;
					}
					
					//if east tile is not a wall, then it is safe
					if (level_map[i][cur_box_x - 1] != '#')
					{
						safe = true;
						break;
					}
					
					//if NW corner
					if ((level_map[i][cur_box_x] == '#') && (level_map[i][cur_box_x - 1] == '#'))
					{
						corner_N = true;
						break;
					}
				}
				
				//search south to see if there are accessible tiles along unbroken
				//east walls until a corner is found.  boxes and players are ignored
				//and considered accessible tiles since they can move
				for (int i = cur_box_y + 1; i < level_map.size(); i++)
				{
					//if goal is found along the way then it cannot be an unsafe position
					if ((level_map[i][cur_box_x] == '.') ||
					 (level_map[i][cur_box_x] == '*') || (level_map[i][cur_box_x] == '+'))
					{
						safe = true;
						break;
					}
					
					//if east tile is not a wall, then it is safe
					if (level_map[i][cur_box_x - 1] != '#')
					{
						safe = true;
						break;
					}
					
					//if SW corner
					if ((level_map[i][cur_box_x] == '#') && (level_map[i][cur_box_x - 1] == '#'))
					{
						corner_S = true;
						break;
					}
				}
				
				//if unbroken wall along path east and west of the box to corners with any goal
				//increment score to signify unsafe position
				if (!safe)
				{
					if(corner_N && corner_S)
						score += 1000;
				}	
			}
		}
	}
	return score;
} //int h2(const State &cur_state)


/* Function is used to generate all valid states from current state
 * A valid level is assumed.  Meaning only 1 player agent, and 1 goal
 *  for each box, with at least 1 goal and 1 box.
 * Called in search algorithm functions.
 * 
 * smode cases:
 *  NONE - totalcost of moves are ignored in bfs/dfs
 *  UCS - totalcost of moves are calculated
 *  GBFSH1 - totalcost of moves ignored, heuristics function 1 used
 *  GBFSH2 - totalcost of moves ignored, heuristics function 2 used
 *  ASH1 - totalcost of moves calculated, heuristics function 1 used with totalcost
 *  ASH2 - totalcost of moves calculated, heuristics function 2 used with totalcost
 * 
 * Preconditions: State object representing current state in the agent,
 *  int representing what searching algorithm used
 * Postconditions: Returns a queue of state objects representing all possible 
 *  states from the current state
 */
std::queue<State> gen_valid_states (const State &cur_state, const int smode = NONE)
{
	std::queue<State> valid_moves;
	std::stringstream ss(cur_state.state_str);
	std::vector< std::vector<char> > new_level_map;
	State new_state;
	std::string line;
	bool found = false;
	char player, box_move;
	int x, y, counter = 0, MOVE_COST = 1, PUSH_COST = 1;

	//if search algorithm is uniformed cost search, set push_cost to 2
	if (smode == UCS)
		PUSH_COST = 2;

	//generate vector array since indexes used for movement
	//are easier, eg moving down at pos [1,2] leads to [2,2]
	//level_map stored as (y,x) cordinates due to reading line by line
	std::vector< std::vector<char> > level_map;
	while (getline(ss,line, '\n'))
	{
		std::vector<char> temp;
		level_map.push_back(temp);
		for (int i = 0; i <line.length() ; i++)
		{
			if (!found)
			{
				//assumes only 1 player exists in any given state
				//if found user, set found to true, and x, y positions
				if (line[i] == '@' | line[i] == '+' )
				{
					player = line[i];
					x = i;
					y = counter;
					found = true;
				}
			}
			level_map[counter].push_back(line[i]);
		}
		counter++;
	}
	if (!found)
	{
		std::cout<<"No player found on level"<<std::endl;
		return valid_moves;
	}
	
	//north tile pos= x, y-1
	//generate north move
	char north = level_map[y-1][x];
	switch (north)
	{
		//move to empty spot
		case ' ':
			new_level_map = level_map;
			//adjusting for player tile and tile north of player
			new_level_map[y-1][x] = '@';
			//if player = @, then was standing on empty floor
			//otherwise player was standing on goal
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("u, ");
			new_state.depth++;
			new_state.moves++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += MOVE_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;	
			valid_moves.push(new_state);
			break;
		//move to empty goal
		case '.':
			new_level_map = level_map;
			//adjusting for player tile and tile north of player
			new_level_map[y-1][x] = '+';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("u, ");
			new_state.depth++;
			new_state.moves++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += MOVE_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to box on floor
		case '$':
			new_level_map = level_map;
			//adjusting for player tile and tile north of player
			new_level_map[y-1][x] = '@';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//adjusting for box tile and tile north of box
			box_move = new_level_map[y-2][x];
			//if north of box is a wall or another box
			if (box_move == '#')
				break;
			else if (box_move == '$')
				break;
			else if (box_move == '*')
				break;
			//if north of box is an empty floor
			else if (box_move == ' ')
			{
				new_level_map[y-2][x] = '$';
			}
			//if north of box is an empty goal
			else if (box_move == '.')
			{
				new_level_map[y-2][x] = '*';
			}
			else
				break;
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("u, ");
			new_state.depth++;
			new_state.pushes++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += PUSH_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to box on goal
		case '*':
			new_level_map = level_map;
			//adjusting for player tile and tile north of player
			new_level_map[y-1][x] = '+';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//adjusting for box tile and tile north of box
			box_move = new_level_map[y-2][x];
			//if north of box is a wall or another box
			if (box_move == '#')
				break;
			else if (box_move == '$')
				break;
			else if (box_move == '*')
				break;
			//if north of box is an empty floor
			else if (box_move == ' ')
			{
				new_level_map[y-2][x] = '$';
			}
			//if north of box is an empty goal
			else if (box_move == '.')
			{
				new_level_map[y-2][x] = '*';
			}
			else
				break;
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("u, ");
			new_state.depth++;
			new_state.pushes++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += PUSH_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to wall
		case '#':
			break;
		default:
			break;
	}
	
	//east tile pos= x, y-1
	//generate east move
	char east = level_map[y][x+1];
	switch (east)
	{
		//move to empty spot
		case ' ':
			new_level_map = level_map;
			//adjusting for player tile and tile east of player
			new_level_map[y][x+1] = '@';
			//if player = @, then was standing on empty floor
			//otherwise player was standing on goal
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("r, ");
			new_state.depth++;
			new_state.moves++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += MOVE_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			//std::cout<<new_state.state_str<<std::endl;
			valid_moves.push(new_state);
			break;
		//move to empty goal
		case '.':
			new_level_map = level_map;
			//adjusting for player tile and tile east of player
			new_level_map[y][x+1] = '+';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("r, ");
			new_state.depth++;
			new_state.moves++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += MOVE_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to box on floor
		case '$':
			new_level_map = level_map;
			//adjusting for player tile and tile east of player
			new_level_map[y][x+1] = '@';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//adjusting for box tile and tile east of box
			box_move = new_level_map[y][x+2];
			//if east of box is a wall or another box
			if (box_move == '#')
				break;
			else if (box_move == '$')
				break;
			else if (box_move == '*')
				break;
			//if east of box is an empty floor
			else if (box_move == ' ')
			{
				new_level_map[y][x+2] = '$';
			}
			//if east of box is an empty goal
			else if (box_move == '.')
			{
				new_level_map[y][x+2] = '*';
			}
			else
				break;
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("r, ");
			new_state.depth++;
			new_state.pushes++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += PUSH_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1|| smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to box on goal
		case '*':
			new_level_map = level_map;
			//adjusting for player tile and tile east of player
			new_level_map[y][x+1] = '+';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//adjusting for box tile and tile east of box
			box_move = new_level_map[y][x+2];
			//if east of box is a wall or another box
			if (box_move == '#')
				break;
			else if (box_move == '$')
				break;
			else if (box_move == '*')
				break;
			//if east of box is an empty floor
			else if (box_move == ' ')
			{
				new_level_map[y][x+2] = '$';
			}
			//if east of box is an empty goal
			else if (box_move == '.')
			{
				new_level_map[y][x+2] = '*';
			}
			else
				break;
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("r, ");
			new_state.depth++;
			new_state.pushes++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += PUSH_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to wall
		case '#':
			break;
		default:
			break;
	}
	
	//south tile pos= x, y-1
	//generate south move
	char south = level_map[y+1][x];
	switch (south)
	{
		//move to empty spot
		case ' ':
			new_level_map = level_map;
			//adjusting for player tile and tile south of player
			new_level_map[y+1][x] = '@';
			//if player = @, then was standing on empty floor
			//otherwise player was standing on goal
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("d, ");
			new_state.depth++;
			new_state.moves++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += MOVE_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to empty goal
		case '.':
			new_level_map = level_map;
			//adjusting for player tile and tile south of player
			new_level_map[y+1][x]= '+';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("d, ");
			new_state.depth++;
			new_state.moves++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += MOVE_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to box on floor
		case '$':
			new_level_map = level_map;
			//adjusting for player tile and tile south of player
			new_level_map[y+1][x] = '@';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//adjusting for box tile and tile south of box
			box_move = new_level_map[y+2][x];
			//if south of box is a wall or another box
			if (box_move == '#')
				break;
			else if (box_move == '$')
				break;
			else if (box_move == '*')
				break;
			//if south of box is an empty floor
			else if (box_move == ' ')
			{
				new_level_map[y+2][x]= '$';
			}
			//if south of box is an empty goal
			else if (box_move == '.')
			{
				new_level_map[y+2][x] = '*';
			}
			else
				break;
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("d, ");
			new_state.depth++;
			new_state.pushes++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += PUSH_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to box on goal
		case '*':
			new_level_map = level_map;
			//adjusting for player tile and tile south of player
			new_level_map[y+1][x] = '+';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//adjusting for box tile and tile south of box
			box_move = new_level_map[y+2][x];
			//if south of box is a wall or another box
			if (box_move == '#')
				break;
			else if (box_move == '$')
				break;
			else if (box_move == '*')
				break;
			//if south of box is an empty floor
			else if (box_move == ' ')
			{
				new_level_map[y+2][x] = '$';
			}
			//if south of box is an empty goal
			else if (box_move == '.')
			{
				new_level_map[y+2][x] = '*';
			}
			else
				break;
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("d, ");
			new_state.depth++;
			new_state.pushes++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += PUSH_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to wall
		case '#':
			break;
		default:
			break;
	}

	//west tile pos= x, y-1
	//generate west move
	char west = level_map[y][x-1];
	switch (west)
	{
		//move to empty spot
		case ' ':
			new_level_map = level_map;
			//adjusting for player tile and tile west of player
			new_level_map[y][x-1] = '@';
			//if player = @, then was standing on empty floor
			//otherwise player was standing on goal
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("l, ");
			new_state.depth++;
			new_state.moves++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += MOVE_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to empty goal
		case '.':
			new_level_map = level_map;
			//adjusting for player tile and tile west of player
			new_level_map[y][x-1]  = '+';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("l, ");
			new_state.depth++;
			new_state.moves++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += MOVE_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to box on floor
		case '$':
			new_level_map = level_map;
			//adjusting for player tile and tile west of player
			new_level_map[y][x-1]  = '@';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//adjusting for box tile and tile west of box
			box_move = new_level_map[y][x-2];
			//if west of box is a wall or another box
			if (box_move == '#')
				break;
			else if (box_move == '$')
				break;
			else if (box_move == '*')
				break;
			//if west of box is an empty floor
			else if (box_move == ' ')
			{
				new_level_map[y][x-2] = '$';
			}
			//if west of box is an empty goal
			else if (box_move == '.')
			{
				new_level_map[y][x-2] = '*';
			}
			else
				break;
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("l, ");
			new_state.depth++;
			new_state.pushes++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += PUSH_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to box on goal
		case '*':
			new_level_map = level_map;
			//adjusting for player tile and tile west of player
			new_level_map[y][x-1]  = '+';
			(player == '@') ? new_level_map[y][x] = ' ' : new_level_map[y][x] = '.';
			
			//adjusting for box tile and tile west of box
			box_move = new_level_map[y][x-2];
			//if west of box is a wall or another box
			if (box_move == '#')
				break;
			else if (box_move == '$')
				break;
			else if (box_move == '*')
				break;
			//if west of box is an empty floor
			else if (box_move == ' ')
			{
				new_level_map[y][x-2] = '$';
			}
			//if west of box is an empty goal
			else if (box_move == '.')
			{
				new_level_map[y][x-2] = '*';
			}
			else
				break;
			
			//create and update new state
			new_state = cur_state;
			new_state.state_str = "";
			//turning vector<vector<char>> back to string
			for (int i = 0; i < new_level_map.size(); i++)
			{
				std::vector<char> temp = new_level_map[i];
				std::vector<char>::iterator itr;
				for (itr = temp.begin(); itr != temp.end(); itr++)
					new_state.state_str.push_back( *itr);
				new_state.state_str.append("\n");
			}
			
			//update state stats
			new_state.move_list.append("l, ");
			new_state.depth++;
			new_state.pushes++;
			if (smode == UCS || smode == ASH1 || smode == ASH2)
				new_state.total_cost += PUSH_COST;
			if (smode == GBFSH1 || smode == ASH1)
				new_state.hscore = h1(new_state);
			if (smode == GBFSH2 || smode == ASH2)
				new_state.hscore = h2(new_state);
			if (smode == ASH1 || smode == ASH2)
				new_state.hscore += new_state.total_cost;
			valid_moves.push(new_state);
			break;
		//move to wall
		case '#':
			break;
		default:
			break;
	}

	return valid_moves;
} //std::queue<State> gen_valid_states (const State &cur_state, const int smode = NONE)

/* Function executes depth first search algorithm on an inital state.
 *  Due to sokoban puzzles having infinite depth, an explored list is
 *  used to prevent infinite loops. 
 * 
 * Preconditions: Takes in a State object for initial state of level
 * Postconditions: Returns a SearchStat object for search results stats
 */
SearchStat bfs(State &initial_state)
{
	std::deque<State> open;
	std::vector<State> closed;
	SearchStat report;
	report.rep_node_count = 0;
	report.fringe_node = 0;
	report.explored_count = 1;//will be replaced, just to stop cout spam
	report.node_count = 1;
	report.node.state_str = "NULL";
	State current_state;
	
	//push first state into queue
	open.push_back(initial_state);
	while (!open.empty())
	{
		//take N from OPEN
		current_state = open.front();
		open.pop_front();
		//push N onto CLOSED
		closed.push_back(current_state);
		
		//print out in case a long time is taken and wondering if it froze
		if ((closed.size() % 5000) == 0)
			std::cout << "...explored "<< closed.size() <<" nodes..."<<std::endl;
			
		//if found, set report node to current node, set explored count to closed list size
		if (is_goal(current_state))
		{
			report.node = current_state;
			report.explored_count = closed.size();
			open.pop_front();
			break;
		}
		
		//generate valid states
		std::queue<State> valid_states = gen_valid_states(current_state);
		std::deque<State>::iterator it;
		std::vector<State>::iterator itr;
		
		//while queue is not empty of states
		while (!valid_states.empty())
		{
			bool already_seen = false;
			State temp_state = valid_states.front();
			//check if state has already been seen on open list
			for (it = open.begin(); it != open.end(); it++)
			{
				if (it->state_str == temp_state.state_str)
				{
					already_seen = true;
					break;
				}
			}
			//check if state has already been seen on closed list
			for (itr = closed.begin(); itr != closed.end(); itr++)
			{
				if (itr->state_str == temp_state.state_str)
				{
					already_seen = true;
					break;
				}
			}
			//if not duplicate, then add state to open queue
			if (!already_seen)
			{
				report.node_count++;
				//add to back of open
				open.push_back(temp_state);
			}
			else
				report.rep_node_count++;
			valid_states.pop();
		}
	}
	report.fringe_node = open.size();
	return report;
} //SearchStat bfs(State &initial_state)

/* Function executes depth first search algorithm on an inital state.
 *  Due to sokoban puzzles having infinite depth, an explored list is
 *  used to prevent infinite loops.
 * 
 * Preconditions: Takes in a State object for initial state of level
 * Postconditions: Returns a SearchStat object for search results stats
 */
SearchStat dfs(State &initial_state)
{
	std::deque<State> open;
	std::vector<State> closed;
	SearchStat report;
	report.rep_node_count = 0;
	report.fringe_node = 0;
	report.explored_count = 1;//will be replaced, just to stop cout spam
	report.node_count = 1;
	report.node.state_str = "NULL";
	State current_state;
	
	//push first state into queue
	open.push_back(initial_state);
	while (!open.empty())
	{
		//take N from OPEN
		current_state = open.front();
		open.pop_front();
		//push N onto CLOSED
		closed.push_back(current_state);
		
		//print out in case a long time is taken and wondering if it froze
		if ((closed.size() % 5000) == 0)
			std::cout << "...explored "<< closed.size() <<" nodes..."<<std::endl;
			
		//if found, set report node to current node, set explored count to closed list size
		if (is_goal(current_state))
		{
			report.node = current_state;
			report.explored_count = closed.size();
			open.pop_front();
			break;
		}
		
		//generate valid states
		std::queue<State> valid_states = gen_valid_states(current_state);
		std::deque<State>::iterator it;
		std::vector<State>::iterator itr;
		std::deque<State> temp_open;
		
		//while queue is not empty of states
		while (!valid_states.empty())
		{
			bool already_seen = false;
			State temp_state = valid_states.front();
			//check if state has already been seen on open list
			for (it = open.begin(); it != open.end(); it++)
			{
				if (it->state_str == temp_state.state_str)
				{
					already_seen = true;
					break;
				}
			}
			//check if state has already been seen on closed list
			for (itr = closed.begin(); itr != closed.end(); itr++)
			{
				if (itr->state_str == temp_state.state_str)
				{
					already_seen = true;
					break;
				}
			}
			//if not duplicate, then add state to open queue
			if (!already_seen)
			{
				report.node_count++;
				//uses a temporary queue to reverse the node order
				temp_open.push_front(temp_state);
			}
			else
				report.rep_node_count++;
			valid_states.pop();
		}
		//reverses the reversed queue to keep left to right searches in the dfs
		while (!temp_open.empty())
		{
			State temp_state = temp_open.front();
			open.push_front(temp_state);
			temp_open.pop_front();
		}
	}
	report.fringe_node = open.size();
	return report;
} //SearchStat dfs(State &initial_state)

/* Function executes uniform cost search algorithm on an inital state.
 *  Due to sokoban puzzles having infinite depth, an explored list is
 *  used to prevent infinite loops.
 * 
 * Preconditions: Takes in a State object for initial state of level
 * Postconditions: Returns a SearchStat object for search results stats
 */
SearchStat ucs(State &initial_state)
{
	std::deque<State> open;
	std::vector<State> closed;
	SearchStat report;
	report.rep_node_count = 0;
	report.fringe_node = 0;
	report.explored_count = 1;//will be replaced, just to stop cout spam
	report.node_count = 1;
	report.node.state_str = "NULL";
	State current_state;
	
	//push first state into queue
	open.push_back(initial_state);
	while (!open.empty())
	{
		//take N from OPEN
		current_state = open.front();
		open.pop_front();
		//push N onto CLOSED
		closed.push_back(current_state);
		
		//print out in case a long time is taken and wondering if it froze
		if ((closed.size() % 5000) == 0)
			std::cout << "...explored "<< closed.size() <<" nodes..."<<std::endl;
			
		//if found, set report node to current node, set explored count to closed list size
		if (is_goal(current_state))
		{
			report.node = current_state;
			report.explored_count = closed.size();
			open.pop_front();
			break;
		}
		
		//generate valid states
		std::queue<State> valid_states = gen_valid_states(current_state, UCS);
		std::deque<State>::iterator it;
		std::vector<State>::iterator itr;
		
		//while queue is not empty of states
		while (!valid_states.empty())
		{
			bool already_seen = false;
			bool inserted = false;
			State temp_state = valid_states.front();
			//check if state has already been seen on open list
			for (it = open.begin(); it != open.end(); it++)
			{
				if (it->state_str == temp_state.state_str)
				{
					already_seen = true;
					break;
				}
			}
			//check if state has already been seen on closed list
			for (itr = closed.begin(); itr != closed.end(); itr++)
			{
				if (itr->state_str == temp_state.state_str)
				{
					already_seen = true;
					break;
				}
			}
			//if not duplicate, then add state to open queue
			if (!already_seen)
			{
				report.node_count++;
				//add to beginning of node with greater total cost
				//used to maintain generated node order
				for (it = open.begin(); it != open.end(); it++)
				{
					//priority of lowest total cost first
					if (it->total_cost > temp_state.total_cost)
					{
						open.insert(it, temp_state);
						inserted = true;
						break;
					}
				}
				//if no node has greater priority, push back
				if (!inserted)
					open.push_back(temp_state);
			}
			else
				report.rep_node_count++;
			valid_states.pop();
		}
	}
	report.fringe_node = open.size();
	return report;
} //SearchStat ucs(State &initial_state)

/* Function executes greedy best first search algorithm on an inital
 *  state with a given heuristics function.
 *  Due to sokoban puzzles having infinite depth, an explored list is
 *  used to prevent infinite loops.  
 * 
 * Preconditions: Takes in a State object for initial state of level, and
 *  and integer representing which heuristics function to use
 * Postconditions: Returns a SearchStat object for search results stats
 */
SearchStat gbfs(State &initial_state, int hfchoice)
{
	std::deque<State> open;
	std::vector<State> closed;
	SearchStat report;
	report.rep_node_count = 0;
	report.fringe_node = 0;
	report.explored_count = 1;//will be replaced, just to stop cout spam
	report.node_count = 1;
	report.node.state_str = "NULL";
	State current_state;
	
	//push first state into queue
	open.push_back(initial_state);
	while (!open.empty())
	{
		//take N from OPEN
		current_state = open.front();
		open.pop_front();
		//push N onto CLOSED
		closed.push_back(current_state);
		
		//print out in case a long time is taken and wondering if it froze
		if ((closed.size() % 5000) == 0)
			std::cout << "...explored "<< closed.size() <<" nodes..."<<std::endl;
			
		//if found, set report node to current node, set explored count to closed list size
		if (is_goal(current_state))
		{
			report.node = current_state;
			report.explored_count = closed.size();
			open.pop_front();
			break;
		}
		
		//generate valid states
		std::queue<State> valid_states;
		valid_states = gen_valid_states(current_state, hfchoice);
		std::deque<State>::iterator it;
		std::vector<State>::iterator itr;
		
		//while queue is not empty of states
		while (!valid_states.empty())
		{
			bool already_seen = false;
			bool inserted = false;
			State temp_state = valid_states.front();
			//check if state has already been seen on open list
			for (it = open.begin(); it != open.end(); it++)
			{
				if (it->state_str == temp_state.state_str)
				{
					already_seen = true;
					break;
				}
			}
			//check if state has already been seen on closed list
			for (itr = closed.begin(); itr != closed.end(); itr++)
			{
				if (itr->state_str == temp_state.state_str)
				{
					already_seen = true;
					break;
				}
			}
			//if not duplicate, then add state to open queue
			if (!already_seen)
			{
				report.node_count++;
				//add to beginning of node with greater total cost
				//used to maintain generated node order
				for (it = open.begin(); it != open.end(); it++)
				{
					//priority of lowest hscore first
					if (it->hscore > temp_state.hscore)
					{
						open.insert(it, temp_state);
						inserted = true;
						break;
					}
				}
				//if no node has greater priority, push back
				if (!inserted)
					open.push_back(temp_state);
			}
			else
				report.rep_node_count++;
			valid_states.pop();
		}
	}
	report.fringe_node = open.size();
	return report;
} //SearchStat gbfs(State &initial_state, int hfchoice)

/* Function executes A* search search algorithm on an inital state with
 *  a given heuristics function.
 *  Due to sokoban puzzles having infinite depth, an explored list is
 *  used to prevent infinite loops.  
 * 
 * Preconditions: Takes in a State object for initial state of level, and
 *  and integer representing which heuristics function to use
 * Postconditions: Returns a SearchStat object for search results stats
 */
SearchStat as(State &initial_state, int hfchoice)
{
	std::deque<State> open;
	std::vector<State> closed;
	SearchStat report;
	report.rep_node_count = 0;
	report.fringe_node = 0;
	report.explored_count = 1;//will be replaced, just to stop cout spam
	report.node_count = 1;
	report.node.state_str = "NULL";
	State current_state;
	
	//push first state into queue
	open.push_back(initial_state);
	while (!open.empty())
	{
		//take N from OPEN
		current_state = open.front();
		open.pop_front();
		//push N onto CLOSED
		closed.push_back(current_state);
		
		//print out in case a long time is taken and wondering if it froze
		if ((closed.size() % 5000) == 0)
			std::cout << "...explored "<< closed.size() <<" nodes..."<<std::endl;
			
		//if found, set report node to current node, set explored count to closed list size
		if (is_goal(current_state))
		{
			report.node = current_state;
			report.explored_count = closed.size();
			open.pop_front();
			break;
		}
				
		//generate valid states
		std::queue<State> valid_states;
		valid_states = gen_valid_states(current_state, hfchoice);
		std::deque<State>::iterator it;
		std::vector<State>::iterator itr;
		
		//while queue is not empty of states
		while (!valid_states.empty())
		{
			bool already_seen = false;
			bool inserted = false;
			State temp_state = valid_states.front();
			//check if state has already been seen on open list
			for (it = open.begin(); it != open.end(); it++)
			{
				if (it->state_str == temp_state.state_str)
				{
					already_seen = true;
					break;
				}
			}
			//check if state has already been seen on closed list
			for (itr = closed.begin(); itr != closed.end(); itr++)
			{
				if (itr->state_str == temp_state.state_str)
				{
					already_seen = true;
					break;
				}
			}
			//if not duplicate, then add state to open queue
			if (!already_seen)
			{
				report.node_count++;
				//add to beginning of node with greater total cost
				//used to maintain generated node order
				for (it = open.begin(); it != open.end(); it++)
				{
					//priority of lowest hscore first
					if (it->hscore > temp_state.hscore)
					{
						open.insert(it, temp_state);
						inserted = true;
						break;
					}
				}
				//if no node has greater priority, push back
				if (!inserted)
					open.push_back(temp_state);
			}
			else
				report.rep_node_count++;
			valid_states.pop();
		}
	}
	report.fringe_node = open.size();
	return report;
} //SearchStat as(State &initial_state, int hfchoice)

/* Function used to execute a search algorithm on a given initial state.
 *  Reports back search results.
 * 
 * Preconditions: Takes in a state object and an int representing search algo
 * Postconditions:  Executes search algo and prints search stats.
 */
void choose_search(State &init_state, int search_choice)
{
	timeval start, end;
	long sec, microsec;
	int level_size;
	SearchStat final_stat;
	std::string user_choice;
	
	switch (search_choice)
	{
		case BFS:
			std::cout << "BREADTH FIRST SEARCH:" << std::endl;
			gettimeofday(&start, NULL);
			final_stat = bfs(init_state);
			gettimeofday(&end, NULL);
			break;
			
		case DFS:
			std::cout << "DEPTH FIRST SEARCH:" << std::endl;
			gettimeofday(&start, NULL);
			final_stat = dfs(init_state);
			gettimeofday(&end, NULL);
			break;
			
		case UCS:
			std::cout << "UNIFORM COST SEARCH:" << std::endl;
			gettimeofday(&start, NULL);
			final_stat = ucs(init_state);
			gettimeofday(&end, NULL);
			break;
			
		case GBFSH1:
			std::cout << "GREEDY BEST FIRST SEARCH, HEURISTICS FUNCTION 1:" << std::endl;
			gettimeofday(&start, NULL);
			final_stat = gbfs(init_state, GBFSH1);
			gettimeofday(&end, NULL);
			break;
			
		case GBFSH2:
			std::cout << "GREEDY BEST FIRST SEARCH, HEURISTICS FUNCTION 2:" << std::endl;
			gettimeofday(&start, NULL);
			final_stat = gbfs(init_state, GBFSH2);
			gettimeofday(&end, NULL);
			break;
			
		case ASH1:
			std::cout << "A* SEARCH, HEURISTICS FUNCTION 1:" << std::endl;
			gettimeofday(&start, NULL);
			final_stat = as(init_state, ASH1);
			gettimeofday(&end, NULL);
			break;
			
		case ASH2:
			std::cout << "A* SEARCH, HEURISTICS FUNCTION 2:" << std::endl;
			gettimeofday(&start, NULL);
			final_stat = as(init_state, ASH2);
			gettimeofday(&end, NULL);
			break;
			
		default:
			std::cout << "Unrecognized choice" << std::endl;
	}

	//substring used to remove ending ', ' in string
	std::cout << "  Solution: " << std::endl;
	std::cout << "    "
		<< final_stat.node.move_list.substr(0,(final_stat.node.move_list.size()-2))
		<< std::endl;
	std::cout << "    # of nodes generated: ";
	std::cout << final_stat.node_count<<std::endl;
	std::cout << "    # of duplicate states generated: ";
	std::cout << final_stat.rep_node_count<<std::endl;
	std::cout << "    # of fringe nodes when termination occured: ";
	std::cout << final_stat.fringe_node<<std::endl;
	std::cout << "    # of explored nodes: ";
	std::cout << final_stat.explored_count<<std::endl;
	//report search algorithm runtime
	std::cout << "  Actual run time: ";
	sec = end.tv_sec - start.tv_sec;
	microsec = end.tv_usec - start.tv_usec;
	std::cout << (sec + (microsec/1000000.0))<< " seconds" << std::endl;
	
} //bool choose_search(State &init_state, int search_choice)

int main(int argc, char** argv)
{	
	int level_size;
	bool repeat = true;
	std::string usr_input;
	std::ifstream fs;
	std::string line;
	std::string input_level = "";

	//checks if argument exists, can't input level if no txt file
	if (argc != 2)
	{
		std::cerr << "  usage: " << argv[0] << "<sokoban_level>.txt"
			<< std::endl;
		return 0;
	}
	
	//opens sokoban level txt file and store as string
	fs.open (argv[1]);
	if (!fs)
	{
		std::cerr << "  error opening file " << argv[1]
			<< std::endl;
		return 0;
	}
	
	//get size of array from first line of input level
	std::getline(fs, line, '\n');
	level_size = atoi(line.c_str());

	//append lines to string
	while (std::getline(fs, line))
	{
		input_level.append(line) += "\n";
	}
	fs.close();
	
	State init_state;
	init_state.state_str = input_level;
	init_state.move_list = "";
	init_state.moves = init_state.pushes =
	init_state.total_cost = init_state.depth =
	init_state.hscore = 0;
	
	std::cout << "Sokoban level input:" << std::endl;
	std::cout << level_size << std::endl;
	std::cout << init_state.state_str;
	
	//while loop used to repeat search algorithms
	while (repeat)
	{
		bool loop = true, valid_input = true;
		
		//checks for valid search algorithm choice
		while (loop)
		{
			//asks user to select search choice
			std::cout << "\nMenu:\n"
				<< "  1) Breadth first search\n"
				<< "  2) Depth first search\n"
				<< "  3) Uniform cost search\n"
				<< "  4) Greedy best first search\n"
				<< "  5) A* search"
			<< std::endl;
			
			std::cin >> usr_input;
			if (usr_input == "1")
			{
				choose_search(init_state, BFS);
				valid_input = true;
				loop = false;
			}
			else if (usr_input == "2")
			{
				choose_search(init_state, DFS);
				valid_input = true;
				loop = false;
			}
			else if (usr_input == "3")
			{
				choose_search(init_state, UCS);
				valid_input = true;
				loop = false;
			}
			else if (usr_input == "4")
			{
				std::cout << "  Choose heuristics function 1 or 2 for Greedy Best First Search: ";
				std::cin >> usr_input;
				if (usr_input == "1")
				{
					choose_search(init_state, GBFSH1);
					valid_input = true;
					loop = false;
				}
				else if (usr_input == "2")
				{
					choose_search(init_state, GBFSH2);
					valid_input = true;
					loop = false;
				}
				else
				{
					std::cout << "Invalid heuristics choice." << std::endl;
					valid_input = false;
				}
			}
			else if (usr_input == "5")
			{
				std::cout << "  Choose heuristics function 1 or 2: ";
				std::cin >> usr_input;
				if (usr_input == "1")
				{
					choose_search(init_state, ASH1);
					valid_input = true;
					loop = false;
				}
				else if (usr_input == "2")
				{
					choose_search(init_state, ASH2);
					valid_input = true;
					loop = false;
				}
				else
				{
					std::cout << "Invalid heuristics choice." << std::endl;
					valid_input = false;
				}
			}
			else
			{
				std::cout << "Invalid choice.  Please choose again." << std::endl;
				valid_input = false;
			}
		}
		
		//while loop used for user to choose a valid choice to repeat or not
		while (valid_input)
		{
			std::cout << "Choose another search algorithm?[y/n]: ";
			std::cin >> usr_input;
			//a valid input sets valid_input to false and breaks loop
			if (usr_input == "y")
			{
				valid_input = false;
				repeat = true;
			}
			else if (usr_input == "Y")
			{
				valid_input = false;
				repeat = true;
			}
			else if (usr_input == "n")
			{
				valid_input = false;
				repeat = false;
			}
			else if (usr_input == "N")
			{
				valid_input = false;
				repeat = false;
			}
			else
				std::cout << "Invalid choice.  ";
		}
	}
	
	return 0;
} //int main(int argc, char** argv)
