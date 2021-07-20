#include "PathSearch.h"

namespace ufl_cap4053
{

	namespace searches
	{
		void PathSearch::distance(searchNode* adj)
		{

			//tile adjacent to current tile
			double x1 = adj->location->getXCoordinate();
			double y1 = adj->location->getYCoordinate();

			//goal tile
			double goalX = goal->location->getXCoordinate();
			double goalY = goal->location->getYCoordinate();

			adj->hCost = sqrt(abs((2 * ((goalX - x1)))) + abs((2 * (goalY - y1)))); //straight line distance formula

			if (adj != start) //default start to gCost
			{
				adj->gCost = adj->location->getWeight() * (tileRadius * 2) + curr->gCost;
			}

			adj->fCost = adj->gCost + adj->hCost;
		}//end distance

		void PathSearch::buildSolution()
		{
			if (solution.size() != 0)//if a solution is already built
				return;

			searchNode* temp = goal;
			while (temp != nullptr)
			{
				solution.push_back(temp->location);
				temp = temp->parent;
			}

		}//end buildSolution

		void PathSearch::setNeighbors(TileMap* _tileMap)
		{
			for (auto it = searchMap.begin(); it != searchMap.end(); it++)
			{
				for (int x = -1; x <= 1; x++)
				{
					for (int y = -1; y <= 1; y++)
					{
						//Conditions:
						//if the coordinates are in bounds and are not the initial tile. (row < numRows && column < numColumns)
						//The tile is passible (weight != 0)
						if ((it->first.first + x < _tileMap->getRowCount()) && (it->first.second + y < _tileMap->getColumnCount()) && ((it->first.first + x >= 0) && (it->first.second + y >= 0)) && (_tileMap->getTile(it->first.first + x, it->first.second + y)->getWeight() != 0))
						{
							if (it->first.first + x != it->first.first || it->first.second + y != it->first.second)
							{
								if (areAdjacent(searchMap[{it->first.first, it->first.second}]->location, _tileMap->getTile(it->first.first + x, it->first.second + y)))
								{
									searchMap[{it->first.first, it->first.second}]->neighbors.push_back(searchMap[{it->first.first + x, it->first.second + y}]);
								}
							}
						}
					}
				}
			}
		}//end setNeighbors

		void PathSearch::outputSearchNode(searchNode* node) //Debug Function
		{
			node->location->setFill(4286513152);
			//std::cout << "Location: " << node->location->getRow() << "," << node->location->getColumn() << std::endl;

			//std::cout << "\nNeighbors:" << std::endl;
			for (unsigned int i = 0; i < node->neighbors.size(); i++)
			{
				//std::cout << "(" << node->neighbors[i]->location->getRow() << "," << node->neighbors[i]->location->getColumn() << ")" << std::endl;
				node->neighbors[i]->location->setFill(4286513152);
			}

		}//end outputSearchNode

		bool PathSearch::areAdjacent(const Tile* lhs, const Tile* rhs)
		{
			int y2 = rhs->getColumn();
			int y1 = lhs->getColumn();
			int x2 = rhs->getRow();
			int x1 = lhs->getRow();
			int dx = y2 - y1;
			int dy = x2 - x1;

			if (dx <= 1 && dy <= 1)
			{
				//odd rows
				if (lhs->getRow() % 2 == 1)
				{
					//offsets
					if (dx == -1 && dy == -1)
					{
						return false;
					}
					else if (dx == -1 && dy == 1)
					{
						return false;
					}

				}
				//even rows
				else
				{
					//offsets
					if (dx == 1 && dy == -1)
					{
						return false;
					}
					else if (dx == 1 && dy == 1)
					{
						return false;
					}
				}
				return true;
			}
			return false;

		}//end areAdjacent

		PathSearch::PathSearch()
		{
			tileRadius = 0;
			start = nullptr;
			goal = nullptr;
			curr = nullptr;
			done = false;
		}//end constructor
		PathSearch::~PathSearch()
		{
			start = nullptr;
			goal = nullptr;
			curr = nullptr;

			//if the Queue isnt empty clear it
			while (!pQ.empty())
			{
				pQ.pop();
			}
			int numObjects = 0;
			//if the map's neighbor vectors aren't freed, free them.
			for (auto it = searchMap.begin(); it != searchMap.end(); it++)
			{
				it->second->neighbors.clear();
				it->second->neighbors.resize(0);
				delete it->second;
				numObjects++;
			}
			//std::cout << "Number of Nodes Destroyed: " << numObjects << std::endl;
			searchMap.clear();
			solution.clear();
			solution.resize(0);
		}//end destructor

		void PathSearch::load(TileMap* _tileMap)
		{
			//std::cout << "Load" << std::endl;
			int numObj = 0;
			for (int i = 0; i < _tileMap->getRowCount(); i++) //rows
			{
				for (int j = 0; j < _tileMap->getColumnCount(); j++) //columns
				{
					if (_tileMap->getTile(i, j)->getWeight() != 0) //if the tile is passible add it as a searchNode.
					{
						searchNode* newNode = new searchNode(_tileMap->getTile(i, j));
						searchMap[{i, j}] = newNode; //add the node to the map to search later.
						numObj++;
					}
				}
			}
			//set adjacent nodes
			//std::cout << "Number of Nodes Allocated: " << numObj << std::endl;
			tileRadius = _tileMap->getTileRadius();
			setNeighbors(_tileMap);

		}//end load

		void PathSearch::initialize(int startRow, int startCol, int goalRow, int goalCol)
		{
			//std::cout << "Initialize" << std::endl;

			done = false;
			curr = nullptr;
			start = searchMap[{startRow, startCol}];
			goal = searchMap[{goalRow, goalCol}];
			distance(start);
			searchMap[{startRow, startCol}]->visited = true;
			pQ.push({ start->fCost, start }); //push the starting node onto the queue.
		}//end initialize

		void PathSearch::update(long timeslice)
		{
			//std::cout << "Update" << std::endl;

			clock_t timeslice_ms = timeslice;
			clock_t now = (clock() / (CLOCKS_PER_SEC / 1000));
			if (!isDone())
			{
				if (timeslice > 0)
				{
					while (now != timeslice_ms)
					{
						if (done == true)
							return;

						algorithm();
						now = (clock() / ((CLOCKS_PER_SEC) / 1000));

					}
				}
				else
				{
					algorithm(); //iterate through the algorithm once
				}
			}

		}//end update
		void PathSearch::algorithm()
		{
			if (!isDone() || !pQ.empty()) //if queue is not empty run through the algorithm
			{
				curr = pQ.top().second;
				pQ.pop();

				if (curr == goal) // if a solution is found, we're done.
				{
					done = true;
					buildSolution();
					return;
				}

				for (unsigned int i = 0; i < curr->neighbors.size(); i++) //for each edge of the current Tile
				{
					searchNode* temp = curr->neighbors[i];

					double tempCost = curr->gCost + (tileRadius * 2) * curr->neighbors[i]->location->getWeight();
					if (temp->visited == true)
					{
						if (tempCost < temp->gCost)
						{
							//UPDATE HEURSTIC WEIGHT HERE
							temp->gCost = tempCost;
							temp->fCost = temp->gCost + temp->hCost * 1.2; //Heuristic Weight of 1.2
							temp->parent = curr;
							pQ.push({ temp->fCost, temp });
						}

					}
					else
					{
						double goalX = goal->location->getXCoordinate();
						double goalY = goal->location->getYCoordinate();
						double x = temp->location->getXCoordinate();
						double y = temp->location->getYCoordinate();
						double h = sqrt(((goalX - x) * (goalX - x)) + ((goalY - y) * (goalY - y)));
						temp->gCost = tempCost;
						temp->hCost = h;

						//UPDATE HEURISTIC WEIGHT HERE
						temp->fCost = temp->gCost + temp->hCost * 1.2; //Heuristic Weight of 1.2
						temp->parent = curr;
						temp->visited = true;
						pQ.push({ temp->fCost, temp });

						//set Markers to show nodes visited
						if (temp != goal)
						{
							temp->location->setMarker(4278255360);
						}
					}

				}

			}
		}//end algorithm

		void PathSearch::shutdown() //for the same map
		{

			//clear the map and unvisit all nodes

			while (!pQ.empty())
			{
				pQ.pop();
			}
			for (auto it = searchMap.begin(); it != searchMap.end(); it++)
			{
				it->second->visited = false;
			}
		}//end shutdown
		void PathSearch::unload() //prep for other maps
		{
			//reset the queue
			while (!pQ.empty())
			{
				pQ.pop();
			}

			int numObj = 0;
			//if the map's neighbor vectors aren't freed, free them.
			for (auto it = searchMap.begin(); it != searchMap.end(); it++)
			{
				it->second->neighbors.clear();
				it->second->neighbors.resize(0);
				delete it->second;
				numObj++;
			}

			//std::cout << "Number of Nodes Destroyed: " << numObj << std::endl;
			searchMap.clear();
			solution.clear();
			solution.resize(0);
			done = false;

		}//end unload

		bool PathSearch::isDone()const
		{
			// std::cout << "isDone" << std::endl;

			if (done) //if the goal is found and a solution is built
			{
				return true;
			}
			return false;
		}//end isDone

		std::vector<Tile const*> const PathSearch::getSolution() const
		{
			searchNode* temp = goal;
			while (temp->location != solution.back())
			{
				temp->location->addLineTo(temp->parent->location, 4294901760);
				temp = temp->parent;
			}
			return solution;
		}
	}
}  // close namespace ufl_cap4053::searches
