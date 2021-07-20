/*
	- Author: Sean Howard
	- File: Pathsearch.h
	- Description: Implements the A* algorithm on a Tile map
*/
#pragma once

#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
#include "../../Source/Framework/TileSystem/Tile.h" //Tile object
#include "../../Source/Framework/TileSystem/TileMap.h" //TileMap object
#include <vector>
#include <map>
#include <iostream>
#include <cmath>
#include <ctime>
#include <queue>


typedef std::pair<int, int> pairs; //for vertices (row, col)

namespace ufl_cap4053
{
	namespace searches
	{

		class PathSearch
		{
			struct searchNode
			{
				searchNode()
				{
					visited = false;
					location = nullptr;
					parent = nullptr;
					gCost = 0;
					hCost = 0;
					fCost = 0;
				}
				searchNode(Tile* _location)
				{
					visited = false;
					parent = nullptr;
					gCost = 0;
					hCost = 0;
					fCost = 0;
					location = _location;
				}
				searchNode* parent; //stores the parent 
				Tile* location;
				double gCost;
				double hCost; //heuristic
				double fCost; //gCost + hCost
				std::vector<searchNode*> neighbors;  //neighbors of the current location;
				bool visited; //defaulted to false
			};

		public:

			DLLEXPORT PathSearch(); // EX: DLLEXPORT required for public methods - see platform.h
			DLLEXPORT ~PathSearch(); //destructor
			DLLEXPORT void load(TileMap* _tileMap);
			DLLEXPORT void initialize(int startRow, int startCol, int goalRow, int goalCol);
			DLLEXPORT void update(long timeslice);
			DLLEXPORT void shutdown();
			DLLEXPORT void unload();
			DLLEXPORT bool isDone() const;
			DLLEXPORT std::vector<Tile const*> const getSolution() const;

		private:
			/* ========== Helper Functions ===========*/

			bool areAdjacent(const Tile* lhs, const Tile* rhs); //check for adjacency
			void outputSearchNode(searchNode* node); //cout function for searchNode
			void setNeighbors(TileMap* _tileMap); //sets adjacent tiles
			void buildSolution(); //solution builder
			void distance(searchNode* adj); // Calculates gCost, hCost, & fCost
			void algorithm(); //A* 


			/*======== Private Member Variables ========*/

			std::vector<Tile const*> solution; //solution
			std::map<pairs, searchNode*> searchMap;

			searchNode* start;
			searchNode* goal;
			searchNode* curr;
			std::priority_queue<std::pair<double, searchNode*>, std::vector<std::pair<double, searchNode*>>, std::greater<std::pair<double, searchNode*>>> pQ; //Priority Queue
			bool done;
			double tileRadius;

		};

	}

}  // close namespace ufl_cap4053::searches
