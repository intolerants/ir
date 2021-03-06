#include "stlastar.h" // See header for copyright and usage information

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0

using namespace std;

// Global data

// The world map

const int MAP_WIDTH = 330;
const int MAP_HEIGHT = 400;

int world_map[ MAP_WIDTH * MAP_HEIGHT ];

// map helper functions

int GetMap( int x, int y )
{
    if( x < 0 ||
        x >= MAP_WIDTH ||
         y < 0 ||
         y >= MAP_HEIGHT
      )
    {
        return 9;    
    }

    return world_map[(y*MAP_WIDTH)+x];
}



// Definitions

class MapSearchNode
{
public:
    int x;   // the (x,y) positions of the node
    int y;  
    
    MapSearchNode() { x = y = 0; }
    MapSearchNode( int px, int py ) { x=px; y=py; }

    float GoalDistanceEstimate( MapSearchNode &nodeGoal );
    bool IsGoal( MapSearchNode &nodeGoal );
    bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
    float GetCost( MapSearchNode &successor );
    bool IsSameState( MapSearchNode &rhs );

    void PrintNodeInfo(int result[][2], int steps); 


};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

    // same state in a maze search is simply when (x,y) are the same
    if( (x == rhs.x) &&
        (y == rhs.y) )
    {
        return true;
    }
    else
    {
        return false;
    }

}

void MapSearchNode::PrintNodeInfo(int result[][2], int steps)
{
    result[steps][0] = x;
    result[steps][1] = y;

    // char str[100];
    // sprintf( str, "Node position : (%d,%d)\n", x,y );

    // cout << world_map[y*MAP_WIDTH + x] << " ";

    // cout << str;

}

// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
    return fabsf(x - nodeGoal.x) + fabsf(y - nodeGoal.y);   
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

    if( (x == nodeGoal.x) &&
        (y == nodeGoal.y) )
    {
        return true;
    }

    return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

    int parent_x = -1; 
    int parent_y = -1; 

    if( parent_node )
    {
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }
    

    MapSearchNode NewNode;

    // push each possible move except allowing the search to go backwards

    if( (GetMap( x-1, y ) < 9) 
        && !((parent_x == x-1) && (parent_y == y))
      ) 
    {
        NewNode = MapSearchNode( x-1, y );
        astarsearch->AddSuccessor( NewNode );
    }   

    if( (GetMap( x, y-1 ) < 9) 
        && !((parent_x == x) && (parent_y == y-1))
      ) 
    {
        NewNode = MapSearchNode( x, y-1 );
        astarsearch->AddSuccessor( NewNode );
    }   

    if( (GetMap( x+1, y ) < 9)
        && !((parent_x == x+1) && (parent_y == y))
      ) 
    {
        NewNode = MapSearchNode( x+1, y );
        astarsearch->AddSuccessor( NewNode );
    }   

        
    if( (GetMap( x, y+1 ) < 9) 
        && !((parent_x == x) && (parent_y == y+1))
        )
    {
        NewNode = MapSearchNode( x, y+1 );
        astarsearch->AddSuccessor( NewNode );
    }   

    return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{
    return (float) GetMap( x, y );

}


// Main

int findPath( int* world_map2, int result[][2], int* start, int* end )
{
    // Create an instance of the search class...

    memcpy(world_map, world_map2, MAP_WIDTH*MAP_HEIGHT*sizeof(int));

    AStarSearch<MapSearchNode> astarsearch;

    unsigned int SearchCount = 0;

    const unsigned int NumSearches = 1;

    int steps;

    while(SearchCount < NumSearches)
    {

        // Create a start state
        MapSearchNode nodeStart;
        // cout << "start " << world_map[start[1]*MAP_WIDTH + start[0]] << endl;
        nodeStart.x = start[0];
        nodeStart.y = start[1];

        // Define the goal state
        MapSearchNode nodeEnd;
        // cout << "end " << world_map[end[1]*MAP_WIDTH + end[0]] << endl;
        nodeEnd.x = end[0];
        nodeEnd.y = end[1];
        
        // Set Start and goal states
        
        astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

        unsigned int SearchState;
        unsigned int SearchSteps = 0;

        do
        {
            SearchState = astarsearch.SearchStep();

            SearchSteps++;

    #if DEBUG_LISTS

            cout << "Steps:" << SearchSteps << "\n";

            int len = 0;

            cout << "Open:\n";
            MapSearchNode *p = astarsearch.GetOpenListStart();
            while( p )
            {
                len++;
    // #if !DEBUG_LIST_LENGTHS_ONLY            
                ((MapSearchNode *)p)->PrintNodeInfo();
    // #endif
                p = astarsearch.GetOpenListNext();
                
            }

            cout << "Open list has " << len << " nodes\n";

            len = 0;

            cout << "Closed:\n";
            p = astarsearch.GetClosedListStart();
            while( p )
            {
                len++;
    // #if !DEBUG_LIST_LENGTHS_ONLY            
                p->PrintNodeInfo();
    // #endif          
                p = astarsearch.GetClosedListNext();
            }

            cout << "Closed list has " << len << " nodes\n";
    #endif

        }
        while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

        if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
        {
            cout << "Search found goal state\n";

                MapSearchNode *node = astarsearch.GetSolutionStart();

    #if DISPLAY_SOLUTION
                cout << "Displaying solution\n";
    #endif
                steps = 0;

                node->PrintNodeInfo(result, steps);
                for( ;; )
                {
                    node = astarsearch.GetSolutionNext();

                    if( !node )
                    {
                        break;
                    }

                    steps ++;
                    node->PrintNodeInfo(result, steps);
                
                };

                cout << "Solution steps " << steps << endl;

                // Once you're done with the solution you can free the nodes up
                astarsearch.FreeSolutionNodes();

    
        }
        else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) 
        {
            cout << "Search terminated. Did not find goal state\n";
        
        }

        // Display the number of loops the search went through
        cout << "SearchSteps : " << SearchSteps << "\n";

        SearchCount ++;

        astarsearch.EnsureMemoryFreed();
    }

    std::ofstream output("outputResult.txt"); 
    output << steps << "\n";
    for (int i = 0; i < steps; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            output << result[i][j] << " "; // behaves like cout - cout is also a stream
        }
        output << "\n";
    } 

    // cout << "Result" << endl;
    // for (int i = 0; i < steps; i++)
    //     cout << result[i][0] << " " << result[i][1] << endl;
    
    return steps;
}