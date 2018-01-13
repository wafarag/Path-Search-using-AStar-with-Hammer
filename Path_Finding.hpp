/*
 * A* Algorithm
 * The robot holds a hammer that can use it once
 *
 * Defining Path Finding functions
 *
 * Created on: Jan 7, 2018
 *      Author: Wael Farag
 */

#include "Cell.hpp"
#include <cmath>


// The Grid is our Search Space - 2D array
vector<vector<Cell*> > Grid(N_COLS, vector<Cell*>(N_RAWS)); // 2D array

// Define Open and Closed Lists, as well as the Solution Path List
vector<Cell*> openList;
vector<Cell*> closedList;
vector<Cell*> pathList;

// Define the positions of Start and End Points
Cell *StartPos;
Cell *EndPos;


// define a container to hold the best solution
Cell *Best_Solution;
vector<Cell*> Best_Solution_Path;

// Function to delete an element from any List (Open, Closed ... etc)
vector<Cell*> removeFromList(vector<Cell*> List, Cell* Element_Cell)
{
    for (unsigned int i = 0; i <= List.size(); i++)
        {
            if ((List[i]->x == Element_Cell->x) and (List[i]->y == Element_Cell->y))
                {
                    List.erase(List.begin() + i );
                }
        }
    return List;
}


// Function to delete an element from any List (Open, Closed ... etc)
vector<Wall> removeFromWalls(vector<Wall> WallsList, Wall Element_Wall)
{
    for (unsigned int i = 0; i <= WallsList.size(); i++)
        {
            if ((WallsList[i].x == Element_Wall.x) and (WallsList[i].y == Element_Wall.y))
                {
                    WallsList.erase(WallsList.begin() + i );
                }
        }
    return WallsList;
}



// Function to Calculate the Heuristic Value "H" for a Cell - based on Euclidean Distance
float Euclidean_Dist_Heuristic(Cell* First_Cell, Cell* Second_Cell)
{
    float dist = (float) sqrt((First_Cell->x - Second_Cell->x)*(First_Cell->x - Second_Cell->x) +
                 (First_Cell->y - Second_Cell->y)*(First_Cell->y - Second_Cell->y));

    return dist;
}

// Function to Calculate the Heuristic Value "H" for a Cell - based on Manhattan Distance
float Manhattan_Dist_Heuristic(Cell* First_Cell, Cell* Second_Cell)
{
    float dist = (float)(fabs(First_Cell->x - Second_Cell->x) + fabs(First_Cell->y - Second_Cell->y));

    return dist;
}

// Function to check if an Element is included in a Vector (List) or not
bool Includes(vector<Cell*> List, Cell* Element_Cell)
{
    bool flag = false;
    for (unsigned int i = 0; i < List.size(); i++)
        {
            if ((List[i]->x == Element_Cell->x) and (List[i]->y == Element_Cell->y))
                {
                    flag = true;
                }
        }
    return flag;
}

// Set up the search Grid
void Initialize_Grid()
{
    // Initialize the Grid with Cell Coordinates
    for (unsigned int i = 0; i < N_COLS; i++)
    {
        for (unsigned int j = 0; j < N_RAWS; j++)
            {
                Cell *C = new Cell(i,j);
                Grid[i][j] = C;
            }
    }
    // Find  all the neighbors for all Grid Cells
    for (unsigned int i = 0; i < N_COLS; i++)
    {
        for (unsigned int j = 0; j < N_RAWS; j++)
            {
                Grid[i][j]->Find_Neighbors(Grid);
            }
    }
}


// Function to check if any cell is a Wall or not
bool Is_Cell_Wall( vector<Wall> Walls, unsigned int x_pos, unsigned int y_pos)
{
    bool State = false;

    for (unsigned int i =0; i < Walls.size(); i++)
    {
        if ( (x_pos == Walls[i].x) and (y_pos == Walls[i].y))
        {
            State = true;
        }
    }

    return State;
}


// The Core of the A* Algorithm
void Search_Path()
{
    bool Solution_Found_All = false;

    // Define a vector container to hold the solutions that will found
    //vector<Cell*> Solutions;
    // Define a temporary container to hold the Map Walls coordinates
    vector<Wall> Temp_MAP_Walls = MAP_Walls;

    // Define a container for the current Robot Position
    Cell *CurrentPos;

    Best_Solution = new Cell(0,0);
    Best_Solution->G = 10000.0;      // Any big value

    for (unsigned int k=0; k < (MAP_Walls.size()+1); k++)
    {
        bool Solution_Found_k = false;

        //CurrentPos.push_back(new Cell(0,0));
        // start to remove a single wall each iteration and search for solution
        if ( k != 0)
        {
            Temp_MAP_Walls = removeFromWalls(Temp_MAP_Walls, Temp_MAP_Walls[k-1]);
        }

        // Clear Lists to start searching.
        openList.clear();
        closedList.clear();

        // openList starts with the beginning only
        openList.push_back(StartPos);
        // Loop till all the cells in the openList are checked
        while (openList.size() > 0)
        {
            unsigned int index_Lowest_F = 0;
            //CurrentPos.push_back(new Cell(0,0));
            for (unsigned int i = 0; i < openList.size(); i++)
            {
                if (openList[i]->F() < openList[index_Lowest_F]->F())
                {
                    index_Lowest_F = i;
                }
            }
            CurrentPos = openList[index_Lowest_F]; // move the Robot to this position with lowest F

            // Check if the Robot is already reached the final position
            if ((CurrentPos->x == EndPos->x) and (CurrentPos->y == EndPos->y))
            {
                Solution_Found_k   = true;
                Solution_Found_All = true;
                break;
            }

            // Best position (Robot) moves from openList to closedList
            openList = removeFromList(openList, CurrentPos);
            closedList.push_back(CurrentPos);

            // check all the neighboring cells
            CurrentPos->Find_Neighbors(Grid);
            vector<Cell*> neighbors = CurrentPos->neighbors;

            for (unsigned int i = 0; i < neighbors.size(); i++)
            {
                Cell* neighbor = neighbors[i];
                // Does the next Cell is a valid one?
                if (!Includes(closedList, neighbor) and !Is_Cell_Wall(Temp_MAP_Walls, neighbor->x, neighbor->y))
                {
                    float newG = CurrentPos->G + Euclidean_Dist_Heuristic(neighbor, CurrentPos);
                    //float newG = neighbor->G + Manhattan_Dist_Heuristic(neighbor, CurrentPos);

                    // Is this a better path than before?
                    bool newPath_found = false;

                    if (Includes(openList, neighbor))
                    {
                        if (newG < neighbor->G)
                        {
                            neighbor->G = newG;
                            newPath_found = true;
                        }
                    } else
                    {
                        neighbor->G = newG;
                        newPath_found = true;
                        openList.push_back(neighbor);
                    }

                    // Yes, it is a better path
                    if (newPath_found)
                    {
                        neighbor->H = Euclidean_Dist_Heuristic(neighbor, EndPos);
                        //neighbor->H = Manhattan_Dist_Heuristic(neighbor, EndPos);
                        neighbor->PreviousPos = CurrentPos;
                    }
                }
            }
        }
        // Store the found solution in each iteration
        //if (Solution_Found) Solutions.push_back(CurrentPos);
        //cout << " Solutions: " << Solutions[k]->G << endl;

        if ((Best_Solution->G > CurrentPos->G) and Solution_Found_k)
        {
            Cell* pathCell = CurrentPos;
            Best_Solution_Path.clear();
            Best_Solution_Path.push_back(pathCell);

            Best_Solution->G = CurrentPos->G;
            //
            while (pathCell->PreviousPos != NULL)
            {
                    Best_Solution_Path.push_back(pathCell->PreviousPos);
                    pathCell = pathCell->PreviousPos;
            }

        }

        //cout << " Best_Solution: " << Best_Solution->G << endl;
        // Build up the Walls again
        Temp_MAP_Walls = MAP_Walls;
    } // end of the k loop

    //
    if (Solution_Found_All)
    {
        cout << "Cheers, A Solution is Found !!" << endl;
        cout << "Total Cost G =" << Best_Solution->G << endl;
        //cout << "Current Position y =" << CurrentPos[index_Smallest_G]->y << endl;

    } else
    {
       cout << "Sorry, No Solution is found ..." << endl << endl;
       //cout << "Current Position x =" << CurrentPos->x << endl;
       //cout << "Current Position y =" << CurrentPos->y << endl;
    }
}

void Display_Grid()
{
    SetConsoleTextAttribute(console, WHITE);
    cout << endl << "-------------- Displaying the Grid -------------" << endl << endl;

    for (unsigned int i = 0; i < N_COLS; i++)
    {
        for (unsigned int j = 0; j < N_RAWS; j++)
        {
            Grid[i][j]->Draw(WHITE);
        }
    }
}

void Display_OpenList()
{
    SetConsoleTextAttribute(console, BLUE);
    cout << endl << "-------------- Displaying the Open Cell List -------------" << endl << endl;

    for (unsigned int i = 0; i < openList.size(); i++)
    {
        openList[i]->Draw(BLUE);
    }
}

void Display_ClosedList()
{
    SetConsoleTextAttribute(console, RED);
    cout << endl << "-------------- Displaying the Closed Cell List -------------" <<endl<<endl;

    for (unsigned int i = 0; i < closedList.size(); i++)
    {
        closedList[i]->Draw(RED);
    }
    cout << endl<< "Number of Searched Cells: " << closedList.size() << endl<<endl;
}

void Display_Path()
{
    SetConsoleTextAttribute(console, GREEN);
    cout << endl << "-------------- Displaying the Path Cell List -------------" <<endl<<endl;

    if (Best_Solution != NULL)
    {
       // Find the list of the path cells by working backwards
       Cell* pathCell = Best_Solution;
       pathList.push_back(pathCell);
       float pathCost = pathCell->G;

       while (pathCell->PreviousPos != NULL)
       {
           pathList.push_back(pathCell->PreviousPos);
           pathCell = pathCell->PreviousPos;
           //pathCost = pathCost + pathCell->G;
        }
        for (unsigned int i = 0; i < pathList.size(); i++)
        {
            pathList[i]->Draw(GREEN);
        }
        cout<<endl<<" Total Path Cost =" << pathCost << endl << endl;
    }
}


void Display_Best_Solution_Path()
{
    SetConsoleTextAttribute(console, GREEN);
    cout << endl << "-------------- Displaying the Path Cell List -------------" <<endl<<endl;

    if (Best_Solution != NULL)
    {
        for (unsigned int i = 0; i < Best_Solution_Path.size(); i++)
        {
            Best_Solution_Path[i]->Draw(GREEN);
        }
        cout<<endl<<" Total Path Cost =" << Best_Solution->G << endl << endl;
    }
}


// Draw the full MAP including the shortest path
void Draw_Final_Grid()
{
    SetConsoleTextAttribute(console, WHITE);
    cout << endl << "------------------------ Drawing the Final Grid -----------------------" <<endl;
    cout << endl << "Legend: RED -> 'Wall', GREEN -> 'Path Cell'. " <<endl;
    cout << endl << "Legend: BLUE -> Broken Wall, WHITE -> 'Normal Grid Cell'. " <<endl<<endl;

    for (unsigned int j = 0; j < N_RAWS; j++)
    {
        cout << string(20, ' ');

        for (unsigned int i = 0; i < N_COLS; i++)
        {
            if(!Is_Cell_Clear(Grid[i][j]->x, Grid[i][j]->y))
            {
                //if(Includes(pathList, Grid[i][j]))
                if(Includes(Best_Solution_Path, Grid[i][j]))
                {
                   SetConsoleTextAttribute(console, BLUE);
                   cout << "*";
                } else
                {
                    SetConsoleTextAttribute(console, RED);
                    cout << "*";
                }
            }
            else
            {
                //if(Includes(pathList, Grid[i][j]))
                if(Includes(Best_Solution_Path, Grid[i][j]))
                {
                    SetConsoleTextAttribute(console, GREEN);
                    cout << "*";
                }
                else
                {
                    SetConsoleTextAttribute(console, WHITE);
                    cout << "*";
                }
            }
        }
        cout<<endl;
    }
}












