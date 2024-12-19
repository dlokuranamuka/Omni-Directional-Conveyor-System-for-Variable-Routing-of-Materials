#include <iostream>
#include <cmath>

// Function prototypes

// Initialize the C-space
// Cspace is a 2D array of integers that represents the configuration space of size rows x cols
int** initCSpace(int r, int c);

// Initialize the position of the C-space
// Position is a 1D array of integers that represents the position of the C-space
int* initPosition(int x, int y);

// Print the C-space
// Print the C-space to the console for visualization and debugging
void printCSpace(int **space);

// Detail the C-space
// Detail the C-space with the boundries of the map
// The boundries are set to 1 in the C-space
// The input arguments are the radius of the object and the coordinates of the boundries
void detailCSpace(int radius, int coord[2][2]);

// Bezier curve function
// Compute the Bezier curve for the path planning
// The curve cordinates are set to 2 in the C-space
// 3 control points are used to compute the Bezier curve
// The input arguments are the start and goal positions
// The 2nd control point is set to the center of the C-space
void bezierCurve(int xs, int ys, int xg, int yg);

// Cell identifiers
// Identify the cells that are occupied by the object
// The input arguments are the cell number, x and y coordinates, and the radius of the object
// Output is a boolean value that indicates if the cell is occupied
bool isCellOccupied(int cellNo ,int x, int y, int radius);

// Define C-space dimensions
const int length = 300;
const int width = 400;
const int rows = length / 10;
const int cols = width / 10;

// Initialize the C-space data structure
typedef struct
{
    int **space;
    int *position;
} cSpace;

// Object data structure
typedef struct
{
    int radius;
    int xGoal;
    int yGoal;
} object;

// Cell details data structure
// Contains the x and y coordinates of the centroid of the cell;
typedef struct
{
    int x;
    int y;
} cellDetail;

// Number of cells
const int noCell = 10;

// Radius of a cell
const int cellRadius = 5;

// Initialize the C-space 1 as a global variable
cSpace cSpace1;

// Initialize the details of the cells
cellDetail cellDetails[noCell] = {
  { 8, 30 },
  { 15, 35 },
  { 22, 30 },
  { 8, 20 },
  { 15, 25 },
  { 22, 20 },
  { 8, 10 },
  { 15, 15 },
  { 22, 10 },
  { 15, 5 },
};

using namespace std;

int main(void)
{
    // Initialize the C-space1
    cSpace1.space = initCSpace(rows, cols);
    if (cSpace1.space == NULL)
    {
        cout << "Memory allocation failed for C-space" << endl;
        return 1;
    }

    // Initialize the position of the C-space
    cSpace1.position = initPosition(0, 0);
    if (cSpace1.position == NULL)
    {
        cout << "Memory allocation failed for C-space position" << endl;
        return 1;
    }


    // Initializing the object types

    // Object 1
    object object1;
    object1.radius = 9;
    object1.xGoal = 0;
    object1.yGoal = 20;

    // Object 2
    object object2;
    object2.radius = 9;
    object2.xGoal = 29;
    object2.yGoal = 20;

    // Object 3
    object object3;
    object3.radius = 11;
    object3.xGoal = 0;
    object3.yGoal = 39;

    // Object 4
    object object4;
    object4.radius = 11;
    object4.xGoal = 29;
    object4.yGoal = 39;

    // Detailing C-space with boundries of the map
    // int coord1[2][2] = {{0,0},{0,cols-1}};
    // int coord2[2][2] = {{0,rows-1},{0,0}};
    // int coord3[2][2] = {{0,rows-1},{cols-1,cols-1}};
    // int coord4[2][2] = {{rows-1,rows-1},{0,cols-1}};
    // detailCSpace(object1.radius, coord1);
    // detailCSpace(object1.radius, coord2);
    // detailCSpace(object1.radius, coord3);
    // detailCSpace(object1.radius, coord4);

    // BezierCurve for object 1
    bezierCurve( 15, 0, object1.xGoal, object1.yGoal);

    // For cell 1, check if it is occupied by the object
    // if (isCellOccupied(0, rows/2, 0, object1.radius))
    // {
    //     cout << "Cell 1 is occupied" << endl;
    // }
    // else
    // {
    //     cout << "Cell 1 is not occupied" << endl;
    // }
    // for (int k = 0; k < noCell; ++k)
    // {
    //     for (int i = 0; i < rows; ++i) 
    //     {
    //         for (int j = 0; j < cols; ++j) 
    //         {
    //             if (pow((i - cellDetails[k].x), 2) + pow((j - cellDetails[k].y), 2) <= pow(cellRadius, 2)) 
    //             {
    //                 cSpace1.space[i][j] = k;
    //             }
    //         }
    //     }
    // }
    for (int k = 0; k < noCell; ++k)
    {
        if (isCellOccupied(k, 15, 5, object1.radius))
        {
            cout << "Cell " << k << " is occupied" << endl;
        }
        else
        {
            cout << "Cell " << k << " is not occupied" << endl;
        }
    }


    // Print the C-space
    printCSpace(cSpace1.space);


    // Free allocated memory
    for (int i = 0; i < rows; ++i)
    {
        free(cSpace1.space[i]);
    }
    free(cSpace1.space);
    free(cSpace1.position);
}

// Initialize the C-space
int** initCSpace(int r, int c)
{
    // Allocate memory for the C-space
    int **n = (int**)malloc(r * sizeof(int *));
    if (n == NULL)
    {
        return n;
    }
    for (int i = 0; i < r; i++)
    {
        n[i] = (int*)malloc(c * sizeof(int));
        if (n[i] == NULL)
        {
            for (int k = 0; k < i; ++k)
            {
                // Free allocated rows before returning
                free(n[k]);
            }
            free(n);
            return n;
        }
    }

    // Initialize the C-space with zeros
    for (int i = 0; i < r; i++)
    {
        for (int j = 0; j < c; j++)
        {
            n[i][j] = 0;
        }
    }

    return n;
}

// Initialize the position of the C-space
int* initPosition(int x, int y)
{
    int *n = (int*)malloc(2 * sizeof(int));
    if (n == NULL)
    {
        return n;
    }
    n[0] = x;
    n[1] = y;

    return n;
}

// Detail the C-space
void detailCSpace(int radius, int coord[2][2]) 
{

    if(((coord[0][1] - coord[0][0]) == 0) || ((coord[1][1] - coord[1][0]) == 0))
    {
        for (int i = 0; i < rows; ++i) 
        {
            for (int j = 0; j < cols; ++j) 
            {
                int k = coord[0][0];
                int l = coord[1][0];
                do
                {
                    do
                    {
                        if (pow((i - k), 2) + pow((j - l), 2) <= pow(radius-1, 2)) 
                        {
                            cSpace1.space[i][j] = 1;
                        }
                        ++l;
                    }
                    while (l <= coord[1][1]); 
                    l = coord[1][0];
                    ++k;
                }
                while (k <= coord[0][1]);
            }
        }
    }

    else
    {
        // The slope of the line
        float m = (float(coord[0][1]-coord[0][0]))/(float(coord[1][1]-coord[1][0]));

        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                int k = coord[0][0];
                int l = coord[1][0];
                do
                {
                    if (pow((i - k), 2) + pow((j - l), 2) <= pow(radius-1, 2)) 
                    {
                        cSpace1.space[i][j] = 1;
                    }
                    if (k < coord[0][1])
                    {
                        k++;
                        l = (1/m)*(k-coord[0][0])+coord[1][0];
                    }
                }     
                while (k < coord[0][1]);
            }
        }
    }
}

// Bezier curve function
void bezierCurve(int xs, int ys, int xg, int yg)
{
    // Control points
    int p0[2] = {xs, ys};
    int p1[2] = {cols/2, rows/2};
    int p2[2] = {xg, yg};

    // Computing the Bezier curve
    for (float t = 0; t <= 1; t += 0.01) 
    {
        int x = (1 - t) * (1 - t) * p0[0] + 2 * (1 - t) * t * p1[0] + t * t * p2[0];
        int y = (1 - t) * (1 - t) * p0[1] + 2 * (1 - t) * t * p1[1] + t * t * p2[1];
        if (x >= 0 && x < rows && y >= 0 && y < cols) 
        {
            // Mark Bezier path in C-space as 2
            cSpace1.space[x][y] = 2; 
        }
    }
}

// Print the C-space
void printCSpace(int **space)
{
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            cout << space[i][j] << " ";
        }
        cout << endl;
    }
}

// Cell identifiers
bool isCellOccupied(int cellNo ,int x, int y, int radius)
{
    for (int i = 0; i < rows; ++i) 
    {
        for (int j = 0; j < cols; ++j) 
        {
            if (pow((i - x), 2) + pow((j - y), 2) <= pow(radius, 2)) 
            {
                if (pow((i - cellDetails[cellNo].x), 2) + pow((j - cellDetails[cellNo].y), 2) <= pow(cellRadius, 2))
                {
                    return true;
                }
            }
        }
    }
    return false;
}