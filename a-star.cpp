#include <iostream>
#include <queue>
#include <string>
#include <cmath>
#include <ctime>
#include<windows.h>

using namespace std;

const int n=60; // horizontal size of the map
const int m=60; // vertical size size of the map
static int map[n][m]; //full map
static int closed_nodes_map[n][m]; // map of checked nodes
static int open_nodes_map[n][m]; // map of non checked nodes
static int dir_map[n][m]; // directions map
const int dir=8; // number of possible directions to go 

static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

class node
{
    int xPos;
    int yPos;
    
    // total distance already travelled to reach the node
    int level;
    
    // priority = level + remaining distance estimate
    int priority;  // smaller -> higher priority

    public:
        node(int xp, int yp, int d, int p) 
            {xPos = xp; yPos = yp; level = d; priority = p;}
    
        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const {return level;}
        int getPriority() const {return priority;}

        void updatePriority(const int & xDest, const int & yDest)
        {
             priority=level+estimate(xDest, yDest) * 10;
        }

        // calculating priority to going straight
        void nextLevel(const int & i)
        {
			level += (dir == 8?(i%2 == 0?10:14):10);
        }
        
        // Estimating remaining distance
        const int & estimate(const int & xDest, const int & yDest) const
        {
            static int xd, yd, d;
            xd = xDest - xPos;
            yd = yDest - yPos;         

             d = static_cast<int>(sqrt(xd*xd+yd*yd));

            return(d);
        }
};

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
  return a.getPriority() > b.getPriority();
}

// A-star "heart"
string pathFind( const int & xStart, const int & yStart, 
                 const int & xFinish, const int & yFinish )
{
    static priority_queue<node> pq[2]; // list of not checked nodes
    static int pqIndex;
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqIndex = 0;

    // resetting the node maps
    for(y = 0; y < m; y++)
    {
        for(x = 0;  x < n; x++)
        {
            closed_nodes_map[x][y] = 0;
            open_nodes_map[x][y] = 0;
        }
    }

    // create the start node
    n0 = new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqIndex].push(*n0);
    open_nodes_map[x][y] = n0->getPriority();

    while(!pq[pqIndex].empty())
    {
        // get the current node with the highest priority from the list of open nodes
        n0=new node( pq[pqIndex].top().getxPos(), pq[pqIndex].top().getyPos(), 
                     pq[pqIndex].top().getLevel(), pq[pqIndex].top().getPriority());

        x = n0->getxPos(); y = n0->getyPos();

        pq[pqIndex].pop(); // remove the node from the open list
        open_nodes_map[x][y] = 0;
 
        closed_nodes_map[x][y] = 1;

        if(x == xFinish && y == yFinish) 
        {
            // generate the path from finish to start
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j = dir_map[x][y];
                c = '0' + (j + dir/2)%dir;
                path = c + path;
                x += dx[j];
                y += dy[j];
            }

            delete n0;

            while(!pq[pqIndex].empty()) pq[pqIndex].pop();           
            return path;
        }

        for(i = 0; i < dir; i++)
        {
            xdx = x + dx[i]; ydy = y + dy[i];

            if(!(xdx < 0 || xdx > n-1 || ydy < 0 || ydy > m-1 || map[xdx][ydy] == 1 
                || closed_nodes_map[xdx][ydy] == 1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(), 
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqIndex].push(*m0);
                    
                    // marking parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update  priority
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    
                    // update the parent direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    while(!(pq[pqIndex].top().getxPos()==xdx && 
                           pq[pqIndex].top().getyPos()==ydy))
                    {                
                        pq[1-pqIndex].push(pq[pqIndex].top());
                        pq[pqIndex].pop();       
                    }
                    pq[pqIndex].pop();
                    
                    // empty the larger size pq to the smaller one
                    if(pq[pqIndex].size()>pq[1-pqIndex].size()) pqIndex=1-pqIndex;
                    while(!pq[pqIndex].empty())
                    {                
                        pq[1-pqIndex].push(pq[pqIndex].top());
                        pq[pqIndex].pop();       
                    }
                    pqIndex=1-pqIndex;
                    pq[pqIndex].push(*m0); // add the better node instead
                }
                else delete m0;
            }
        }
        delete n0;
    }
    return ""; // if no route found
}

int main()
{
    srand(time(NULL));
    int p=0, q=0;

    // create empty map
    for(int y = 0; y < m; y++)
    {
        for(int x = 0; x < n; x++) map[x][y]=0;
    }

    // filling map matrix with a '+' pattern
    for(int x = n/8; x < n*7/8; x++)
    {
        map[x][m/2]=1;
    }
    for(int y = m/8; y < m*7/8; y++)
    {
        map[n/2][y] = 1;
    }
    
	// random filling map with obstacles
    for(int x = 0; x < m; x++)
    {
    	for(int y = 0; y < n; y++)
    	{
		    	p = (rand()%3);
				if(p==0) map[x][y] = 1;	
    	}
    }
    // defining borders
    for(int i = 0; i < m; i++) map[i][0] = 1;
	for(int i = 0; i < m; i++) map[m-1][i] = 1;
	for(int i = 0; i < m; i++) map[0][i] = 1;
	for(int i = 0; i < m; i++) map[i][m-1] = 1;
    
    // random select start and finish locations
    int xA, yA, xB, yB;
    xA = (rand()%60);
    yA = (rand()%60);
    xB = (rand()%60);
    yB = (rand()%60);
    
    cout << "Map Size (X,Y): " << n << "," << m << endl;
    cout << "Start: "<< xA <<","<< yA << endl;
    cout << "Finish: " << xB <<","<< yB << endl;

    clock_t start = clock();
    string route=pathFind(xA, yA, xB, yB);
    if(route=="") cout<<"An empty route generated!"<<endl;
    clock_t end = clock();
    double time_elapsed = double(end - start);
    cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;
    cout<<"Route:"<<endl;
    cout<<route<<endl<<endl;

    // following and displaying the route on the map
    {
        int j; char c;
        int x = xA;
        int y = yA;
        map[x][y] = 2;
        for(int i = 0; i < route.length(); i++)
        {
            c = route.at(i);
            j = atoi(&c); 
            x = x + dx[j];
            y = y + dy[j];
            map[x][y] = 3;
        }
        map[x][y] = 4;
    
        // display map with the route
        for(int y = 0; y < m; y++)
        {
            for(int x = 0;x < n; x++)
                if(map[x][y]==0)
                    cout<<".";
                else if(map[x][y]==1)
                    cout<<"+"; //obstacle
                else if(map[x][y]==2)
                    cout<<"S"; //start
                else if(map[x][y]==3)
                    cout<<"R"; //route
                else if(map[x][y]==4)
                    cout<<"F"; //finish
            cout<<endl;
        }
    }

    system("pause");
}

