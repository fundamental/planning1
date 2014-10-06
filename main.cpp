#include "sokoban_defs.h"

grid_t *background;
goals_t goals;
pos_t robot;
set<action_t> known_states;

vector<path_t*> paths;
paths_t unexplored_path;

//Adds the balls into the grid
grid_t mergeBalls(grid_t g, balls_t balls)
{
    grid_t grid(g.X, g.Y);
    for(int x=0; x<g.X; ++x)
        for(int y=0; y<g.Y; ++y)
            grid(x,y) = g(x,y);

    for(pos_t b:balls)
        grid(b.x,b.y) = 1;

    return grid;
}

//Returns the portion of grid g connected to p
grid_t connectivity(grid_t g, pos_t p)
{
    grid_t grid (g.X, g.Y);
    queue<pos_t> work;
    work.push(p);
    while(!work.empty())
    {
        pos_t p = work.front();
        work.pop();
        if(grid(p) || g(p))
            continue;
        grid(p) = 1;
        if(p.x != 0) //left
            work.push(pos_t(p.x-1, p.y));
        if(p.x != g.X-1) //right
            work.push(pos_t(p.x+1, p.y));
        if(p.y != 0) //down
            work.push(pos_t(p.x, p.y-1));
        if(p.y != g.Y-1) //up
            work.push(pos_t(p.x, p.y+1));
    }
    return grid;
}

//updates balls after a move (ball from 'prev' goes to 'next')
balls_t gsub(balls_t b, pos_t prev, pos_t next)
{
    balls_t res;
    for(auto bb:b)
        res.push_back(bb==prev ? next : bb);
    return res;
}

//Find out if the robot can get to any side of each box and push it into
//an open space on the other side
actions_t findActions(grid_t back_w_balls, grid_t robot_connect, balls_t balls)
{
    actions_t result;
    //Per ball try to move it up/down/left/right
    for(int i=0; i<(int)balls.size(); ++i) {
        auto b = balls[i];
        if(robot_connect(b.x+1, b.y, 0) && !back_w_balls(b.x-1, b.y, 1))//L
            result.push_back({L,{b.x+1,b.y},{b.x,b.y},gsub(balls,{b.x,b.y},{b.x-1,b.y})});
        if(robot_connect(b.x-1, b.y, 0) && !back_w_balls(b.x+1, b.y, 1))//R
            result.push_back({R,{b.x-1,b.y},{b.x,b.y},gsub(balls,{b.x,b.y},{b.x+1,b.y})});
        if(robot_connect(b.x, b.y+1, 0) && !back_w_balls(b.x, b.y-1, 1))//D
            result.push_back({U,{b.x,b.y+1},{b.x,b.y},gsub(balls,{b.x,b.y},{b.x,b.y-1})});
        if(robot_connect(b.x, b.y-1, 0) && !back_w_balls(b.x, b.y+1, 1))//U
            result.push_back({D,{b.x,b.y-1},{b.x,b.y},gsub(balls,{b.x,b.y},{b.x,b.y+1})});
    }

    return result;
}

void print(actions_t a)
{
    if(a.empty())
        printf("{NO ACTIONS}\n");
    for(auto aa: a)
    {
        printf("{Move Robot (%d,%d) -> (%d,%d)}\n", aa.pos.x, aa.pos.y, aa.robot.x, aa.robot.y);
    }
}

void print(paths_t p)
{
    if(p.empty())
        printf("{NO PATHS}\n");
    for(auto pp: p)
    {
        printf("{path gone %f remaining %f {Move Robot (%d,%d) -> (%d,%d)}}\n", pp.total_cost, pp.remaining, pp.act.pos.x, pp.act.pos.y, pp.act.robot.x, pp.act.robot.y);
    }
}

//Finds Euclidean distance (greedy) from balls to goals
float dist2(balls_t balls)
{
    assert(balls.size() == goals.size());
    vector<bool> v;
    for(auto bb:goals) {
        (void) bb;
        v.push_back(0);
    }

    //greedy fit
    float total = 0;
    for(auto x:balls) {
        float min = 99999;
        int   ind = -1;
        for(int i=0; i<(int)goals.size(); ++i) {
            float c = abs(x.x-goals[i].x) + abs(x.y-goals[i].y);
            if(v[i])
                continue;
            if(c < min) {
                min = c;
                ind = i;
            }
        }
        total += min;
        v[ind] = 1;
    }
    return total;
}

//Calculates robot move distance in connected grid spaces
int robotGridDist(grid_t g, pos_t a, pos_t b)
{
    //Return 1 to account for robot's push
    if(a == b)
        return 1;

    //setup for bfs
    queue<pos_t> toVisit;
    queue<int> dists;
    set<pos_t> visited;
    toVisit.push(a);
    dists.push(1);   //Also 1 to account for push

    //robot == 1 in grid, so ignore in first round
    bool firstPass = true;

    //performs BFS on grid
    while(!toVisit.empty())
    {
        pos_t p = toVisit.front();
        toVisit.pop();
        int distToP = dists.front();
        dists.pop();
        if((g(p) && !firstPass) || visited.count(p) != 0)
            continue;
        vector<pos_t> newPs;
        if(p.x != 0) //left
            newPs.push_back(pos_t(p.x-1, p.y));
        if(p.x != g.X-1) //right
            newPs.push_back(pos_t(p.x+1, p.y));
        if(p.y != 0) //down
            newPs.push_back(pos_t(p.x, p.y-1));
        if(p.y != g.Y-1) //up
            newPs.push_back(pos_t(p.x, p.y+1));
        visited.insert(p);
        for(int i=0; i < newPs.size(); ++i)
        {
            pos_t next = newPs.at(i);
            if(visited.count(next) == 0)
            {
                if(b == next)
                    return distToP + 1;
                toVisit.push(newPs.at(i));
                dists.push(distToP + 1);
            }
        }
        firstPass = false;
    }
    //something went wrong and the robot can't get to push location
    return 99999;
}

//Calculates robot move distance in connected grid spaces
string getCommands(grid_t g, pos_t a, pos_t b, dir_t d)
{
    string last_comm = "";

    switch(d){
        case U:
            last_comm = "U";
            break;
        case D:
            last_comm = "D";
            break;
        case L:
            last_comm = "L";
            break;
        case R:
            last_comm = "R";
            break;
    }

    if(a == b)
        return last_comm;

    //setup for bfs
    queue<pos_t> toVisit;
    queue<string> strings;
    set<pos_t> visited;
    toVisit.push(a);
    strings.push("");   

    //robot == 1 in grid, so ignore in first round
    bool firstPass = true;

    //performs BFS on grid
    while(!toVisit.empty())
    {
        pos_t p = toVisit.front();
        toVisit.pop();
        string stringToP = strings.front();
        strings.pop();
        if((g(p) && !firstPass) || visited.count(p) != 0)
            continue;
        vector<pos_t> newPs;
        vector<string> newStrings;
        if(p.x != 0) //left
            newPs.push_back(pos_t(p.x-1, p.y));
            newStrings.push_back(stringToP + "L");
        if(p.x != g.X-1) //right
            newPs.push_back(pos_t(p.x+1, p.y));
            newStrings.push_back(stringToP + "R");
        if(p.y != 0) //down
            newPs.push_back(pos_t(p.x, p.y-1));
            newStrings.push_back(stringToP + "U");
        if(p.y != g.Y-1) //up
            newPs.push_back(pos_t(p.x, p.y+1));
            newStrings.push_back(stringToP + "D");
        visited.insert(p);
        for(int i=0; i < newPs.size(); ++i)
        {
            pos_t next = newPs.at(i);
            if(visited.count(next) == 0)
            {
                if(b == next)
                    return newStrings.at(i) + last_comm;
                toVisit.push(newPs.at(i));
                strings.push(newStrings.at(i));
            }
        }
        firstPass = false;
    }
    //something went wrong and the robot can't get to push location
    return "_";
}

//for each goal, finds distance to closest ball using BFS
int ballsGridDist(balls_t balls){
    assert(balls.size() == goals.size());
    grid_t g = *background;
    int totalDistance = 0;

    //track which balls/goals are claimed
    vector<bool> bs;
    vector<bool> gs;
    for(auto bb:balls) {
        (void) bb;
        bs.push_back(0);
        gs.push_back(0);
    }

    //eliminate goal/ball pairs that are already accounted for
    for(int i=0; i<balls.size(); ++i){
        for(int j=0; j<goals.size(); ++j){
            if(balls[i] == goals[j]){
                bs[i] = 1;
                gs[j] = 1;
            }
        }
    }

    for(int i=0; i<goals.size(); ++i) {

        pos_t gg = goals[i];

        //if a ball is at goal, distance is 0
        if(gs[i])
            continue;

        //setup for BFS
        queue<pos_t> toVisit;
        queue<int> dists;
        set<pos_t> visited;
        toVisit.push(gg);
        dists.push(0);

        //performs BFS on grid
        while(!toVisit.empty())
        {
            pos_t p = toVisit.front();
            toVisit.pop();
            int distToP = dists.front();
            dists.pop();            
            if(g(p) || visited.count(p) != 0)
                continue;
            vector<pos_t> newPs;
            if(p.x != 0) //left
                newPs.push_back(pos_t(p.x-1, p.y));
            if(p.x != g.X-1) //right
                newPs.push_back(pos_t(p.x+1, p.y));
            if(p.y != 0) //down
                newPs.push_back(pos_t(p.x, p.y-1));
            if(p.y != g.Y-1) //up
                newPs.push_back(pos_t(p.x, p.y+1));
            visited.insert(p);
            bool foundBall = false;
            for(int i=0; i < newPs.size(); ++i)
            {
                pos_t next = newPs.at(i);
                if(visited.count(next) == 0)
                {
                    //grab distance to first unclaimed ball
                    for(int i=0; i<balls.size(); ++i){
                        if(next==balls[i] && !bs[i]){
                            foundBall = true;
                            bs[i] = 1;
                        }
                    }
                    if(foundBall){
                        totalDistance += distToP + 1;
                        break;
                    }

                    //no ball found for this square
                    toVisit.push(newPs.at(i));
                    dists.push(distToP + 1);
                }
            }
            if(foundBall)
                break;
        }
    }
    return totalDistance;
}

//check for corner stuck plus hall stuck
//NOTE: not all cases are covered, but this is sufficient
//for given puzzles.
bool isImpossible(grid_t g, balls_t balls)
{
    grid_t gg = mergeBalls(g, balls);
    for(auto b:balls) {
        bool isGoal = false;
        for(auto g:goals)
            isGoal |= g==b;
        if(isGoal)
            continue;
        if(g(b.x-1, b.y, 1) && g(b.x,b.y-1,1)) {
            //printf("impossible LU\n");
            return true;
        } else if(g(b.x-1, b.y, 1) && g(b.x,b.y+1,1)) {
            //printf("impossible LD\n");
            return true;
        } else if(g(b.x+1, b.y, 1) && g(b.x,b.y-1,1)) {
            //printf("impossible RU\n");
            return true;
        } else if(g(b.x+1, b.y, 1) && g(b.x,b.y+1,1)) {
            //printf("impossible RD\n");
            return true;
        }

        ////vertical case
        bool wallLU = g(b.x-1,b.y,1) && g(b.x-1, b.y-1,1);\
        if(wallLU && gg(b.x, b.y-1, 1))
            return true;

        ////horizontal case
        bool wallDL = g(b.x,b.y+1,1) && g(b.x-1, b.y+1,1);
        if(wallDL && gg(b.x-1, b.y,1))
            return true;
    }
    return false;
}

//Generate paths from a set of new actions
paths_t actionsToPaths(path_t &cur, actions_t act)
{
    paths_t result;
    grid_t starting_grid = mergeBalls(*background, cur.act.balls);
    for(auto a:act) {        
        int robotSteps = robotGridDist(starting_grid, cur.act.robot, a.pos);
        float robotCost = 10*robotSteps;
        float ballCosts = 60*ballsGridDist(a.balls);
        result.push_back({&cur, a, ballCosts + 99999*isImpossible(*background,a.balls),
            robotCost + cur.total_cost, cur.robot_steps + robotSteps, cur.depth+1});
    }
    return result;
}

//Add all paths that have not been explored and are not pending
paths_t &append(paths_t &a, const paths_t &b)
{    
    //XXX this should add paths if a lower cost route has been found
    for(auto bb:b) {
        if(bb.remaining > 99999)
            continue;
        bool doAdd = known_states.find(bb.act) == known_states.end();
        if(doAdd) {
            a.push_back(bb);
            known_states.insert(bb.act);
        }
    }
    return a;
}

//Generate starting path, with everything in default positions
path_t initialPath(pos_t robot, balls_t balls)
{
    return path_t{NULL, {NOT_A_DIRECTION, robot, robot, balls}, dist2(balls), 0, 0, 0};
}

//tick the algorithm
void explore(void)
{
    assert(!unexplored_path.empty());
    //find best path unexplored
    int ind=-1;
    double minCost = 1e99;
    for(int i=0; i<(int)unexplored_path.size(); ++i) {
        double cost = unexplored_path[i].total_cost+unexplored_path[i].remaining;
        if(cost < minCost) {
            minCost = cost;
            ind = i;
        }
    }
    assert(ind != -1);
    if(minCost > 99999)
        exit(1);

    //Explore path

    path_t* next_path = new path_t(unexplored_path[ind]);
    //memcpy(next_path, &unexplored_path[ind],sizeof(path_t));
    paths.push_back(next_path);
    //paths.push_back(unexplored_path[ind]);
    path_t &p = *next_path;
    unexplored_path.erase(unexplored_path.begin()+ind);
    //printf("-------------------------------------------------\n");
    //background->dump(p.act.robot, p.act.balls, goals);

    grid_t g1 = mergeBalls(*background, p.act.balls);
    grid_t g2 = connectivity(g1, p.act.robot);
    //g2.dump();

    actions_t a = findActions(g1, g2, p.act.balls);
    //print(a);
    static int iteration = 0;
    iteration++;
    append(unexplored_path, actionsToPaths(p, a));
    //printf("step: %d depth: %d space: %d cost: %f r_steps %d\n",iteration, p.depth, unexplored_path.size(), p.remaining, p.robot_steps);
}

//True if one of the explored paths results in a zero distance win
bool foundSolution(void)
{
    for(int i=0; i<(int)paths.size(); ++i)
        if(paths[i]->remaining == 0)
            return true;
    return false;
}

//NOTE: Assumes first line is "X_DIM,Y_DIM" (2 digits each), and each 
//successive line is one row of the level
void parse_level(char *levelFile)
{
    //opening file
    FILE * pFile;
    pFile = fopen(levelFile, "r");
    if(pFile == NULL) perror("Error opening level file");

    //reading width and height
    char levelWidth [4];
    char levelHeight [4];
    fgets(levelWidth, 3, pFile);
    getc(pFile);
    fgets(levelHeight, 3, pFile); 
    getc(pFile);
    int w = atoi(levelWidth);
    int h = atoi(levelHeight);

    //setting up grid
    background = new grid_t(w,h);
    auto &bg = *background;
    char nextChar[2];
    pos_t robot;
    balls_t balls;
    for(int y=0; y<h; y++){
        for(int x=0; x<=w; x++){
            fgets(nextChar, 2, pFile);
            switch(nextChar[0]){
                case '\n':
                    break;
                case 'X':
                    bg(x,y) = 1;
                    break;
                case ' ':
                    bg(x,y) = 0;
                    break;
                case 'R':
                    robot = pos_t{x,y};
                    bg(x,y) = 0;
                    break;
                case 'G':
                    goals.push_back({x,y});
                    bg(x,y) = 0;
                    break;
                case 'B':
                    balls.push_back({x,y});
                    bg(x,y) = 0;
                    break;
            }
            
        }
    }
    fclose(pFile);    
    //background->dump(robot, balls, goals);
    //printf("\nReady:\n\n");
    unexplored_path.push_back(initialPath(robot, balls));
}

//calculates the string of robot input leading to a solution
void printSolution(path_t* endState)
{
    int path_length = (int)endState->depth;
    path_t** solution = new path_t* [path_length+1];
    path_t* path_ptr = endState;
    for(int i=path_length; i>=0; --i)
    {
        //printf("i: %d\t Pointer: %p\t Prev: %p\t Depth: %d\n", i, path_ptr,path_ptr->prev,(int)path_ptr->depth);
        solution[i] = path_ptr;
        path_ptr = path_ptr->prev;
    }
    for(int i=0; i<path_length; ++i)
    {
        path_t p1 = *solution[i];
        path_t p2 = *solution[i+1];

        /*background->dump(p1.act.robot, p1.act.balls, goals);
        printf("P1_pos: (%d,%d); P1_robot: (%d,%d)\n",p1.act.pos.x,p1.act.pos.y,p1.act.robot.x,p1.act.robot.y);
        printf("\n\n");
        background->dump(p2.act.robot, p2.act.balls, goals);
        printf("P2_pos: (%d,%d); P2_robot: (%d,%d)\n",p2.act.pos.x,p2.act.pos.y,p2.act.robot.x,p2.act.robot.y);
        */

        grid_t filled_grid = mergeBalls(*background, p1.act.balls);
        string next_substr = getCommands(filled_grid, p1.act.robot, p2.act.pos, p2.act.direction);
        printf("%s", next_substr.c_str());
        //printf("\n========================\n");
    }
    printf("\n");

}

int main(int argc, char* argv[])
{
    if(argc < 2) {
        printf("Usage: %s level\n", argv[0]);
        return 1;
    }

    char levelFile [16];
    sprintf(levelFile, "level%s.txt", argv[1]);
    parse_level(levelFile);

    while(!unexplored_path.empty() && !foundSolution())
        explore();

    int ind = -1;
    bool solved = false;
    for(int i=0; i < paths.size(); ++i){
        if(paths[i]->remaining == 0){
            ind = i;
            solved = true;
        }
    }
    path_t* finalState = paths[ind];

    if(!solved)
        printf("No solution found.\n");
    else{
        printSolution(finalState);
    }

    for(auto p:paths)
        delete p;

    delete background;

    return 0;
}
