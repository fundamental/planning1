#include <cstdio>
#include <vector>
#include <queue>
#include <cassert>
#include <cstring>
#include <cstdlib>

using std::vector;
using std::queue;

struct pos_t;
struct state_t;
struct graph_t;
struct subspace_t;
struct action_t;
struct path_t;
struct grid_t;

//NaN state for initiail conditions
enum dir_t {L,R,U,D,NOT_A_DIRECTION};

struct pos_t
{
    pos_t() :pos_t(0,0) {}
    pos_t(int X, int Y) :x(X), y(Y) {}
    bool operator==(const pos_t &o) const {
        //printf("return(%dx%d==%dx%x:=%d)\n",x,y,o.x,o.y,o.x == x && o.y == y);
        return o.x == x && o.y == y;
    }
    int x,y;
};

typedef vector<pos_t> balls_t;


struct action_t {
    //Acts on
    dir_t direction;
    pos_t pos;
    //Results in
    pos_t robot;
    balls_t balls;
};

typedef vector<action_t> actions_t;

struct path_t {
    path_t *prev;
    action_t act;
    float    remaining;
    float    total_cost;
};

typedef vector<path_t> paths_t;


struct grid_t
{
    int *data;
    int X, Y;
    grid_t(int x, int y)
        :X(x), Y(y)
    {
        data = new int[x*y];
        memset(data, 0, x*y*sizeof(int));
    }

    grid_t(const grid_t &g)
        :X(g.X), Y(g.Y)
    {
        data = new int[X*Y];
        for(int i=0; i<X*Y; ++i)
            data[i] = g.data[i];
    }

    ~grid_t(void )
    {
        delete [] data;
    }

    int operator()(int x, int y, int outOfBounds)
    {
        if(0<=x && x<X && 0<=y && y<Y)
            return (*this)(x,y);
        return outOfBounds;
    }

    int &operator()(int x, int y)
    {
        assert(0<=x && x<X && 0<=y && y<Y);
        return data[x+y*X];
    }

    int &operator()(pos_t p)
    {
        return (*this)(p.x,p.y);
    }

    void dump(pos_t robot, vector<pos_t> balls, vector<pos_t> goals)
    {
        for(int y=0; y<Y; ++y) {
            for(int x=0; x<X; ++x) {
                bool isBall = false;
                bool isGoal = false;
                for(pos_t b:balls)
                    isBall |= b == pos_t(x,y);
                for(pos_t g:goals)
                    isGoal |= g == pos_t(x,y);

                if(robot == pos_t(x,y))
                    putchar('R');
                else if(isGoal && isBall)
                    putchar('*');
                else if(isGoal)
                    putchar('G');
                else if(isBall)
                    putchar('B');
                else if((*this)(x,y))
                    putchar('X');
                else
                    putchar('_');
            }
            putchar('\n');
        }
    }

    void dump(void)
    {
        for(int y=0; y<Y; ++y) {
            for(int x=0; x<X; ++x) {
                if((*this)(x,y))
                    putchar('X');
                else
                    putchar('_');
            }
            putchar('\n');
        }
    }
};

grid_t     mergeBalls(grid_t g);
grid_t     connectivity(grid_t,  pos_t);

state_t    convertToState();
graph_t    loadGraph();
subspace_t findSubspace();
actions_t  findActions(state_t);

grid_t *background;
vector<pos_t> balls;
vector<pos_t> goals;
pos_t robot;

paths_t paths;
paths_t unexplored_path;

grid_t mergeBalls(grid_t g, vector<pos_t> balls)
{
    grid_t grid(g.X, g.Y);
    for(int x=0; x<g.X; ++x)
        for(int y=0; y<g.Y; ++y)
            grid(x,y) = g(x,y);

    for(pos_t b:balls)
        grid(b.x,b.y) = 1;

    return grid;
}

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


balls_t gsub(balls_t b, pos_t prev, pos_t next)
{
    balls_t res;
    for(auto bb:b)
        res.push_back(bb==prev ? next : bb);
    return res;
}

actions_t findActions(grid_t background, grid_t connectivity, vector<pos_t> balls)
{
    actions_t result;
    //Per ball try to move it up/down/left/right
    for(int i=0; i<(int)balls.size(); ++i) {
        //printf("Testing Ball\n");
        auto b = balls[i];
        if(connectivity(b.x+1, b.y, 0) && !background(b.x-1, b.y, 1))//L
            result.push_back({L,{b.x+1,b.y},{b.x,b.y},gsub(balls,{b.x,b.y},{b.x-1,b.y})});
        if(connectivity(b.x+1, b.y, 0) && !background(b.x-1, b.y, 1))//R
            result.push_back({R,{b.x-1,b.y},{b.x,b.y},gsub(balls,{b.x,b.y},{b.x+1,b.y})});
        if(connectivity(b.x, b.y+1, 0) && !background(b.x, b.y-1, 1))//D
            result.push_back({D,{b.x,b.y+1},{b.x,b.y},gsub(balls,{b.x,b.y},{b.x,b.y-1})});
        if(connectivity(b.x, b.y-1, 0) && !background(b.x, b.y+1, 1))//U
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

float dist(vector<pos_t> a, vector<pos_t> b)
{
    assert(a.size() == b.size());
    float total = 0;
    for(int i=0; i<(int)a.size(); ++i)
        total += abs(a[i].x-b[i].x) + abs(a[i].y-b[i].y);
    return total;
}

paths_t actionsToPaths(path_t &cur, actions_t act)
{
    //XXX improve cost functions
    paths_t result;
    for(auto a:act) {
        float incremental_cost = 0.5;
        result.push_back({&cur, a, dist(a.balls,goals), incremental_cost+cur.total_cost});
    }

    return result;
}

paths_t &append(paths_t &a, const paths_t &b)
{
    //printf("Unexplored:\n");
    //print(a);
    //printf("Explored:\n");
    //print(paths);
    //Add all paths which have not been explored and are not pending
    //XXX this should add paths if a lower cost route has been found
    for(auto bb:b) {
        bool doAdd = true;
        for(auto p:a)
        {
            if(p.act.robot == bb.act.robot && p.act.balls == bb.act.balls) {
                //printf("found -> {%p, %dx%d, %dx%d}\n", p.prev, p.act.robot.x, p.act.robot.y,
                //        p.act.balls[0].x, p.act.balls[0].y);
                doAdd = false;
            }
        }
        for(auto p:paths)
        {
            if(p.act.robot == bb.act.robot && p.act.balls == bb.act.balls) {
                //printf("found -> {%p, %dx%d, %dx%d}\n", p.prev, p.act.robot.x, p.act.robot.y,
                //        p.act.balls[0].x, p.act.balls[0].y);
                doAdd = false;
            }
        }
        //printf("%d -> {%p, %dx%d, %dx%d}\n", doAdd, bb.prev, bb.act.robot.x, bb.act.robot.y,
        //        bb.act.balls[0].x, bb.act.balls[0].y);
        if(doAdd)
            a.push_back(bb);
    }
    return a;
}

//struct path_t {
//    path_t *prev;
//    action_t act;
//    float    inc_cost;
//    float    inc_gain;
//    float    total_cost;
//};

path_t initialPath(pos_t robot, balls_t balls)
{
    return path_t{NULL, {NOT_A_DIRECTION, robot, robot, balls}, dist(balls, goals), 0};
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

    //Explore path
    paths.push_back(unexplored_path[ind]);
    path_t &p = *paths.rbegin();
    unexplored_path.erase(unexplored_path.begin()+ind);
    printf("-------------------------------------------------\n");
    background->dump(p.act.robot, p.act.balls, goals);

    grid_t g1 = mergeBalls(*background, p.act.balls);
    grid_t g2 = connectivity(g1, p.act.robot);

    actions_t a = findActions(g1, g2, p.act.balls);
    printf("actions: %d ->", (int)unexplored_path.size());
    append(unexplored_path, actionsToPaths(p, a));
    printf("%d\n",(int)unexplored_path.size());
}

//One of the explored paths result in a zero distance win
bool foundSolution(void)
{
    for(int i=0; i<(int)paths.size(); ++i)
        if(paths[i].remaining == 0)
            return true;
    return false;
}

void setup_demo(void)
{
    background = new grid_t(6,4);
    auto &bg = *background;
    bg(5,0) = 1;
    bg(5,1) = 1;
    bg(0,2) = 1;
    bg(1,3) = 1;

    balls.push_back(pos_t(4,2));
    balls.push_back(pos_t(3,2));
    goals.push_back(pos_t(1,1));
    goals.push_back(pos_t(2,1));
    background->dump(robot, balls, goals);
    printf("\n\n");

    grid_t g1 = mergeBalls(*background, balls);
    grid_t g2 = connectivity(g1, robot);
    g2.dump();

    actions_t a = findActions(g1, g2, balls);
    print(a);

    paths.push_back(initialPath(robot, balls));
    append(unexplored_path, actionsToPaths(paths[0], a));
    while(!unexplored_path.empty() && !foundSolution())
        explore();
}

int main()
{
    setup_demo();
    return 0;
}
