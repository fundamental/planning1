#include <cstdio>
#include <vector>
#include <queue>
#include <cassert>
#include <cstring>
#include <cstdlib>
#include <set>
#include <cmath>

using std::vector;
using std::queue;
using std::set;

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
    bool operator<(const pos_t &o) const {
        return x==o.x ? y < o.y : x < o.x;
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

    //for sets
    bool operator==(const action_t &a) const
    {
        return robot == a.robot && balls == a.balls;
    }
    bool operator<(const action_t &a) const {
        return robot == a.robot ? balls < a.balls : robot < a.robot;
    }
};

typedef vector<action_t> actions_t;

struct path_t {
    path_t *prev;
    action_t act;
    float    remaining;
    float    total_cost;
    size_t   depth;
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

set<action_t> known_states;

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
        //printf("[%d %d] [%d %d] [%d %d] [%d %d]\n",
        //connectivity(b.x+1, b.y, 0), !background(b.x-1, b.y, 1),
        //connectivity(b.x-1, b.y, 0), !background(b.x+1, b.y, 1),
        //connectivity(b.x, b.y+1, 0), !background(b.x, b.y-1, 1),
        //connectivity(b.x, b.y-1, 0), !background(b.x, b.y+1, 1));
        //XXX double check this directions are all right
        if(connectivity(b.x+1, b.y, 0) && !background(b.x-1, b.y, 1))//L
            result.push_back({L,{b.x+1,b.y},{b.x,b.y},gsub(balls,{b.x,b.y},{b.x-1,b.y})});
        if(connectivity(b.x-1, b.y, 0) && !background(b.x+1, b.y, 1))//R
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

float dist2(vector<pos_t> a, vector<pos_t>b)
{
    assert(a.size() == b.size());
    vector<bool> v;
    for(auto bb:b) {
        (void) bb;
        v.push_back(0);
    }

    //greedy fit
    float total = 0;
    for(auto x:a) {
        float min = 99999;
        int   ind = -1;
        for(int i=0; i<(int)b.size(); ++i) {
            float c = abs(x.x-b[i].x) + abs(x.y-b[i].y);
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

float dist(vector<pos_t> a, vector<pos_t> b)
{
    assert(a.size() == b.size());
    float total = 0;
    float min   = 100000;
    for(int i=0; i<(int)a.size(); ++i) {
        float c = abs(a[i].x-b[i].x) + abs(a[i].y-b[i].y);
        if(c < min)
            min = c;
        total += c;
    }
    return total;
}

//check for corner stuck plus hall stuck
bool isImpossible(grid_t g, balls_t balls, vector<pos_t> goals)
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
        //continue;
        //XXX unreliable cases

        ////vertical case
        bool wallLU = g(b.x-1,b.y,1) && g(b.x-1, b.y-1,1);
        //bool wallLD = g(b.x-1,b.y,1) && g(b.x-1, b.y+1,1);
        //bool wallRU = g(b.x+1,b.y,1) && g(b.x+1, b.y-1,1);
        //bool wallRD = g(b.x+1,b.y,1) && g(b.x+1, b.y+1,1);

        if(wallLU && gg(b.x, b.y-1, 1))
            return true;
        //if((wallLU || wallRU) && gg(b.x, b.y-1))
        //    return true;
        //if((wallLD|| wallRD) && gg(b.x, b.y+1))
        //    return true;

        ////horizontal case
        //bool wallUL = g(b.x,b.y-1,1) && g(b.x-1, b.y-1,1);
        bool wallDL = g(b.x,b.y+1,1) && g(b.x-1, b.y+1,1);
        //bool wallUR = g(b.x,b.y-1,1) && g(b.x+1, b.y-1,1);
        //bool wallDR = g(b.x,b.y+1,1) && g(b.x+1, b.y+1,1);
        //
        //if((wallUL || wallDL) && gg(b.x-1, b.y-1))
        //    return true;
        //if((wallUR|| wallDR) && gg(b.x+1, b.y))
        //    return true;
        if(wallDL && gg(b.x-1, b.y,1))
            return true;
        //printf("point %dx%d\n", b.x, b.y);
        //printf("[%d %d %d %d   %d %d %d %d]\n",wallLU, wallLD, wallRU, wallRD, wallUL, wallDL, wallUR, wallDR);
    }
    return false;
}


paths_t actionsToPaths(path_t &cur, actions_t act)
{
    //XXX improve cost functions
    paths_t result;
    for(auto a:act) {
        float incremental_cost = 0.1;
        //if(known_states.find(a) != known_states.end())
        //    continue;
        //if(isImpossible(*background,a.balls, goals))
        //    printf("Impossible = {(%d,%d) -> (%d,%d)}\n", a.pos.x, a.pos.y, a.robot.x, a.robot.y);
        result.push_back({&cur, a, dist2(a.balls,goals) + 99999*isImpossible(*background,a.balls, goals),
                incremental_cost+cur.total_cost, cur.depth+1});
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
        if(bb.remaining > 99999)
            continue;
        bool doAdd = known_states.find(bb.act) == known_states.end();
        //doAdd = true;
        //for(auto p:a)
        //{
        //    if(p.act.robot == bb.act.robot && p.act.balls == bb.act.balls) {
        //        //printf("found -> {%p, %dx%d, %dx%d}\n", p.prev, p.act.robot.x, p.act.robot.y,
        //        //        p.act.balls[0].x, p.act.balls[0].y);
        //        doAdd = false;
        //    }
        //}
        //for(auto p:paths)
        //{
        //    if(p.act.robot == bb.act.robot && p.act.balls == bb.act.balls) {
        //        //printf("found -> {%p, %dx%d, %dx%d}\n", p.prev, p.act.robot.x, p.act.robot.y,
        //        //        p.act.balls[0].x, p.act.balls[0].y);
        //        doAdd = false;
        //    }
        //}
        if(doAdd) {
            a.push_back(bb);
            known_states.insert(bb.act);
        }
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
    return path_t{NULL, {NOT_A_DIRECTION, robot, robot, balls}, dist2(balls, goals), 0, 0};
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
    paths.push_back(unexplored_path[ind]);
    path_t &p = *paths.rbegin();
    unexplored_path.erase(unexplored_path.begin()+ind);
    printf("-------------------------------------------------\n");
    background->dump(p.act.robot, p.act.balls, goals);

    grid_t g1 = mergeBalls(*background, p.act.balls);
    grid_t g2 = connectivity(g1, p.act.robot);
    //g2.dump();

    actions_t a = findActions(g1, g2, p.act.balls);
    //print(a);
    static int iteration = 0;
    iteration++;
    printf("actions: %d ->", (int)unexplored_path.size());
    append(unexplored_path, actionsToPaths(p, a));
    printf("%d\n",(int)unexplored_path.size());
    printf("step: %d depth: %d space: %d cost: %f\n",iteration, p.depth, unexplored_path.size(), p.remaining);
    //static int maxdepth = 0;
    //if((int)p.depth > maxdepth) {
    //    maxdepth = p.depth;
    //    background->dump(p.act.robot, p.act.balls, goals);
    //}
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

void clear(grid_t &g, int x1, int x2, int y1, int y2)
{
    for(int x=x1; x<=x2; ++x)
        for(int y=y1; y<=y2; ++y)
            g(x,y) = 0;
}


//first level of the flash game

//012345678901234567
//xxxx   xxxxxxxxxxx0
//xxxxB  xxxxxxxxxxx1
//xxxx  Bxxxxxxxxxxx2
//xx  B B xxxxxxxxxx3
//xx x xx xxxxxxxxxx4
//   x xx xxxxx   gg5
// B  B           gg6
//xxxx xxx xxRx   gg7
//xxxx     xxxxxxxxx8
void setup_demo2(void)
{
    background = new grid_t(18,9);
    auto &bg = *background;
    //XXX really make some sort of import function
    for(int x=0; x<18; ++x)
        for(int y=0; y<9; ++y)
            bg(x,y) = 1;
    clear(bg,4,7,0,3);
    clear(bg,0,17,6,6);
    clear(bg,4,4,0,8);
    clear(bg,7,7,0,8);
    clear(bg,13,17,5,7);
    clear(bg,4,7,8,8);
    clear(bg,0,2,5,6);
    bg(2,3) = 0;
    bg(3,3) = 0;
    bg(2,4) = 0;
    bg(11,7) = 0;
    bg(7,0) = bg(7,1) = bg(7,2) = 1;

    bg.dump();

    balls.push_back({1,6});
    balls.push_back({4,1});
    balls.push_back({4,3});
    balls.push_back({4,6});
    balls.push_back({6,2});
    balls.push_back({6,3});
    for(int i=16; i<=17; ++i)
        for(int j=5; j<=7; ++j)
            goals.push_back({i,j});
    //goals.push_back({16,5});
    robot = pos_t{11,7};
    background->dump(robot, balls, goals);
    //printf("\n\n");

    //grid_t g1 = mergeBalls(*background, balls);
    //grid_t g2 = connectivity(g1, robot);
    //g2.dump();

    //actions_t a = findActions(g1, g2, balls);
    //print(a);

    unexplored_path.push_back(initialPath(robot, balls));
    //append(unexplored_path, actionsToPaths(paths[0], a));
    while(!unexplored_path.empty() && !foundSolution())
        explore();
}

int main()
{
    setup_demo2();
    return 0;
}
