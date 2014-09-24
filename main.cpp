#include <cstdio>
#include <vector>
#include <queue>
#include <cstring>

using std::vector;
using std::queue;

struct pos_t;
struct state_t;
struct graph_t;
struct subspace_t;
struct action_t;
struct path_t;
struct grid_t;

struct action_t {
};

typedef vector<action_t> actions_t;

struct pos_t
{
    pos_t() :pos_t(0,0) {}
    pos_t(int X, int Y) :x(X), y(Y) {}
    bool operator==(pos_t o) {
        return o.x == x && o.y == y;
    }
    int x,y;
};


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

    int &operator()(int x, int y)
    {
        return data[x+y*X];
    }

    int &operator()(pos_t p)
    {
        return data[p.x+p.y*X];
    }

    void dump(pos_t robot, vector<pos_t> balls)
    {
        for(int y=0; y<Y; ++y) {
            for(int x=0; x<X; ++x) {
                bool isBall = false;
                for(pos_t b:balls)
                    isBall |= b == pos_t(x,y);

                if(robot == pos_t(x,y))
                    putchar('R');
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
pos_t robot;

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

actions_t findActions(grid_t background, grid_t connectivity, vector<pos_t> balls)
{
    return actions_t();
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
    background->dump(robot, balls);
    printf("\n\n");

    grid_t g1 = mergeBalls(*background, balls);
    grid_t g2 = connectivity(g1, robot);
    actions_t a = findActions(g1, g2, balls);
    g2.dump();
}

int main()
{
    setup_demo();
    return 0;
}
