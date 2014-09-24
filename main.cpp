#include <cstdio>
#include <vector>
#include <cstring>

using std::vector;

struct pos_t;
struct state_t;
struct graph_t;
struct subspace_t;
struct action_t;
struct actions_t;
struct path_t;
struct grid_t;

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

    int &operator()(int x, int y)
    {
        return data[x+y*X];
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
};

state_t    convertToState();
graph_t    loadGraph();
subspace_t findSubspace();
actions_t  findActions(state_t);

grid_t *background;
vector<pos_t> balls;
pos_t robot;

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
}

int main()
{
    setup_demo();
    return 0;
}
