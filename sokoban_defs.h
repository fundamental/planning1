#include <cstdio>
#include <vector>
#include <queue>
#include <cassert>
#include <cstring>
#include <cstdlib>
#include <set>
#include <cmath>
#include <string>
#include <iostream>

using std::vector;
using std::queue;
using std::set;
using std::string;

//NaN state for initiail conditions
enum dir_t {L,R,U,D,NOT_A_DIRECTION};

// Struct declarations
struct pos_t;
struct action_t;
struct path_t;
struct grid_t;

struct pos_t
{
    pos_t() :pos_t(0,0) {}
    pos_t(int X, int Y) :x(X), y(Y) {}
    bool operator==(const pos_t &o) const {
        return o.x == x && o.y == y;
    }
    bool operator<(const pos_t &o) const {
        return x==o.x ? y < o.y : x < o.x;
    }
    int x,y;
};

typedef vector<pos_t> balls_t;
typedef vector<pos_t> goals_t;

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
    int      robot_steps;
    size_t   depth;
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

    void dump(pos_t robot, balls_t balls, goals_t goals)
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

typedef vector<path_t> paths_t;