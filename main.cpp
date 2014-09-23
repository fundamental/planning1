struct state_t;
struct graph_t;
struct subspace_t;
struct action_t;
struct actions_t;
struct path_t;

state_t    convertToState();
graph_t    loadGraph();
subspace_t findSubspace();
actions_t  findActions(state_t);


int main()
{
    return 0;
}
