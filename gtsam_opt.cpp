#include <GraphOpt.h>

int
main(int argc, char *argv[])
{
    std::vector<std::string> graphs = {
      "../data/graph_101.dat",
      "../data/graph_126.dat",
      "../data/graph_151.dat",
      "../data/graph_26.dat",
      "../data/graph_51.dat",
      "../data/graph_76.dat"};
    
    for(int i=0; i<1000; i++) {
        basalt::GraphOpt gopt;
        gopt.load(graphs[i%graphs.size()]);
        for(int its=0; its<5; its++) {
            gopt.optimize(1);
            gopt.initialEstimate = gopt.estimate;
        }
    }
    return 0;
}