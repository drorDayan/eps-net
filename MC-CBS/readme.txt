# Multi-Agent Path Finding for Large Agent
Jiaoyang Li<jiaoyanl@usc.edu>, Dec 2018

Code for the paper "Multi-Agent Path Finding for Large Agent" published in AAAI 2018. The main goal is to take the shapes of agents into consideration in the Multi-Agent Path Finding problem. 

This code includes two independent solvers: one for rectangle-shaped agents on 2D grid and the other for arbitrary-shaped agents on general graph.


## Dependencies

### BOOST ( https://www.boost.org/ )
used by both solvers.

### sparsehash (https://github.com/sparsehash/sparsehash )
used by both solvers.

### YAML (https://github.com/jbeder/yaml-cpp )
used by "general-graph" solver.

### Eigen (http://eigen.tuxfamily.org/index.php?title=Main_Page )
used by "general-graph" solver.

## Instances
Folder "instances" includes all instances used in the paper.


## Evaluation
run2Dgrid.bat and runGeneralGraph.bat have example commands to reproduce experimental results in the paper.
You can also use command --help to learn more details.


## MDD-SAT
This code only works for rectangle-shaped agents on 2D grid.

To compile:
./config_relase
make optimized

To run experiments:
cd src
./insolver_reLOC --lusc-map-input=xxx --lusc-robot-input=xxx --lusc-output-file=xxx --encoding=lmdd++ --total-timeout=300



