## Dev Notes ##

File to keep track of development notes for CASA nad act as a guide for best practices.

#### Nodes ####
Nodes should written in c++.

#### Namespaceing ####
Nodes namespacing and topic namespacing is kept separate. Nodes are namespaced in the launch file when defining the name and use the namespace `casa_X`. Topics are namespaced within the node and use the `casaX/` namespace. 