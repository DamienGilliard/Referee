# One page synthesis
This single page presents the current result of this repository. It is aimed at understanding the current state of the repository and should be an evolutive document.

## Result of the registration
The repository's registration result with the test data is illustrated by the following figure. Initial positions of the scans are marked by the dark axes (dark red for x axis and dark green for y axis). Redistred scans' positions are illustrated by the light axes (bright red for x axis and bright green for y axis). The blue axes represent the translation resulting of the registration.

<center><img src="./transformations.png" width = "70%"></center>

## Illustration of the registration's basis
The repository relies on the analysis of the probabilities of each point cloud's transformations and to illustrate this, we provide for each point cloud the mean and standard variation of the point cloud's rotations. We do it for the rotations for two reasons: - first, with our dataset, the point cloud's rotations are along the z axis and thus all characterizable by a single value, allowing to plot the probability densities in a 2D graph (3D translation vectors should be represented in 4D, for example with a 3D graph and heat map, which would be much harder to read.)

<center><img src="./normal_distribution.png" width = "70%"></center>