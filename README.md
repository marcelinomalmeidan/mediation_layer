# mediation_layer

This package handles collision avoidance for multiple vehicles in an enclosed volume. This repository makes sure that no vehicle
exits the volume, neither allow them to collide among themselves. 

The Mediation Layer package is based on potential fields, repelling vehicles that are too close to each other, as well as repelling
vehicles from the boundary walls. When no repulsion forces are acting on a quadcopter, the Mediation Layer's output converges to the 
desired reference. This package can be summarized in the following image:


In the image above, y_{ref} is a desired reference
![equation](<a href="http://www.codecogs.com/eqnedit.php?latex=\ddot{\bm{y}}_{ref}^*&space;=&space;k_p(\bm{y}_{ref}&space;-&space;\bm{y}_{ref}^*)&space;&plus;&space;k_d(\dot{\bm{y}}_{ref}&space;-&space;\dot{\bm{y}}_{ref}^*)&space;&plus;&space;\ddot{\bm{y}}_{ref}&space;&plus;&space;\bm{F}_{field}(\bm{X})" target="_blank"><img src="http://latex.codecogs.com/gif.latex?\ddot{\bm{y}}_{ref}^*&space;=&space;k_p(\bm{y}_{ref}&space;-&space;\bm{y}_{ref}^*)&space;&plus;&space;k_d(\dot{\bm{y}}_{ref}&space;-&space;\dot{\bm{y}}_{ref}^*)&space;&plus;&space;\ddot{\bm{y}}_{ref}&space;&plus;&space;\bm{F}_{field}(\bm{X})" title="\ddot{\bm{y}}_{ref}^* = k_p(\bm{y}_{ref} - \bm{y}_{ref}^*) + k_d(\dot{\bm{y}}_{ref} - \dot{\bm{y}}_{ref}^*) + \ddot{\bm{y}}_{ref} + \bm{F}_{field}(\bm{X})" /></a>)
