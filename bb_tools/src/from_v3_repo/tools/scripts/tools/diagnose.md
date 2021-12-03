# Diagnose via CMD line

## Port Diagnose 

Currently we have port diagnose via GUI which will show result of port diagnose. 
Now for supporting command line only when we are only using `ssh`. 

### How to use 

Since we have no idea about which PC will connect to Mifi and so simply usage.
Split scripts to two: one for Large PC of Lionel, another for small PC of
Lionel.

BTW, we don't have GS port diagnose. 


1. move into tools folder: `cd ~/catkin/src/boothbot/tools/scripts/tools`
2. PCs
   1. Large PC: run `./port_diagnose_lionel.py`. 
   2. Small PC: run `./port_diagnose_cb.py`. 


## Dashboard (terminal)

We have GUI page to show states of Lionel. For usage case in termial, created
similar dashboard in termial.

### How to Use

1. move into tools folder: `cd ~/catkin/src/boothbot/tools/scripts/tools`
2. PCs
   1. On Lionel: run `./dashboard_lionel.py`
   2. On GS: run `./dashboard_gs.py`
