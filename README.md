# Decentralized Collective Robot Construction
This project is a Multi-Agents Reinforcement Learning (MARL) Collective Robot Construction simulation environment based on [ARGoS3](https://www.argos-sim.info)

## Prerequisites
* Ubuntu 18.04 or above
* Python 3.6
* Argos3
[18.04](https://drive.google.com/file/d/19RZtiHKYhTA_SXzMLRSksH_zO5YXr_kR/view) or [20.04](https://drive.google.com/file/d/1oO2lb2LuLq4IrZmNMiJurWTotHp_pDye/view)

## Installation
0. Install Argos3
Download .deb file for Ubuntu
[18.04](https://drive.google.com/file/d/19RZtiHKYhTA_SXzMLRSksH_zO5YXr_kR/view) or [20.04](https://drive.google.com/file/d/1oO2lb2LuLq4IrZmNMiJurWTotHp_pDye/view)
```bash
$ sudo apt install ./argos3_simulator-3.0.0-x86_64-beta59.deb
```

1. Clone this repository and submodules.
```bash
$ git clone https://github.com/gaya7698/collective_construction collective_construction
$ cd collective_construction                # go into the folder you git cloned
$ mkdir build                               # create build folder
$ cd build                                  # go into build folder
$ cmake ..                                  # setup compilation
$ make                                      # compile
$ cd ..
```
## Usage
### Run with visuliztion
```bash
$ cd collective_construction                # go into the folder you git cloned
$ chmod +x run_agents.sh                    # make file executable
$ ./run_agents.sh


```
### Run headless
```bash
$ cd collective_construction                # go into the folder you git cloned
$ chmod +x run_agents_headless.sh                    # make file executable
$ ./run_agents_headless.sh
```

## References
- ARGos3: https://github.com/ilpincy/argos3