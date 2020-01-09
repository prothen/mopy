# mopy
Driver for Qualisys motion capture system using Python with the Qualisys Track Manager (QTM) API. 
Supports both local multiprocessed interface as well as ROS-based interface with corresponding launch files.

## Brief
This repository provides a library for forwarding and filtering raw qualisys state measurements. 
The measurements provide position and attitude in form of a rotation matrix. 
(Unique and singularity-free representation of unit 2-sphere.) The rotation matrix is then converted to a quaternion and exposed to the desired interface.

The provided interfaces to Qualisys are either a multiprocessed environment for on-target implementation or ROS-based.
Possible extensions using filtering algorithms are planned to be implemented. Skeletons are provided.

## Usage
Essentially two distinct implementations are provided. 
First the local implementation without ROS using a multiprocessed environment 
and second a ROS-based implementation that publishes for each body name provided 
in the launch file argument model_names a separate topic in the private namespace of mopy.
 (Note: _this can also be remapped to a desired topic -> a remap placeholder is provided._)
 
_For example_: Provided the launch command with `model:=testname` adding a remap to the launch file with `
        <remap from="~testname" to="/global_namespace/desired_topic" />`
- see also the documentation [here](https://qualisys.github.io/qualisys_python_sdk/index.html)
### Raw Position and Attitude Measurement
- Execute ROS node
    - `roslaunch mopy stream.launch`
        - will create distinct topics for all models registered in current Qualisys configuration (server side)
    - `roslaunch mopy stream.launch model:=modelname`
        - will create distinct topic for `modelnames` (list of strings) without space in following syntax
            - `roslaunch mopy stream.launch model:=modelname1,modelname2 filter:=True debug:=True`
    - `roslaunch mopy stream.launch model:=modelname filter:=True` 
        - will create distinct topic for `modelnames` and filter states (todo)
    - `roslaunch mopy stream.launch model:=modelname filter:=True debug:=True` 
        - will create distinct topic for `modelnames` and filter states (todo) together with debug prints

## Setup
- The following procedure is tested under Python 3.7
- Clone this repository in a catkin compliant workspace structure.
    - e.g. `yourws/src/mopy`
- Install dependencies
    - `pip install -r requirements.txt`
    - `cd yourws/externals/transformations && python setup.py install`
- build your workspace
    - `cd yourws && catkin build`
- source the workspace environment variables
    - e.g. `source yourws/devel/setup.bash`
- execute `roslaunch` and `rosrun` as detailed above.
    
### Todo
- [x] exit asyncio more gracefully and adjust
    - [x] improve exit by restructuring and using signal.SIGINT events
- [ ] commit multiprocessed ROS-free implementation for QTM interaction
- [ ] include filter to estimate v, w


### Contribution
Any form of contribution is very welcome and very much appreciated. For python scripts adhere to PEP8 and for ROS 
conventions REP standards (such as REP105 for frames).

For any questions feel free to raise an issue or for bug fixes to submit a pull request.

**Contact:** Philipp Rothenhäusler | E-mail: _philipp.rothenhaeusler (gmail.com)_

**For Python**:
- show PEP8 violations (and manual fix - recommended)
    
    `pip install pycodestyle`
    
    usage:
    
    `pycodestyle --show-source --show-pep8 interfaces.py` (show code)
    
    `pycodestyle --statistics -qq interfaces.py` (show common errors)

- automatically correct PEP8 violations
    
    `pip install autopep8`

    usage:

    `autopep8 --in-place --aggressive --aggressive interface.py`
