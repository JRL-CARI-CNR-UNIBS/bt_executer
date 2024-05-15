# bt_executer

**bt_executer** is a specialized package designed to run behavior trees based on [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2). It functions through the use of `bt_executer_node.cpp`, a node that can be initiated using `bt_executer.launch.py`. This launch process is responsible for importing configurations from `config/bt_config_template.yaml` and `config/skills_config_template.yaml` through the *cnr_param* utility:

- `bt_config_template.yaml` specifies which skill plugins should be loaded, as well as identifies the package and the name of the .xml file that contains the behavior tree structure.
- `skills_config_template.yaml` outlines the necessary parameters for each node (leaf) within the tree.

It's important to ensure that the behavior tree's .xml file is located within the `trees` directory of the package identified by `bt_package` in `bt_config_template.yaml`. Furthermore, each leaf node within the tree is equipped with an input port named `param_ns`, which is used to specify the sub-namespace for its parameters.

Within the `skills` folder, you will find a collection of predefined skills. These are made available as plugins, adhering to a specific naming convention: *skill_name_plugin*. This standardized naming convention is crucial for the bt_executer node to locate and load the plugins, thereby enhancing the system's modularity and ease of use.

## Use in an external package
If you want to execute a behavior tree in your specific application, you should:
  1) Copy `config/bt_config_template.yaml` and `config/skills_config_template.yaml` to `your_package/config/bt_config.yaml` and `your_package/config/skills_config.yaml`;
  2) Copy `trees/main_tree.xml` (or write it from scratch) to `your_package/trees/` folder, and modify the .xml file as needed;
  3) Set the list of plugins to load, the name of `your_package` and of the .xml file in`your_package/config/bt_config.yaml` and the skills parameters in `your_package/config/skills_config.yaml`;
  4) Copy `launch/bt_executer.launch.py` to `your_package/launch/` folder and change the line `config_folder = PathJoinSubstitution([FindPackageShare("bt_executer"),"config"])` to `config_folder = PathJoinSubstitution([FindPackageShare("your_package"),"config"])`

Note that the main behavior tree must have `<BehaviorTree ID="MainTree">`.
## Install
Dowload the `deps.repo` file, which contains all the dependecies of the bt_executer package, and compile:
```
cd ws/src
wget  --backups=1 https://raw.githubusercontent.com/JRL-CARI-CNR-UNIBS/bt_executer/master/deps.repo
vcs import < deps.repo
cd ..
colcon build --symlink-install 
```
## Examples
[Here](https://github.com/JRL-CARI-CNR-UNIBS/bt_executer_examples.git) you can find some examples of how to use the package, based on [this cell](https://github.com/JRL-CARI-CNR-UNIBS/battery_cell.git).