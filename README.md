# bt_executer

**bt_executer** is a specialized package designed to run behavior trees based [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2). It functions through the use of `bt_executer_node.cpp`, a node that can be initiated using `bt_executer.launch.py`. This launch process is responsible for importing configurations from `config/bt_config.yaml` and `config/skills_config.yaml` through the *cnr_param* utility. 

- `bt_config.yaml` specifies which skill plugins should be loaded, as well as identifies the .xml file that contains the behavior tree structure.
- `skills_config.yaml` outlines the necessary parameters for each node (leaf) within the tree.

It's important to ensure that the behavior tree's .xml file is located within the `trees` directory. Furthermore, each leaf node within the tree is equipped with an input port named `param_ns`, which is used to specify the sub-namespace for its parameters.

Within the `skills` folder, you will find a collection of predefined skills. These are made available as plugins, adhering to a specific naming convention: *skill_name_plugin*. This standardized naming convention is crucial for the bt_executer node to locate and load the plugins, thereby enhancing the system's modularity and ease of use.