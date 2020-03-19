tutorial one

建構一棵樹

1. "Behavior" is leaf of a behavior  tree
2. Tree 的中間節點跟root為composites : Parallels, Sequences, Chooser, Selector
3. class ToBlackboard is a behavior.
4. subscriber.ToBalckboard: 如果訂閱的topic沒有收到值，每次tick的status會是running，否則就會是success

[moduel API](http://docs.ros.org/kinetic/api/py_trees_ros/html/modules.html)