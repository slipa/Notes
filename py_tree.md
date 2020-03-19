### Behavior

Behavior 需要繼承自 py_trees.Behaviour
其主要架構為：

```python
class Foo(py_trees.Behaviour):
    def __init__(self, name):
        """
        """
        super(Foo, self).__init__(name)

    def setup(self, timeout):
        """
        """

    def initialise(self):
        """
        """

    def update(self):
        """
        """
        

    def terminate(self, new_status):
        """
        """
```


Parallels enable a kind of concurrency

digraph parallel {
graph [fontname="times-roman"];
node [fontname="times-roman"];
edge [fontname="times-roman"];
"Parallel" [fontcolor=black, shape=note, fontsize=11, style=filled, fillcolor=gold];
"B1" [fontcolor=black, shape=ellipse, fontsize=11, style=filled, fillcolor=gray];
"Parallel" -> "B1";
"B2" [fontcolor=black, shape=ellipse, fontsize=11, style=filled, fillcolor=gray];
"Parallel" -> "B2";
}
Ticks every child every time the parallel is run (a poor man’s form of paralellism).

Parallels will return FAILURE if any child returns FAILURE
Parallels with policy SUCCESS_ON_ONE return SUCCESS if at least one child returns SUCCESS and others are RUNNING.
Parallels with policy SUCCESS_ON_ALL only returns SUCCESS if all children return SUCCESS
See also


