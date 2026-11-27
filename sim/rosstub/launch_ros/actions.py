class Node:
    def __init__(self, package=None, executable=None, name=None, output=None, prefix=None,
                 parameters=None, remappings=None, condition=None, arguments=None, **kw):
        self.package, self.executable, self.name = package, executable, name
        self.prefix, self.parameters, self.remappings = prefix, parameters or [], remappings or []
        self.condition, self.arguments = condition, arguments or []
        self.node_package, self.node_executable = package, executable
