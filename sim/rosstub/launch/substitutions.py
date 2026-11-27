class LaunchConfiguration:
    def __init__(self, name, default=None): self.name = name
class PathJoinSubstitution:
    def __init__(self, parts): self.parts = parts
class PythonExpression:
    def __init__(self, parts): self.parts = parts
class TextSubstitution:
    def __init__(self, text=''): self.text = text
class EnvironmentVariable(LaunchConfiguration): pass
