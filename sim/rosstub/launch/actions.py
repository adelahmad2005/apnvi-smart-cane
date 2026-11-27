class _Text:
    def __init__(self, text): self.text = text
class DeclareLaunchArgument:
    def __init__(self, name, default_value=None, **kw):
        self.name = name; self.default_value = [_Text(default_value)]
class GroupAction:
    def __init__(self, actions=(), scoped=True, condition=None, **kw):
        self.actions = list(actions); self.condition = condition
class IncludeLaunchDescription:
    def __init__(self, source, launch_arguments=None, condition=None, **kw):
        self.source = source; self.launch_arguments = list(launch_arguments or [])
class AppendEnvironmentVariable:
    def __init__(self, *a, **k): pass
class SetEnvironmentVariable(AppendEnvironmentVariable): pass
class ExecuteProcess(AppendEnvironmentVariable): pass
class TimerAction(GroupAction): pass
class OpaqueFunction(AppendEnvironmentVariable): pass
