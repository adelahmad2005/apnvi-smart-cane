class LaunchDescription:
    def __init__(self, entities=()):
        self.entities = list(entities)
from . import actions, conditions, substitutions, launch_description_sources  # noqa
