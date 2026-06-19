"""
robots/base_robot.py
====================
Interface every robot implements so the Runner can drive any of them
identically. New robots (or new policies) become new subclasses — the Lego
slot for the "policy layer."

Contract:
    spawn(world)   -> add the articulation to the scene, load the policy
    on_reset()     -> (re)initialize any policy/internal state after world.reset()
    step(world)    -> run one inference step: observe -> policy -> apply action
    command(**kw)  -> set high-level commands (e.g. walk_forward=1.0)
"""

from __future__ import annotations
from abc import ABC, abstractmethod


class BaseRobot(ABC):
    def __init__(self):
        self._command = {}

    def command(self, **kwargs):
        """Set high-level commands. e.g. robot.command(walk_forward=1.0)."""
        self._command.update(kwargs)
        return self

    @abstractmethod
    def spawn(self, world):
        ...

    @abstractmethod
    def on_reset(self):
        ...

    @abstractmethod
    def step(self, world):
        ...
