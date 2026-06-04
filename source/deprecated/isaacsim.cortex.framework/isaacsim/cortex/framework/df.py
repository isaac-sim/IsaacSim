# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The decision framework. Decider networks and state machines are implemented here.

Conceptually, a decider network is a mapping from world and logical state to a choice of high-level
action. That mapping often has intuitive structure to it; decider networks make it easy to design
many of these mappings by hand. The world and logical state is stored in a user defined context
object which is simply passed around within the framework and made available to the decider nodes
and state machines to aid in making decisions and taking action.

A decider network is an acyclic graph of decider nodes, each of which has a simple enter(),
decide(), and exit() interface. The decide() method makes the decision by choosing a child. In a
leaf, that child can be None, in which case, the node typically acts to step an action. Leaves, for
that reason, often represent actions. See DfDecider, DfDecision, and DfAction. DfNetwork is a good
entry point for reading through the code. It shows how the decider network is created and stepped.

The basic descent algorithm for tracing from the root of the decider network down to an action
choice is implemented by df_descend(), which is run once every cycle. Importantly, when the system
reaches a given DfDecider node for the first time along a given decision path through the network,
it calls enter() on that node, followed by decide(). While it continues to trace the same path from
the root to the node, it calls only decide() on that node. But once that node is no longer reached,
it calls exit() on the node. This is implemented by keeping track of the path traced through the
network from cycle to cycle and observing whether the new path trace is branching relative to the
previous. When a branch is detected, exit() is called on the old branch in reverse order from the
old leaf to one node before the branch point (the first node no longer reached), and enter is called
on all new nodes along the new path to the new leaf. See df_descend() for details.

These calls to enter() and exit() give the decider nodes a notion of statefulness allowing them to
setup and tear down local memory to affect their decisions. The methods parallel the API to state
machines (see DfState), which have the API enter(), step(), and exit(). That means state machines
can be used inside decider nodes to help make decisions. Likewise, decider networks can be used
inside state machines to help define the state behavior.

The decider network is fundamentally reactive. It's constantly making decisions from the root to the
choice of action leaf on each cycle, and will change its mind as needed if the world or logical
state changes. But sometimes it's important to be able to lock the decider network to prevent it
from changing its mind at critical points (especially for atomic actions implemented as chain state
machines). Setting the decider's is_locked attribute will lock the path from the root to that
decider node and prevent the descent algorithm from deviating from it. See DfSetLockState for a
useful tool for locking and unlocking decider nodes.

Note that this reactivity contrasts fundamentally from state machines where reactivity is usually
implemented as a set of event trigger designed over the top of an unreactive state machine. In state
machines, the system is unreactive by default, and reactivity is the exception. In decider networks,
the system is reactive by default, and locking the system is the exception. This focus on reactivity
makes decider networks ideal for designing reactive collaborative robotic systems.

A collection of useful classes/methods:

DfNetwork, df_descend, DfDecider, DfDecision DfAction form the basic implementation of decider
networks. DfHsmAction uses a hierarchical state machine to define an action.

DfState is the basic interface for a state machine. DfState is bindable (derives from DfBindable) in
the same way DfDeciders are, so when used in decider networks, they have access to the context and
params in the same way decider nodes do (see also DfDecider). DfStateSequence is a simple chain
state machine that's easy to construct from individual states which execute and ultimately return
None when done. Likewise, a DfHierarchicalState abstracts a hierarchical state of a hierarchical
state machine. The run_state_machine() method takes a state and runs it until its step() method
returns None.

DfDeciderState is a state machine that internally runs a decider network inside its step method,
never exiting.  Likewise, a DfTimedDeciderState is a decider state which as a time limit. Once that
time limit has passed, it exits. DfWaitState is a simple state that just waits for a prespecified
duration.

DfStateMachineDecider is a DfDecider node which internally has a state machine which is stepped
during each decide() call.

DfSetLockState locks or unlocks the decider path to accommodate temporally extended atomic actions.
And DfWriteContextState provides a way to write into the context when the state is entered.

The DfRldsDecider is an important DfDecider type implementing the Robust Logical Dynamical Systems
model of reactive decision making.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from collections.abc import Callable, Sequence
from typing import Any, Optional

""" A logical state monitor is a function which takes this DfLogicalState object as input and
processes it to compute some logical state. The computed logical state should be set in the
DfLogicalState object passed to the monitor.
"""
LogicalStateMonitorType = Callable[["DfLogicalState"], None]


class DfLogicalState:
    """Base class for a logical state representation.

    Logical state objects own their own logical state monitors which monitor the logical state. The
    base class provides the data structure for the monitor list as well as basic APIs for adding
    monitors.
    """

    def __init__(self) -> None:
        self.monitors = []

    def add_monitor(self, monitor: LogicalStateMonitorType) -> None:
        """Add a logical state monitor function to this logical state object.

        The order of the added monitors is retained as the call order.

        Args:
            monitor: The monitor method.
        """
        self.monitors.append(monitor)

    def add_monitors(self, monitors: Sequence[LogicalStateMonitorType]) -> None:
        """Add an ordered sequence of monitors.

        The order of the sequence, and the order of calls to this method, are retained as the call
        order.

        Args:
            monitors: The monitors to be added.
        """
        self.monitors.extend(monitors)

    @abstractmethod
    def reset(self) -> None:
        """This method is left unimplemented (no default version) in the base class because it's.

        important that deriving classes don't forget implement it to reset the logical state when
        the simulation is reset.
        """
        raise NotImplementedError()


class DfDecision:
    """Represents a decision made by the decider. It names the child to take and provides it.

    parameters.

    Args:
        name: The name of the decider node child being chosen.
        params:
            The parameters to pass to that decider child. The object passed as parameters is
            whatever object the child is expecting. See the child node documentation for specifics.
    """

    def __init__(self, name: str, params: Any = None) -> None:
        self.name = name
        self.params = params

    def __str__(self) -> str:
        """Returns a printable version of the Decision.

        Returns:
            A string representation of the decision.
        """
        if self.params:
            return f"{self.name}({self.params})"
        else:
            return self.name


class DfBindable(object):
    """A bindable object is an object that an algorithm can bind the context object and any sent to.

    it parameters to. Both of these are then accessible from within the object as self.context and
    self.params at runtime. For instance, DfDecider nodes and DfState are both DfBindable objects,
    so when used within decider networks (DfNetwork) their API methods (enter(), decide() and exit()
    in the case of DfDecider and enter(), step() and exit() in the case of DfState) can expect
    access to the self.context and self.params.
    """

    def bind(self, context: DfLogicalState, params: Any) -> None:
        """The bind API used to bind the context and params to this object.

        Args:
            context: The context object being bound.
            params: The param object being bound.
        """
        self.context = context
        self.params = params


class DfDecider(DfBindable):
    """A decider node of a decider network. The descent algorithm handles automatically setting the.

    internal context member and passing down the parameters both of which can be accessed from
    enter(), decide() and exit() through self.{context,params}.

    Important: Deriving classes that override __init__() should call super().__init__() on the first
    line. It performs important base class initialization.

    Derived classes should override enter(), decide() and exit() as needed. decide() should make the
    decision (using the internal context and passed parameters to do so) and return a DfDecision()
    object encapsulating its choice.

    See the comments in the API methods enter(), decide() and exit() for details on when these
    methods are called and how to use them. The descriptions make use of the concept of a decision
    session. A decision session for a given decider node is an unbroken sequence of decider network
    steps where each step traces the same path from root to the decider node. enter() is called at
    the beginning of the session, and exit() is called when the session ends as indicated by a
    change in path. decide() is called at every step of a session (after enter() on the first step).

    Example: Consider a trace A->B->C->D from root A to leaf D. The first time down, enter and
    decide() (in that order) is called on each node encountered. As long as the same path A->B->C->D
    is traced, only decide() is called on each of those nodes, and we consider each of those nodes
    to be in session. But once the trace becomes, say, A->B->E->F, the nodes A and B are still in
    session, but C and D aren't. We call exit() on D then C (in that order) because they're exiting
    their session, and call enter() on E and F (in that order) as they're encountered during the
    descent, just before calling their decide(), because they're entering a new session.

    Note, in that example, if the path changes from A->B->E->D, the same algorithm is run. The
    prefix path from A->B->E leading into D is different from what it used to be, so the session has
    changed. We call exit() on D and E (in that order), and call enter() right before decide() on E
    and D as we reach them.
    """

    def __init__(self) -> None:
        super().__init__()
        self.name = "root"
        self.context = None
        self.params = None
        self.children = {}

    def __str__(self) -> str:
        """Return the name of this decider node.

        Returns:
            String representation of the object.
        """
        return self.name

    def add_child(self, name: str, child: "DfDecider") -> None:
        """Add a child decider node to this decider node.

        This child can be chosen by the decide() method by returning DfDecision(name), and
        parameters can be passed to it using the optional params field DfDecision(name, params).

        This method is the main method used to construct the topology of a decider network.

        Args:
            name: The name of the child. This name is used to reference the child when choosing it
                in decide().
            child: The decider node child.
        """
        child.name = name
        self.children[name] = child

    def enter(self) -> None:
        """Decider node API method called when this decider node is reached for the first time by a.

        a unique path from the root. enter() is called before decide().

        Use this method for any setup to prepare as a new decision session is started.
        """

    def decide(self) -> DfDecision:
        """Decider node API method called every time this decider node is reached by a decider.

        network descent from the root. decide() is called after enter() if this is the first call.

        This method is called every step of a given decision session. It chooses among its children
        by returning a DfDecision object. The decision object names the chosen child and optionally
        passes any relevant parameters to it. E.g.

                return DfDecision("child1")
                return DfDecision("child1", params)

        See the API of a given decider node for information on parameters are relevant.

        Returns:
            DfDecision object naming the chosen child. The decision object can optionally contain
            parameters to pass to the object.
        """

    def exit(self) -> None:
        """Decider node API method called the first time this decider node is no longer reached by.

        a path from the root. Decider node exit() methods are called in reverse order from the
        previous leaf for all nodes no longer reached by the current decider network descent trace.

        Use this method to perform any cleanup needed at the end of a given decision session.
        """


class DfAction(DfDecider):
    """A decider node that represents a leaf action that makes no additional decisions of its own.

    This action can be viewed as a "higher-level action". It can command the robot policies that
    govern the robots using the robot's command APIs.

    Semantically, actions are stepped, so step() is added as an API call called automatically by
    decide(). Deriving classes should override step() to define the action's operation.
    """

    def step(self) -> None:
        """Step the action. Deriving classes should override this method."""

    def decide(self) -> None:
        """This decider node automatically calls step() each cycle. Deriving classes should not.

        override this decide() method. Instead, override the step() method to define the operation
        of this action.
        """
        self.step()
        return


def df_descend(
    root: DfDecider, root_params: Any, context: DfLogicalState, prev_stack: list[DfDecider]
) -> list[DfDecider]:
    """Descend the decider network from the root to a leaf. Uses the prev_stack to check when or if.

    branches occure. Returns the current stack representing the path from the root to the leaf.

    When a branch is detected, nodes are popped off the previous stack from the leaf to first child
    of the the joining node and exit() is called on each. Then enter() is called on all nodes in the
    new branch. If there is no previoius stack, a first stack is created and enter() is called on
    the entire path to the leaf.

    The root decider node passed in defines the topology of the decider network recursively through
    the network of child nodes. Leaves have no children. The topology should be an acyclic graph to
    guarantee termination, although the topology is not explicitly verified.

    See DfDecider for details on the decider network algorithm. This method implements the descent
    algorithm along with handling calls to enter() and exit() bracketing a decider node's decision
    session.

    Args:
        root: The root decider node.
        root_params: A set of parameters passed into the root decider node.
        context: The context object bound to each processed node along with the passed parameters.
        prev_stack: The previous cycle's decider node path from root to leaf.

    Returns:
        The decision stack from this descent, the list of decider nodes encountered tracing from the
        root to the leaf.
    """
    stack = [root]
    if prev_stack is not None:
        # Step through the stack in reverse order from the leaf to the root checking for the most
        # distal locked node. If one's found, we'll start the algorithm from that node and just
        # descend from there.
        for i, node in enumerate(reversed(prev_stack)):
            if hasattr(node, "is_locked") and node.is_locked:
                root = node
                root_params = node.params
                stack = prev_stack[0 : (len(prev_stack) - i)]

    root.bind(context, root_params)
    node = root

    is_branched = False
    while True:
        if prev_stack is None:
            is_branched = True
        elif not is_branched and (len(prev_stack) < len(stack) or prev_stack[len(stack) - 1] != node):
            # If we detect branching here, then mark it and handle exiting from the previous branch.
            is_branched = True
            for i in range(len(prev_stack) - 1, -1, -1):  # Iterate backward from end.
                # Exit up through the current index because this node is the first verified divergence.
                prev_stack[i].exit()
                if i == len(stack) - 1:
                    break

        if is_branched:
            node.enter()

        decision = node.decide()
        if decision is None:  # Is leaf
            return stack

        node = node.children[decision.name]
        node.bind(context, decision.params)
        stack.append(node)


class DfState(DfBindable):
    """Interface for a state in a state machine. The main work of the state is done by step(). That.

    method should also return the next state to be executed (which could be self for a self
    transition).

    process_step() handles calling enter() and exit() appropriately during state transitions. It
    enables users to design state machines implicitly by simply defining next state transitions by
    returning the desired next state from the step() method.

    Workflow:
    0. Maintain a reference to the initial state. This is the root of the state graph.
    1. Call enter() on the initial state.
    2. Step transitions using next_state = state.process_step(). A next_state of None represents a
       terminal transtion.
    3. On exit, call state.exit() if state is not None.

    See DfStateMachineDecider for an example of this workflow. Alternatively, DfStateSequence gives
    an example of where handling calls to enter(), step(), and exit() manually may be more
    convenient (there, multiple standalone terminal transitioning states are strung together into a
    sequential state machine, with terminal transitions interpreted as next state transitions).
    """

    def __str__(self) -> str:
        """Return a string representation of this state.

        Returns:
            String representation of the object.
        """
        out = ""
        if hasattr(self, "name"):
            out = self.name
        else:
            out = type(self).__name__
        if self.params:
            out = f"{out}({self.params})"
        return out

    def enter(self) -> None:
        """Deriving classes should use this API call to set up the state as needed for stepping."""

    def step(self) -> "DfState":
        """Deriving classes should use this API call to step the state. This is where the main work.

        is done.

        Step should return the next state transition as represented by a reference to the next state
        object being transtitioned to. A transition to None is considered a terminal transition.
        Note returning self is a self transition and is pretty common during processing.

        Returns:
            The next state reference (or None as a terminal transition).
        """
        return None

    def exit(self) -> None:
        """Deriving classes should use this API call to clean up the state as needed before.

        transitioning to another state.
        """

    def process_step(self) -> "DfState":
        """This method can be used to both process the step and correct exit from the current state.

        and enter the next state if a new state transition (non-self) is detected.

        See the workflow described in the top level comment above.

        Returns:
            The next state reference (or None as a terminal transition).
        """
        next_state = self.step()
        if next_state != self:
            self.exit()
            if next_state is not None:
                next_state.enter()
        return next_state


class DfStateSequence(DfState):
    """Represents a sequential state machine.

    On construction a sequence of states is provided. Each of those states should be terminating,
    i.e. they run with self transitions until completion and return None. This DfStateSequence
    object turns them into a sequential state machine by treating the terminal transitions as
    transitions to the next state in the sequence. Starting with the first state, it runs each in
    turn until termination, automatically transitioning to the next state in the sequence when the
    current state terminates.

    If loop is set to True, it loops back to the beginning once the final state has terminated.
    Otherwise, the higher level sequence state will terminate once it's finished a single pass
    through the sequence.

    Args:
        sequence: The sequence of terminating states which this class turns into a sequential state
            machine.
        loop: When True, loops back to the beginning of the sequence once the final state
            terminates. Otherwise, this state terminates once the final state in the sequence
            terminates. Defaults to False.
    """

    def __init__(self, sequence: Sequence[DfState], loop: Optional[bool] = False) -> None:
        super().__init__()
        self.sequence = sequence
        self.loop = loop
        self.state = None

    def __str__(self) -> str:
        """Return a string representation of the sequence and its current state.

        Returns:
            String representation of the object.
        """
        return f"{type(self).__name__}[{self.state}]"

    def bind(self, context: DfLogicalState, params: Any) -> None:
        """This method can be used to bind the underlying state to the given context and params.

        Both states the support bind() and those that don't can be used in a DfStateSequence. If
        it's supported, then bind() is called, otherwise it's ignored.

        Args:
            context: The context object to bind.
            params: The parameters to bind.
        """
        self.context = context
        self.params = params
        for state in self.sequence:
            if hasattr(state, "bind"):
                state.bind(context, params)

    def enter(self) -> None:
        """Entry into this state initializes the underlying state back to the first state in the.

        provided sequence.
        """
        if len(self.sequence) == 0:
            self.active_index = None
            self.state = None
            return

        self.active_index = 0
        self.state = self.sequence[self.active_index]
        self.state.enter()

    def step(self) -> DfState:
        """Stepping the state steps the internal chain state machine.

        When the current state terminates, it transitions to the next state in the provided
        sequence. If loop is True on construction, it transitions back to the first state once the
        last state in the sequence terminates. Otherwise, this state terminates.

        This state returns self transitions until it terminates (at which point it returns None).

        Returns:
            Self transition until termination. None on termination.

        """
        if self.state is None:
            return None

        next_state = self.state.step()
        if next_state is None:
            self.state.exit()

            self.active_index += 1
            if self.loop and self.active_index == len(self.sequence):
                self.active_index = 0

            if self.active_index < len(self.sequence):
                next_state = self.sequence[self.active_index]
                next_state.enter()

        self.state = next_state

        if self.state is not None:
            return self
        else:
            return None

    def exit(self) -> None:
        """Exit the sequence by exiting the currently active state."""
        if self.state is not None:
            self.state.exit()


class DfHierarchicalState(DfState):
    """A hierarchical state is a state that internally runs a separate state machine.

    The internal state machine is represented by an initial state entry point. The current internal
    state running is called the "active" state.

    The state machine resets back to the initial state every time enter() is called. Then calls to
    step() step the internal state machine making any needed transitions. On exit(), the active
    state is exited if there is one. Once the internal state machine ends (transitions to None),
    there is no longer an active state and calls to step() return None as well.

    Args:
        init_state: The starting state of the internal state machine.
    """

    def __init__(self, init_state: DfState) -> None:
        super().__init__()
        self.init_state = init_state
        self.active_state = None

    def __str__(self) -> str:
        """Return a string representation of this hierarchical state.

        Returns:
            String representation of the object.
        """
        return f"{type(self).__name__}[{self.active_state}]"

    def enter(self) -> None:
        """Entering the hierarchical state sets the active state back to the initial state provided.

        on construction.

        If there was a previously active state, that state is exited before we re-activate and enter
        the initial state.
        """
        if self.init_state is not None:
            if self.active_state is not None:
                self.active_state.exit()

            self.active_state = self.init_state
            self.active_state.enter()

    def step(self) -> DfState:
        """Stepping the hierarchical state steps the internal state machine.

        The hierarchical state transitions back to itelf while running the internal state machine.
        Once the internal state machine terminates, this state terminates as well by returning None.

        Returns:
            Self while the internal state machine is running, None when it terminates.
        """
        # If there's no active state transition to None (stop)
        if self.active_state is None:
            return None

        # Step the active state, processing transitions as needed using process_step(). The
        # higher-level state will self transition as long as there's an active internal state. Once
        # the internal state machine terminates, this state terminates as well (returns None).
        self.active_state = self.active_state.process_step()
        if self.active_state is not None:
            return self
        else:
            return None

    def exit(self) -> None:
        """Exit the state machine by exiting any active state."""
        if self.active_state is not None:
            self.active_state.exit()
            self.active_state = None


class DfHsmAction(DfAction):
    """Interfaces a Hierarchical State Machine (HSM) to a decision framework action so it can be.

    used as a DfAction leaf in the decider network.

    On enter, step, and exit, the HSM calls its own enter, step, and exit methods. Note that if the
    state machine exits (such as a hierarchical state's internal state machine finishes), it will
    keep calling step (and do nothing) until a higher-level decider decides not to run this action
    any more.

    The state machine is anything that's stepped until completion. E.g. HierarchicalState or
    SequenceState.

    Args:
        hsm: The state object stepped internally.
    """

    def __init__(self, hsm: DfState) -> None:
        super().__init__()
        self.hsm = hsm

    def __str__(self) -> str:
        """Return a string representation by delegating to the internal HSM.

        Returns:
            String representation of the object.
        """
        return self.hsm.__str__()

    def enter(self) -> None:
        """Pass through to the internal state's ``enter()``."""
        self.hsm.enter()

    def step(self) -> None:
        """Pass through to the internal state's ``step()``."""
        self.hsm.step()

    def exit(self) -> None:
        """Pass through to the internal state's ``exit()``."""
        self.hsm.exit()


class DfRate(ABC):
    """Abstract interface required by rate objects."""

    @abstractmethod
    def sleep(self) -> None:
        """Sleep for the appropriate duration between steps."""
        raise NotImplementedError()


class DfFastestRate(DfRate):
    """A rate class that simply loops as fast as possible."""

    def sleep(self) -> None:
        """Return immediately without sleeping."""


def run_state_machine(
    state: DfState,
    rate: DfRate,
    cb: Optional[Callable[[], None]] = None,
    is_shutdown_cb: Optional[Callable[[], bool]] = None,
) -> None:
    """Run the given state machine. Exits when there are no more transitions.

    Args:
        state: The starting state of the machine to step.
        rate: A DfRate object to handles the rate at which the state machien is stepped.
        cb: An optional callback called after each state machine step.
        is_shutdown_cb: An optional callback that returns whether the system's been shutdown.
    """
    hstate = DfHierarchicalState(init_state=state)
    hstate.enter()
    while is_shutdown_cb is None or not is_shutdown_cb():
        if hstate is None:
            return

        hstate = hstate.step()
        if cb:
            cb()
        rate.sleep()


class DfDeciderState(DfState):
    """A decider state is a state that's internally running a decider every step.

    The decider network is passed into this state as a reference to the root decider node. This
    state machine maintains it own decision stack data structure, which is reset every time enter()
    is called and used to call exit() in reverse order on all active decider nodes on exit(). The
    step() method calls df_descent starting from the passed in root decider.

    Args:
        decider: The internal decider node used as the root of the decider network stepped
            internally.
    """

    def __init__(self, decider: DfDecider) -> None:
        self.decider = decider
        self.stack = []

    def __str__(self) -> str:
        """Return a string showing the decider name and current decision stack.

        Returns:
            String representation of the object.
        """
        return f"{self.decider.name}[{'->'.join(str(i) for i in self.stack)}]"

    def bind(self, context: DfLogicalState, params: Any) -> None:
        """Binding a context and parameters to this state passes the information into the.

        underlying decider network.

        Uses the root decider's context and params members to store the information since
        df_descend() binds it there anyway at the start of the descent.

        Args:
            context: The context object being bound.
            params: The param object being bound.
        """
        self.decider.bind(context, params)

    def enter(self) -> None:
        """On entry to this state the decision stack is cleared. It's reset during the first step's.

        call to df_descend().
        """
        self.stack = []

    def step(self) -> DfState:
        """Step the state machine by descending the decider network. This state machine always.

        transitions back to itself.

        Returns:
            A reference to itself representing a self transition.
        """
        self.stack = df_descend(self.decider, self.decider.params, self.decider.context, self.stack)
        return self

    def exit(self) -> None:
        """On exit from this state, all active decision sessions from the decision stack are exited.

        in reverse order from leaf to the root.
        """
        if self.stack is not None:
            for node in reversed(self.stack):
                node.exit()


class DfTimedDeciderState(DfDeciderState):
    """A state which steps a decider network from its step() method for a predefined.

    activity_duration number of seconds.

    Note the number of seconds is measured in wallclock time using the system time.time().

    While the decider network is stepping, this state self transitions. Once activity_ duration
    seconds have passed, it transitions to None.

    Args:
        decider: The root decider node of the network stepped when stepping this state.
        activity_duration: How long to step the decider network in seconds.
    """

    def __init__(self, decider: DfDecider, activity_duration: float) -> None:
        super().__init__(decider)
        self.activity_duration = activity_duration

    def __str__(self) -> str:
        """Return a string showing the timed decider stack and activity duration.

        Returns:
            String representation of the object.
        """
        return f"TimedDecider[{'->'.join([str(i) for i in self.stack])}]({self.activity_duration})"

    def enter(self) -> None:
        """On entry, records the current time for measuring how long the internal decider network.

        has been stepped.
        """
        super().enter()
        self.entry_time = time.time()

    def step(self) -> DfState:
        """Steps the internal decider network until self.activity_duration seconds have passed.

        Returns:
            Self transitions until the requisite number of seconds have passed, then None
            (terminal transition).
        """
        next_state = super().step()
        elapse_time = time.time() - self.entry_time

        # If we're within the activity duration, then return the next state as usual. Otherwise,
        # return None to exit
        if elapse_time < self.activity_duration:
            return next_state
        else:
            return None


class DfWaitState(DfState):
    """This state waits a specified length of time before exiting.

    Args:
        wait_time: The number of seconds to wait.
    """

    def __init__(self, wait_time: float) -> None:
        super().__init__()
        self.wait_time = wait_time

    def __str__(self) -> str:
        """Return a string showing the wait time.

        Returns:
            String representation of the object.
        """
        return f"Wait({self.wait_time})"

    def enter(self) -> None:
        """Records the time on entry for measuring how much time has passed."""
        self.entry_time = time.time()

    def step(self) -> DfState:
        """Does nothing, but self transitions while waiting and terminal transitions (to None) once.

        the wait time has passed.

        Returns:
            Self while less than the wait time number of seconds have passed, and None
            otherwise.
        """
        if time.time() - self.entry_time < self.wait_time:
            return self
        else:
            return None

    def exit(self) -> None:
        """Clear internal bookkeeping state."""
        self.entry_time = None


class DfStateMachineDecider(DfDecider):
    """This decider steps a state machine each step during a given decision session. The state.

    machine can be any state machine, but if a given state has a bind() method, bind() will be
    called to give the state access to the context and current params.

    Args:
        state: The initial state of the internal state machine that will be run/
    """

    def __init__(self, state: DfState) -> None:
        super().__init__()
        self.init_state = state
        self.state = None

    def __str__(self) -> str:
        """Return a string showing the decider name and current state.

        Returns:
            String representation of the object.
        """
        return f"{self.name}[{self.state}]"

    def enter(self) -> None:
        """On entry, the state machine is reset back to the initial state."""
        self.state = self.init_state
        if self.state is not None:
            self._bind_state()
            self.state.enter()

    def decide(self) -> DfDecision | None:
        """On decide, the internal state machine is processed.

        Returns:
            None, as this decider processes an internal state machine rather than choosing children.
        """
        if self.state is None:
            return None

        self._bind_state()
        self.state = self.state.process_step()
        return None

    def exit(self) -> None:
        """On exit, the internal state machine is exited."""
        if self.state is not None:
            self._bind_state()
            self.state.exit()

    def _bind_state(self) -> None:
        """If the underlying state is bindable (has a bind() method), this decider's context and.

        parameters are bound to it. This internal method is called before each call to the
        underlying state's API methods so every API call has access to the decider's context and
        params.
        """
        if hasattr(self.state, "bind"):
            self.state.bind(self.context, self.params)


class DfSetLockState(DfState):
    """On entry, this state sets the given decider node's is_locked attribute to the specified.

    value.

    Args:
        set_locked_to: The boolean value to set the lock to on entry. True means locked, False means
            unlocked.
        decider: The decider node to be locked or unlocked.
    """

    def __init__(self, set_locked_to: bool, decider: DfDecider) -> None:
        self.set_locked_to = set_locked_to
        self.decider = decider

    def __str__(self) -> str:
        """Return a string showing the lock state and decider name.

        Returns:
            String representation of the object.
        """
        return f"SetLockState(set_locked_to:{self.set_locked_to}, {self.decider.name})"

    def enter(self) -> None:
        """On entry, sets the locked attribute of the specified decider node."""
        self.decider.is_locked = self.set_locked_to


class DfWriteContextState(DfState):
    """On entry, this state calls the specified write method, passing in the bound context.

    This state is useful for setting unobservable logical state that results from the execution of a
    given action. For instance, object alignment funneling behaviors (such as pinching a block on
    the table to better localize it in the gripper) aren't observable from perception due to the
    fundamental noise limits in perception (uncertainty). But we know it's been localized simply
    because the action's been performed.

    Args:
        write_method: A method which takes the context as input (often a member function of the
            context object). This is the method called on the context object on entry.
    """

    def __init__(self, write_method: Callable[[DfLogicalState], None]) -> None:
        self.write_method = write_method

    def __str__(self) -> str:
        """Return a string showing the write method name.

        Returns:
            String representation of the object.
        """
        return f"WriteContextState({self.write_method.__name__})"

    def enter(self) -> None:
        """Call the provided write method on entry."""
        self.write_method(self.context)


class DfBehavior(ABC):
    """Abstract base class for a behavior.

    A behavior is anything that has a step() and reset() method.
    """

    @abstractmethod
    def step(self) -> None:
        """Step the behavior. Steps generally at the rate of physics, which is often 60hz."""
        raise NotImplementedError()

    @abstractmethod
    def reset(self) -> None:
        """Reset a behavior back to its initial state."""
        raise NotImplementedError()


class DfNetwork(DfBehavior):
    """Represents the decider network.

    The topology of the decider network is defined by the acyclic graph structure of children of a
    root decider node provided on construction. On construction parameters can be passed in which
    are subsequently passed to the root on descent.

    A context object can be specified in multiple ways:
    1. Supplied on construction (which binds it to this object).
    2. Bound to the object explicitly through a call to bind_context(context).
    3. Supplied as input to a given call to step().

    If a context object is passed in to the step() method, it takes priority over any previously
    bound context objects. (Think of the bound context object as the default object.)

    Logical state monitors can be supplied on construction. If they are, they're processed each step
    before the descent through the decider network. Note it's often convenient to process the
    monitors elsewhere as part of an explicit behavior processing pipeline, so it's not manditory to
    pass them in here. In particular, monitors available as part of a given context object are never
    automatically called. They have to be explicitly passed in as monitors during construction.

    Note that the decider network is descended using a DfDeciderState object, so state machines that
    internally use deciders as DfDeciderState objects will process the subnetwork in the same way.
    Additionally, a DfStateMachineDecider whose internal state machine consists of DfDeciderState
    objects can be thought of as extending the decider network conditionally as a function of which
    state it's in.

    Args:
        root: The root decider node whose children recursively define the topology of this decider
            network.
        params: An optional set of parameters passed to the root.
        monitors: An optional sequence of logical state monitors. When specified these are processed
            before descending the decider network.
        context: An optional context object to bind to this decider network. If supplied, it'll be
            bound into all, and those nodes can access it through their context member.
            Alternatively, the context object can be supplied on each step() call.
    """

    def __init__(
        self,
        root: DfDecider,
        params: Optional[Any] = None,
        monitors: Optional[Sequence[LogicalStateMonitorType]] = None,
        context: Optional[DfLogicalState] = None,
    ) -> None:
        super().__init__()
        self._decider = root
        self._params = params

        self._monitors = monitors
        self._bound_context = context
        self._decider_state = DfDeciderState(self._decider)

        self.reset()

    def __str__(self) -> str:
        """Return a string showing the network type and decider state.

        Returns:
            String representation of the object.
        """
        return f"{type(self).__name__}[{self._decider_state}]"

    def reset(self) -> None:
        """Reset the decider network back to the state on construction (empty decision stack)."""
        self._decider_state.enter()

    def bind_context(self, context: DfLogicalState) -> None:
        """Bind the provided context to this decider network. This bound context will be bound into.

        each decider node on descent.

        Note that the bound context can be overridden by passing in a context object to the step()
        method, in which case that passed context object will be bound into each decider node on
        descent.

        Args:
            context: The context object to bind.
        """
        self._bound_context = context

    @property
    def context(self) -> DfLogicalState:
        """Returns the bound context object. This is the context object supplied either on.

        construction or through an explicit call to bind_context().

        Note that passing a context object to step() doesn't bind the context to this object and
        doesn't replace any existing bound context object.
        """
        return self._bound_context

    def step(self, context: Optional[DfLogicalState] = None) -> None:
        """Step this decider network once.

        Processes any logical state monitors passed in on construction first, then descends the
        decider network with a active context and any root parameters bound to the network. The active context is
        the context passed in here if supplied, or the bound context passed on construction or
        through an explicit call to bind_context().

        Args:
            context: An optional context object to bind to the decider network on descent. This
                context object is used in place of any default context bound to this object, but it
                doesn't replace the default bound context.
        """
        if context is None:
            if self._bound_context is not None:
                context = self._bound_context

        # Note the monitors are only processed if they're provided on construction.
        self._process_monitors(context)
        self._decider_state.bind(context, self._params)
        self._decider_state.step()

    def run(
        self, rate: DfRate, context: Optional[DfLogicalState], is_shutdown_cb: Optional[Callable[[], bool]] = None
    ) -> None:
        """Steps this decider network in a basic loop runner.

        One can optionally pass an is_shutdown_cb callback which can be used to terminate the loop
        runner. The loop runner will run as long as is_shutdown_cb() returns False, and terminate
        when it returns True. If no callback is supplied, it will loop forever.

        Args:
            rate: A rate object defining how quickly this loop will run.
            context: An optional context to supply to the step method.
            is_shutdown_cb: An optional callback defining when to terminate the loop runner.
        """
        while is_shutdown_cb is None or not is_shutdown_cb():
            self.step(context)
            rate.sleep()

    def _process_monitors(self, context: DfLogicalState) -> None:
        """Internal member for processing the logical state monitors passed in on construction.

        Monitors are called in the sequence defined on construction.

        Args:
            context: The context object to pass to each monitor function.
        """
        if self._monitors is not None:
            for monitor in self._monitors:
                monitor(context)


class DfRldsNode(DfDecider):
    """Represents a Robust Logical Dynamical System (RLDS) decision state.

    These RLDS's are added to a DfRldsDecider node as children. That node chooses among the children
    using the RLDS decision algorithm. Specifically, we can consider RLDS nodes to form a chain,
    with each node representing a behavior that can be run. Each node has an IsRunnable condition
    and an IsEnterable condition which can be different from IsRunnable but defaults to being the
    same.

    Note also that bind() is called from the DfRldsDecider before is_runnable() or is_enterable()
    are queried, so those methods have access to the decider node's context and current params.
    """

    def is_runnable(self) -> bool | None:
        """Override this method to implement the IsRunnable condition of the RLDS node.

        Returns:
            True if the node is runnable, False otherwise, or None if not overridden.
        """

    def is_enterable(self) -> bool | None:
        """Specify a slightly different enterable condition than the runnable condition.

        It's common to choose the enterable condition to be more stringent than the runnable
        condition so the system is robustly satisfying the runnable condition before the enterable
        condition is triggered.

        The enterable condition defaults to being equivalent to the runnable condition.

        Returns:
            True if the node is enterable, False otherwise.
        """
        return self.is_runnable()


class DfRldsDecider(DfDecider):
    """A decider node implementing the Robust Logical Dynamical System (RLDS) decision protocol.

    The RLDS decision algorithm starts every cycle at the end of the chain and traces up toward the
    beginning checking each node's IsEnterable condition. It chooses the first node it encounters
    whose IsEnterable condition returns true. That node is considered active, so on the next cycle
    it does the same, but checks instead this node's IsRunnable condition.

    Intuitively, the goal of each node's associated policy (which amounts to the ultimate choice of
    action it makes in the decider network under it) is to push the system toward triggering the
    IsEnterable condition of the next node in the chain. These systems are fundamentally reactive
    and if ever a more distal node becomes enterable for any reason, it immediately transitions to
    it.

    See the Robust Logical Dynamical Systems paper for an in-depth description of these systems:

        Representing Robot Task Plans as Robust Logical-Dynamical Systems
        Chris Paxton, Nathan Ratliff, Clemens Eppner, Dieter Fox
        https://arxiv.org/abs/1908.01896

    The theory proves conditions under which this system is guaranteed to push this purely reactive
    system toward a goal condition.

    The RLDS decider node implements this algorithm as a sequence of child decider nodes, ordered in
    order of increasing priority, each of which are DfRldsNode objects with is_runnable() and
    is_enterable() methods. A call to decide() steps from the highest priority node to the lowest,
    checking is_enterable() on each (or is_runnable() if it's already running the decision), and
    simply chooses the first that returns true.

    Note that distal nodes handling necessary preconditions can be tacked on as higher-priority
    nodes beyond the goal in the chain. For instance, a chain that needs the gripper to be open can
    do this so if it finds the gripper to be closed it'll always automatically be triggered. These
    ideas are discussed in the paper, although the paper was a pre-cursor to decider networks, and
    with the more general decider networks it's often easier to implement those preconditions using
    a different decision hierarchy, including by adding them as decorator nodes of sort over the top
    of the DfRldsDecider.
    """

    def __init__(self) -> None:
        super().__init__()
        self.sequence = []

    def __str__(self) -> str:
        """Return the class name of this RLDS decider.

        Returns:
            String representation of the object.
        """
        return type(self).__name__

    class NamedRldsNode:
        """An internal class that packages the name together with the RLDS node.

        Args:
            name: The name of the node.
            rlds_node: The child decider node representing an RLDS node.
        """

        def __init__(self, name: str, rlds_node: DfRldsNode) -> None:
            self.name = name
            self.rlds_node = rlds_node

        def __str__(self) -> str:
            """Return the name of this RLDS node.

            Returns:
                String representation of the object.
            """
            return self.name

    def append_rlds_node(self, name: str, rlds_node: DfRldsNode) -> None:
        """Append the named RLDS node to the end of then chain (highest priority).

        Args:
            name: The name of the node.
            rlds_node: The child decider node representing the RLDS node being added.
        """
        self.sequence.append(DfRldsDecider.NamedRldsNode(name, rlds_node))
        self.add_child(name, rlds_node)

    def enter(self) -> None:
        """Entry into this decision session reset the internal RLDS state so there's no recorded.

        previous node. On the first call to decide() all nodes will be checked for their enterable
        condition (no runnable condition because there's no active node).
        """
        self.prev_node = None

    def decide(self) -> DfDecision:
        """Decides on a child using the RLDS decision algorithm.

        Traces in reverse along the chain of nodes from the last node (highest priority) toward the
        first node (lowest priority). If the node is the active node, it's runnability condition is
        checked, otherwise the node's enterability condition is checked. Chooses the first node
        (highest priority node) whose condition is satisfied.

        Returns:
            The decision for the chosen child, or None if no condition is satisfied.
        """
        for named_rlds_node in reversed(self.sequence):
            rlds_node = named_rlds_node.rlds_node
            rlds_node.bind(self.context, self.params)
            if rlds_node == self.prev_node:
                is_active = rlds_node.is_runnable
            else:
                is_active = rlds_node.is_enterable
            self.prev_node = rlds_node

            if is_active():
                return DfDecision(named_rlds_node.name)

        return None
