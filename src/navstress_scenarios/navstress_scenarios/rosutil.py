"""Blocking service calls from inside callbacks, with timeouts.

Callbacks that call other services run on a MultiThreadedExecutor with a reentrant
callback group; the reply is delivered on another executor thread, so polling the
future here doesn't deadlock. rclpy futures have no timeout of their own.
"""

from __future__ import annotations

import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.client import Client
from rclpy.node import Node


class CallError(RuntimeError):
    pass


def wait_future(fut, timeout_s: float, what: str):
    deadline = time.monotonic() + timeout_s
    while not fut.done():
        if time.monotonic() > deadline:
            fut.cancel()
            raise CallError(f"{what}: no reply within {timeout_s:.1f} s")
        time.sleep(0.005)
    exc = fut.exception()
    if exc is not None:
        raise CallError(f"{what}: {exc}")
    return fut.result()


def call(client: Client, request, timeout_s: float):
    name = client.srv_name
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise CallError(f"service {name} is not available")
    return wait_future(client.call_async(request), timeout_s, f"service {name}")


def to_parameter_value(value) -> ParameterValue:
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    if isinstance(value, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
    if isinstance(value, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    if isinstance(value, str):
        return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    raise TypeError(f"unsupported parameter type {type(value).__name__}")


def from_parameter_value(v: ParameterValue):
    return {
        ParameterType.PARAMETER_BOOL: lambda: v.bool_value,
        ParameterType.PARAMETER_INTEGER: lambda: v.integer_value,
        ParameterType.PARAMETER_DOUBLE: lambda: v.double_value,
        ParameterType.PARAMETER_STRING: lambda: v.string_value,
    }.get(v.type, lambda: None)()


class RemoteParams:
    """Get/set parameters on another node through its standard parameter services."""

    def __init__(self, node: Node, target: str, callback_group, timeout_s: float):
        self.target = target.rstrip("/")
        self.timeout_s = timeout_s
        self._set = node.create_client(
            SetParameters, f"{self.target}/set_parameters", callback_group=callback_group
        )
        self._get = node.create_client(
            GetParameters, f"{self.target}/get_parameters", callback_group=callback_group
        )

    def available(self, timeout_s: float = 0.5) -> bool:
        return self._set.wait_for_service(timeout_sec=timeout_s)

    def get(self, names: list[str]) -> dict:
        resp = call(self._get, GetParameters.Request(names=names), self.timeout_s)
        return {n: from_parameter_value(v) for n, v in zip(names, resp.values, strict=True)}

    def set(self, values: dict) -> None:
        req = SetParameters.Request(
            parameters=[Parameter(name=k, value=to_parameter_value(v)) for k, v in values.items()]
        )
        resp = call(self._set, req, self.timeout_s)
        bad = [
            f"{k}: {r.reason or 'rejected'}"
            for k, r in zip(values, resp.results, strict=True)
            if not r.successful
        ]
        if bad:
            raise CallError(f"{self.target} rejected parameters: {'; '.join(bad)}")
