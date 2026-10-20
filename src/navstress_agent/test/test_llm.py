"""LLM client tests. The HTTP client is exercised against a local stub server on
127.0.0.1; no test talks to a real LLM provider."""

import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from navstress_agent.llm import (
    LLMError,
    MockLLM,
    OpenAICompatibleClient,
    extract_json,
    read_context,
)


class _Stub(BaseHTTPRequestHandler):
    requests: list = []
    status = 200

    def log_message(self, *args):
        pass

    def _reply(self, payload):
        body = json.dumps(payload).encode()
        self.send_response(self.status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        length = int(self.headers["Content-Length"])
        body = json.loads(self.rfile.read(length))
        _Stub.requests.append((self.path, dict(self.headers), body))
        if self.status != 200:
            return self._reply({"error": {"message": "quota exceeded"}})
        self._reply({"choices": [{"message": {"role": "assistant", "content": '{"ok": 1}'}}]})

    def do_GET(self):
        _Stub.requests.append((self.path, dict(self.headers), None))
        self._reply({"data": [{"id": "model-b"}, {"id": "model-a"}]})


@pytest.fixture
def stub():
    _Stub.requests = []
    _Stub.status = 200
    server = HTTPServer(("127.0.0.1", 0), _Stub)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    yield f"http://127.0.0.1:{server.server_port}/v1beta/openai/"
    server.shutdown()


def test_chat_completion_request_shape(stub):
    client = OpenAICompatibleClient(base_url=stub, api_key="k-123", model="m-1")
    out = client.complete([{"role": "user", "content": "hi"}])
    assert json.loads(out) == {"ok": 1}
    path, headers, body = _Stub.requests[0]
    assert path == "/v1beta/openai/chat/completions"
    assert headers["Authorization"] == "Bearer k-123"
    assert body["model"] == "m-1" and body["messages"][0]["content"] == "hi"
    assert body["response_format"] == {"type": "json_object"}


def test_json_mode_can_be_disabled(stub):
    client = OpenAICompatibleClient(base_url=stub, api_key="k", model="m", json_mode=False)
    client.complete([{"role": "user", "content": "hi"}])
    assert "response_format" not in _Stub.requests[0][2]


def test_list_models(stub):
    client = OpenAICompatibleClient(base_url=stub, api_key="k", model="m")
    assert client.list_models() == ["model-a", "model-b"]
    assert _Stub.requests[0][0] == "/v1beta/openai/models"


def test_http_error_becomes_llm_error_with_detail(stub):
    _Stub.status = 429
    client = OpenAICompatibleClient(base_url=stub, api_key="k", model="m")
    with pytest.raises(LLMError, match="HTTP 429.*quota exceeded"):
        client.complete([{"role": "user", "content": "hi"}])


def test_unreachable_endpoint_becomes_llm_error():
    client = OpenAICompatibleClient(
        base_url="http://127.0.0.1:9/", api_key="k", model="m", timeout_s=2
    )
    with pytest.raises(LLMError, match="failed"):
        client.complete([{"role": "user", "content": "hi"}])


def test_from_env_requires_key_and_model():
    with pytest.raises(LLMError, match="NAVSTRESS_LLM_API_KEY"):
        OpenAICompatibleClient.from_env({"NAVSTRESS_LLM_MODEL": "m"})
    c = OpenAICompatibleClient.from_env(
        {"NAVSTRESS_LLM_API_KEY": "k", "NAVSTRESS_LLM_MODEL": "m", "NAVSTRESS_LLM_JSON_MODE": "0"}
    )
    assert c.base_url.startswith("https://generativelanguage.googleapis.com/")
    assert c.json_mode is False


@pytest.mark.parametrize(
    "text",
    [
        '{"a": 1}',
        'Sure! ```json\n{"a": 1}\n``` hope that helps',
        'Here you go: {"a": 1} -- done',
    ],
)
def test_extract_json_tolerates_fences_and_prose(text):
    assert extract_json(text) == {"a": 1}


def test_extract_json_rejects_non_json():
    with pytest.raises(ValueError):
        extract_json("I cannot help with that.")


def test_mock_requires_context_line():
    with pytest.raises(ValueError, match="CONTEXT_JSON"):
        MockLLM().complete([{"role": "user", "content": "no context"}])
    assert read_context('x\nCONTEXT_JSON: {"round": 2}\n') == {"round": 2}
