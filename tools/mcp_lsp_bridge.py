#!/usr/bin/env python.exe
"""
Lightweight MCP server that bridges selected LSP operations for one language.

Usage:
  python.exe mcp_lsp_bridge.py --language cpp
  python.exe mcp_lsp_bridge.py --language python
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional


SERVER_INFO = {
    "name": "codex-lsp-bridge",
    "version": "0.1.0",
}


LANG_CONFIG = {
    "cpp": {
        "lsp_command": "clangd",
        "lsp_args": ["--log=error"],
        "language_id": "cpp",
        "tool_prefix": "cpp",
    },
    "python": {
        "lsp_command": "pyright-langserver",
        "lsp_args": ["--stdio"],
        "language_id": "python",
        "tool_prefix": "py",
    },
}


def log(message: str) -> None:
    print(f"[mcp-lsp-bridge] {message}", flush=True, file=os.sys.stderr)


def path_to_uri(path: str) -> str:
    return Path(path).resolve().as_uri()


def read_content_length(stream) -> Optional[int]:
    headers: Dict[str, str] = {}
    while True:
        line = stream.readline()
        if not line:
            return None
        if isinstance(line, bytes):
            line = line.decode("utf-8", errors="replace")
        line = line.strip()
        if not line:
            break
        if ":" in line:
            key, value = line.split(":", 1)
            headers[key.strip().lower()] = value.strip()
    value = headers.get("content-length")
    if value is None:
        return None
    return int(value)


def read_protocol_message(stream) -> Optional[Dict[str, Any]]:
    length = read_content_length(stream)
    if length is None:
        return None
    content = stream.read(length)
    if not content:
        return None
    if isinstance(content, bytes):
        content = content.decode("utf-8", errors="replace")
    return json.loads(content)


def write_protocol_message(stream, payload: Dict[str, Any]) -> None:
    encoded = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    header = f"Content-Length: {len(encoded)}\r\n\r\n".encode("utf-8")
    stream.write(header)
    stream.write(encoded)
    stream.flush()


class LspClient:
    def __init__(self, command: str, args: list[str], language_id: str) -> None:
        self.command = command
        self.args = args
        self.language_id = language_id
        self.proc: Optional[subprocess.Popen] = None
        self.msg_queue: queue.Queue[Dict[str, Any]] = queue.Queue()
        self.reader_thread: Optional[threading.Thread] = None
        self.request_id = 1
        self.diagnostics_by_uri: Dict[str, Any] = {}

    def start(self) -> None:
        self.proc = subprocess.Popen(
            [self.command, *self.args],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0,
        )
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

    def _reader_loop(self) -> None:
        assert self.proc is not None
        assert self.proc.stdout is not None
        while True:
            try:
                message = read_protocol_message(self.proc.stdout)
                if message is None:
                    break
                method = message.get("method")
                if method == "textDocument/publishDiagnostics":
                    params = message.get("params", {})
                    uri = params.get("uri")
                    if isinstance(uri, str):
                        self.diagnostics_by_uri[uri] = params.get("diagnostics", [])
                self.msg_queue.put(message)
            except Exception as exc:
                log(f"LSP reader error: {exc}")
                break

    def stop(self) -> None:
        if self.proc is None:
            return
        try:
            self.notify("exit", None)
        except Exception:
            pass
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None

    def notify(self, method: str, params: Optional[Dict[str, Any]]) -> None:
        assert self.proc is not None and self.proc.stdin is not None
        payload = {"jsonrpc": "2.0", "method": method}
        if params is not None:
            payload["params"] = params
        write_protocol_message(self.proc.stdin, payload)

    def request(self, method: str, params: Dict[str, Any], timeout: float = 30.0) -> Any:
        assert self.proc is not None and self.proc.stdin is not None
        req_id = self.request_id
        self.request_id += 1
        payload = {
            "jsonrpc": "2.0",
            "id": req_id,
            "method": method,
            "params": params,
        }
        write_protocol_message(self.proc.stdin, payload)
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = max(0.01, deadline - time.time())
            try:
                message = self.msg_queue.get(timeout=remaining)
            except queue.Empty:
                continue
            if message.get("id") == req_id:
                if "error" in message:
                    raise RuntimeError(json.dumps(message["error"], ensure_ascii=False))
                return message.get("result")
        raise TimeoutError(f"LSP request timed out: {method}")

    def initialize(self, root_uri: str) -> None:
        self.request(
            "initialize",
            {
                "processId": None,
                "clientInfo": {"name": "codex-lsp-bridge", "version": "0.1.0"},
                "rootUri": root_uri,
                "capabilities": {},
                "workspaceFolders": [{"uri": root_uri, "name": "workspace"}],
            },
            timeout=20.0,
        )
        self.notify("initialized", {})

    def open_document(self, file_path: str) -> str:
        uri = path_to_uri(file_path)
        text = Path(file_path).read_text(encoding="utf-8", errors="replace")
        self.notify(
            "textDocument/didOpen",
            {
                "textDocument": {
                    "uri": uri,
                    "languageId": self.language_id,
                    "version": 1,
                    "text": text,
                }
            },
        )
        return uri


def format_tool_text(title: str, payload: Any) -> str:
    return f"{title}\n{json.dumps(payload, ensure_ascii=False, indent=2)}"


def mcp_success(text: str) -> Dict[str, Any]:
    return {"content": [{"type": "text", "text": text}], "isError": False}


def mcp_error(text: str) -> Dict[str, Any]:
    return {"content": [{"type": "text", "text": text}], "isError": True}


def tool_specs(prefix: str) -> list[Dict[str, Any]]:
    base_props = {
        "file_path": {"type": "string", "description": "Absolute or relative file path"},
        "line": {"type": "integer", "minimum": 0, "description": "0-based line"},
        "character": {"type": "integer", "minimum": 0, "description": "0-based character"},
    }
    return [
        {
            "name": f"{prefix}_hover",
            "description": f"{prefix.upper()} LSP hover at a position",
            "inputSchema": {
                "type": "object",
                "properties": base_props,
                "required": ["file_path", "line", "character"],
                "additionalProperties": False,
            },
        },
        {
            "name": f"{prefix}_definition",
            "description": f"{prefix.upper()} LSP go-to-definition at a position",
            "inputSchema": {
                "type": "object",
                "properties": base_props,
                "required": ["file_path", "line", "character"],
                "additionalProperties": False,
            },
        },
        {
            "name": f"{prefix}_completion",
            "description": f"{prefix.upper()} LSP completion at a position",
            "inputSchema": {
                "type": "object",
                "properties": {
                    **base_props,
                    "max_items": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 100,
                        "default": 20,
                    },
                },
                "required": ["file_path", "line", "character"],
                "additionalProperties": False,
            },
        },
        {
            "name": f"{prefix}_diagnostics",
            "description": f"{prefix.upper()} LSP diagnostics for a file",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "file_path": {"type": "string", "description": "Absolute or relative file path"},
                    "wait_ms": {
                        "type": "integer",
                        "minimum": 100,
                        "maximum": 5000,
                        "default": 1500,
                    },
                },
                "required": ["file_path"],
                "additionalProperties": False,
            },
        },
    ]


def run_lsp_tool(language: str, tool_name: str, arguments: Dict[str, Any]) -> Dict[str, Any]:
    cfg = LANG_CONFIG[language]
    prefix = cfg["tool_prefix"]
    command = str(cfg["lsp_command"])
    args = [str(x) for x in cfg["lsp_args"]]
    file_path = os.path.abspath(str(arguments.get("file_path", "")))
    if not os.path.isfile(file_path):
        return mcp_error(f"File not found: {file_path}")

    root_uri = Path(file_path).resolve().parent.as_uri()
    client = LspClient(command, args, str(cfg["language_id"]))
    try:
        client.start()
        client.initialize(root_uri=root_uri)
        uri = client.open_document(file_path)

        if tool_name == f"{prefix}_hover":
            result = client.request(
                "textDocument/hover",
                {
                    "textDocument": {"uri": uri},
                    "position": {"line": int(arguments["line"]), "character": int(arguments["character"])},
                },
            )
            return mcp_success(format_tool_text("hover", result))

        if tool_name == f"{prefix}_definition":
            result = client.request(
                "textDocument/definition",
                {
                    "textDocument": {"uri": uri},
                    "position": {"line": int(arguments["line"]), "character": int(arguments["character"])},
                },
            )
            return mcp_success(format_tool_text("definition", result))

        if tool_name == f"{prefix}_completion":
            result = client.request(
                "textDocument/completion",
                {
                    "textDocument": {"uri": uri},
                    "position": {"line": int(arguments["line"]), "character": int(arguments["character"])},
                    "context": {"triggerKind": 1},
                },
            )
            max_items = int(arguments.get("max_items", 20))
            items = []
            if isinstance(result, dict) and isinstance(result.get("items"), list):
                items = result["items"][:max_items]
            elif isinstance(result, list):
                items = result[:max_items]
            return mcp_success(format_tool_text("completion", {"items": items}))

        if tool_name == f"{prefix}_diagnostics":
            wait_ms = int(arguments.get("wait_ms", 1500))
            time.sleep(wait_ms / 1000.0)
            diagnostics = client.diagnostics_by_uri.get(uri, [])
            return mcp_success(format_tool_text("diagnostics", diagnostics))

        return mcp_error(f"Unsupported tool: {tool_name}")
    except Exception as exc:
        return mcp_error(f"LSP tool failed: {exc}")
    finally:
        client.stop()


def handle_initialize(request_id: Any, params: Dict[str, Any]) -> Dict[str, Any]:
    requested_protocol = params.get("protocolVersion")
    if not isinstance(requested_protocol, str) or not requested_protocol:
        requested_protocol = "2024-11-05"
    return {
        "jsonrpc": "2.0",
        "id": request_id,
        "result": {
            "protocolVersion": requested_protocol,
            "capabilities": {
                "tools": {"listChanged": False},
                "resources": {"subscribe": False, "listChanged": False},
                "prompts": {"listChanged": False},
            },
            "serverInfo": SERVER_INFO,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--language", choices=("cpp", "python"), required=True)
    parsed = parser.parse_args()

    cfg = LANG_CONFIG[parsed.language]
    prefix = cfg["tool_prefix"]
    specs = tool_specs(prefix)

    input_stream = os.sys.stdin.buffer
    output_stream = os.sys.stdout.buffer

    while True:
        msg = read_protocol_message(input_stream)
        if msg is None:
            break

        msg_id = msg.get("id")
        method = msg.get("method")
        params = msg.get("params", {}) or {}

        if method == "initialize":
            write_protocol_message(output_stream, handle_initialize(msg_id, params))
            continue

        if method == "notifications/initialized":
            continue

        if method == "ping" and msg_id is not None:
            write_protocol_message(output_stream, {"jsonrpc": "2.0", "id": msg_id, "result": {}})
            continue

        if method == "tools/list" and msg_id is not None:
            write_protocol_message(output_stream, {"jsonrpc": "2.0", "id": msg_id, "result": {"tools": specs}})
            continue

        if method == "tools/call" and msg_id is not None:
            tool_name = params.get("name")
            arguments = params.get("arguments", {}) or {}
            result = run_lsp_tool(parsed.language, str(tool_name), arguments)
            write_protocol_message(output_stream, {"jsonrpc": "2.0", "id": msg_id, "result": result})
            continue

        if method == "resources/list" and msg_id is not None:
            write_protocol_message(output_stream, {"jsonrpc": "2.0", "id": msg_id, "result": {"resources": []}})
            continue

        if method == "resources/templates/list" and msg_id is not None:
            write_protocol_message(
                output_stream, {"jsonrpc": "2.0", "id": msg_id, "result": {"resourceTemplates": []}}
            )
            continue

        if method == "shutdown" and msg_id is not None:
            write_protocol_message(output_stream, {"jsonrpc": "2.0", "id": msg_id, "result": {}})
            continue

        if method == "exit":
            break

        if msg_id is not None:
            write_protocol_message(
                output_stream,
                {
                    "jsonrpc": "2.0",
                    "id": msg_id,
                    "error": {"code": -32601, "message": f"Method not found: {method}"},
                },
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
