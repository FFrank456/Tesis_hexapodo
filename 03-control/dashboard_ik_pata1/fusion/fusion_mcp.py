"""Cliente mínimo del MCP de Fusion (servidor local "MCP Server Adapter", HTTP en 127.0.0.1).

Solo se usan scripts de SOLO LECTURA (readOnly=True): no modifican el diseño.
"""
import json
import urllib.request


class FusionMCP:
    def __init__(self, url="http://127.0.0.1:53157/mcp"):
        self.url, self.sid, self.n = url, None, 0
        self._call("initialize", {"protocolVersion": "2025-03-26", "capabilities": {},
                                  "clientInfo": {"name": "dashboard_ik_pata1", "version": "1"}})
        self._post({"jsonrpc": "2.0", "method": "notifications/initialized"})

    def _post(self, body):
        h = {"Content-Type": "application/json", "Accept": "application/json, text/event-stream"}
        if self.sid:
            h["Mcp-Session-Id"] = self.sid
        req = urllib.request.Request(self.url, data=json.dumps(body).encode(), headers=h)
        with urllib.request.urlopen(req, timeout=600) as r:
            self.sid = r.headers.get("Mcp-Session-Id") or self.sid
            txt = r.read().decode("utf-8")
        if "data:" in txt[:50]:
            txt = "".join(l[5:] for l in txt.splitlines() if l.startswith("data:"))
        return json.loads(txt) if txt.strip() else None

    def _call(self, method, params):
        self.n += 1
        r = self._post({"jsonrpc": "2.0", "id": self.n, "method": method, "params": params})
        if "error" in r:
            raise RuntimeError(r["error"])
        return r["result"]

    def tool(self, name, args):
        res = self._call("tools/call", {"name": name, "arguments": args})
        texts = [c.get("text", "") for c in res.get("content", [])]
        return "\n".join(texts)

    def run_readonly(self, script):
        """Ejecuta un script de Fusion (def run(_context)) y devuelve lo que imprimió."""
        r = json.loads(self.tool("fusion_mcp_execute",
                                 {"featureType": "script", "object": {"script": script, "readOnly": True}}))
        if not r.get("success"):
            raise RuntimeError(r.get("error") or r)
        return r.get("message", "")

    def active_document(self):
        r = json.loads(self.tool("fusion_mcp_read", {"queryType": "document", "operation": "open"}))
        act = [d for d in r.get("results", []) if d.get("isActive")]
        return act[0] if act else None
