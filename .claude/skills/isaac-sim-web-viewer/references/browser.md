# Keyboard shortcuts and clipboard

| Action | Windows / Linux | Mac |
|---|---|---|
| Copy / paste | Ctrl+C / Ctrl+V | Ctrl+C / Ctrl+V |
| Refresh the browser page | F5 or Ctrl+R | Fn+F5 or Cmd+R |
| Maximize viewport in Isaac Sim | F7 | Fn+F7 |
| Toggle browser fullscreen | F11 | Shift+Fn+F11 |
| Open DevTools | F12 | Fn+F12 or Cmd+Option+I |

The browser Clipboard API requires a [secure context](https://developer.mozilla.org/en-US/docs/Web/Security/Secure_Contexts). `http://127.0.0.1:8210` on this machine is a secure context; use Chromium **on the Isaac Sim host** when clipboard forwarding matters.

This skill serves the viewer over **plaintext HTTP with no TLS**. A LAN URL such as `http://<lan-ip>:8210` is not a secure context, so Chromium blocks clipboard forwarding. Do not work around that with `chrome://flags/#unsafely-treat-insecure-origin-as-secure`: that flag treats an unauthenticated HTTP origin as secure and weakens an important browser boundary (an on-path LAN attacker who serves that origin can then use secure-context APIs, including clipboard, subject to Chrome's remaining permission checks).

Do not put a TLS reverse proxy in front of port **8210 only**. The generated client sets `forceWSS: false` and opens a **plaintext** WebSocket to `ISAACSIM_SIGNAL_PORT` (default **49100**) plus UDP media on `ISAACSIM_STREAM_PORT` (default **47998**). Loading the page over HTTPS then makes Chromium treat `ws://HOST:<signaling-port>` as mixed content, so the stream never starts. This skill does not terminate TLS or WSS, and it does not rebuild the client with `forceWSS: true`. Clipboard from another computer is therefore unsupported on this stack; do not publish the viewer on the public Internet.
