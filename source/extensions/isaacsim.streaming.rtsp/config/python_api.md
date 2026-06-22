# Public API for module isaacsim.streaming.rtsp:

## Classes

- class RTSPStreamingExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

- class RTSPStreamWriter(Writer)
  - def __init__(self, port: int = 8554, mountPath: str = '/stream', encoding: _RTSPWriterEncoding = 'h264', width: int = 1920, height: int = 1080)
  - def write(self, data: dict)
  - def detach(self)
