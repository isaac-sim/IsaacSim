```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

### app.runLoopsGlobal.syncToPresent
- **Default Value**: false
- **Description**: Do not sync threads to the present thread.

### app.runLoops.main.rateLimitEnabled
- **Default Value**: true
- **Description**: Set to `true` to enable rate limiting for the main run loop.

### app.runLoops.main.rateLimitFrequency
- **Default Value**: 120
- **Description**: Rate-limit frequency in Hz for the main run loop.

### app.runLoops.main.rateLimitUseBusyLoop
- **Default Value**: false
- **Description**: Set to `true` to use a busy loop for the main run loop.

### app.runLoops.present.rateLimitEnabled
- **Default Value**: true
- **Description**: Set to `true` to enable rate limiting for the present run loop.

### app.runLoops.present.rateLimitFrequency
- **Default Value**: 60
- **Description**: Rate-limit frequency in Hz for the present run loop.

### app.runLoops.present.rateLimitUseBusyLoop
- **Default Value**: false
- **Description**: Set to `true` to use a busy loop for the present run loop.

### app.runLoops.rendering_0.rateLimitEnabled
- **Default Value**: true
- **Description**: Set to `true` to enable rate limiting for the rendering run loop.

### app.runLoops.rendering_0.rateLimitFrequency
- **Default Value**: 120
- **Description**: Rate-limit frequency in Hz for the rendering run loop.

### app.runLoops.rendering_0.rateLimitUseBusyLoop
- **Default Value**: false
- **Description**: Set to `true` to use a busy loop for the rendering run loop.
