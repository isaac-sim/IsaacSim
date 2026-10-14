#!/usr/bin/env node
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Replace the scaffold signalingServer literal with JSON-serialized bind values.
// Reads ISAACSIM_HOST / ISAACSIM_SIGNAL_PORT / ISAACSIM_STREAM_PORT from the
// environment so Docker ARG values are data, not shell/sed program text.

import fs from "fs";

const path = process.argv[2];
const host = process.env.ISAACSIM_HOST || "";
const signalPort = process.env.ISAACSIM_SIGNAL_PORT || "";
const streamPort = process.env.ISAACSIM_STREAM_PORT || "";

const unsafe = /[\s"'\\;`$|&<>(){}\[\]#\n\r]/;
const hostnameRe =
  /^(?=.{1,253}$)[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?(?:\.[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?)*$/;

function isValidHost(value) {
  if (!value || unsafe.test(value)) {
    return false;
  }
  if (["0.0.0.0", "*", "::", "[::]", "localhost"].includes(value)) {
    return false;
  }
  if (/^(\d{1,3}\.){3}\d{1,3}$/.test(value)) {
    const parts = value.split(".").map((part) => Number(part));
    if (parts.some((n, i) => !/^(0|[1-9][0-9]{0,2})$/.test(value.split(".")[i]) || n < 0 || n > 255)) {
      return false;
    }
    const first = parts[0];
    if (first === 0 || (first >= 224 && first <= 239)) {
      return false;
    }
    return true;
  }
  return hostnameRe.test(value);
}

function isValidPort(value) {
  if (!/^[1-9][0-9]{0,4}$/.test(value)) {
    return false;
  }
  const n = Number(value);
  return n >= 1 && n <= 65535;
}

if (!path) {
  console.error("FAIL: patch-stream-config.mjs <src/main.tsx>");
  process.exit(2);
}
if (!isValidHost(host)) {
  console.error("FAIL: ISAACSIM_HOST is not a bindable IPv4 or DNS name");
  process.exit(2);
}
if (!isValidPort(signalPort) || !isValidPort(streamPort)) {
  console.error("FAIL: ISAACSIM_SIGNAL_PORT / ISAACSIM_STREAM_PORT invalid");
  process.exit(2);
}

const text = fs.readFileSync(path, "utf8");
const needle = "signalingServer: '127.0.0.1',";
if (!text.includes(needle)) {
  console.error(`FAIL: expected ${needle} in ${path}`);
  process.exit(1);
}

const hostLit = JSON.stringify(host);
const replacement =
  `signalingServer: ${hostLit},\n` +
  `    signalingPort: ${Number(signalPort)},\n` +
  `    mediaServer: ${hostLit},\n` +
  `    mediaPort: ${Number(streamPort)},\n` +
  `    forceWSS: false,`;

fs.writeFileSync(path, text.replace(needle, replacement));
