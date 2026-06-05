# Overview

The isaacsim.pip.newton extension bundles pip packages required by the Newton physics engine integration in Isaac Sim. It is loaded early in the extension startup order to ensure Newton dependencies are available before any physics extensions that depend on them.

This extension is managed automatically and does not require manual configuration.
