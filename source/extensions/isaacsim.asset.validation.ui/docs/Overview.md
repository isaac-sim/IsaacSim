# SimReady Profile Validation UI

The SimReady Profile Validation UI provides a guided workflow for validating the current USD stage or a selected USD
asset against the profiles bundled with the SimReady foundation tiers.

Open **Window > SimReady Asset Validation**, choose an asset source and profile, and select **Validate**. Results are
grouped by requirement and include pass, failure, warning, error, and not-implemented states. Failed checks include the
validator message, affected USD location, requirement guidance, and available repair suggestions.

Fixes are available only when a validator supplies an existing fix callback and a valid edit target. Fixes apply only
to the current stage and require explicit confirmation. The extension does not automatically save the stage.

## Profiles

The profile selector uses readable labels while retaining the bundled profile IDs internally:

- `Robot-Body-Isaac` displays as **Robot Assets**
- `Prop-Robotics-Isaac` displays as **Props**

Configure `exts."isaacsim.asset.validation.ui".profileFilter` with profile IDs to exclude from the selector. The
default excludes every bundled profile except `Robot-Body-Isaac` and `Prop-Robotics-Isaac`, so the UI shows only
**Robot Assets** and **Props**. Set the list to `[]` to show every bundled profile.

Override selector labels with `exts."isaacsim.asset.validation.ui".profileNames.<Profile-Id>`. Unlisted IDs keep the
generated pretty name.

Set `exts."isaacsim.asset.validation.ui".docsUrl` to change the base URL opened by **Open requirement docs**.

## User Guide

See the Isaac Sim user guide page **Robot Setup > Asset Validation** for screenshots, the Robot Assets procedure, and a
control reference for every button in the window.
