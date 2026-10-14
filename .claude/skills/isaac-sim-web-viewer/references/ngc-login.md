# NGC login prompt

Use this only when `scripts/ngc_login.sh --check` reports missing `nvcr.io` credentials and no key is already in the environment.

Do **not** ask the operator to paste an NGC API key into chat or into a tool call.

> NGC credentials are not present for `nvcr.io`, so the prebuilt Isaac Sim image cannot be pulled.
>
> Do not send the API key in this conversation.
>
> On this machine, authenticate locally:
> 1. Open https://ngc.nvidia.com/setup/api-key if you need a key
> 2. Run `docker login nvcr.io` yourself (username `$oauthtoken`; paste the key only at that docker prompt)
> 3. Or export `NGC_API_KEY` in your local shell (not in chat) and say that login is ready
>
> Tell me when `scripts/ngc_login.sh --check` prints `OK`.

Stop until `scripts/ngc_login.sh --check` succeeds. If a key is already in the environment, run `scripts/ngc_login.sh` (stdin login only; never put the key on the command line, in logs, or in chat). Unset the key in the agent session afterward. Never invent an API key.
