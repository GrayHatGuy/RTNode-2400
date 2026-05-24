# Proof Probe Harnesses

`tests/proof_probe_harness.py` wraps `tests/proof_probe.py` into named scenarios so the common transport paths can be re-run without rebuilding long one-off commands.

The harnesses currently cover:

- `local-tcp-to-local-tcp`
- `lora-to-local-tcp`
- `local-tcp-to-wan`
- `wan-to-local-tcp`

The last two use the same mixed `tcp` and `lora` endpoint shape as the earlier proof probes, but keep separate work directories so LAN-to-WAN reachability checks and WAN-ingress checks do not reuse identities, cached paths, or hash files from another scenario.

The harness also supports orchestrated runs so the server side can be started, waited on, exercised by the client, and then stopped in one command.

## List the scenarios

```bash
/Users/james/Offline/Reticulum/RTNode-HeltecV4/.venv/bin/python \
  /Users/james/Offline/Reticulum/RTNode-HeltecV4/tests/proof_probe_harness.py list
```

## Show the commands for one scenario

```bash
/Users/james/Offline/Reticulum/RTNode-HeltecV4/.venv/bin/python \
  /Users/james/Offline/Reticulum/RTNode-HeltecV4/tests/proof_probe_harness.py \
  show lora-to-local-tcp
```

## Start the server side

```bash
/Users/james/Offline/Reticulum/RTNode-HeltecV4/.venv/bin/python \
  /Users/james/Offline/Reticulum/RTNode-HeltecV4/tests/proof_probe_harness.py \
  server local-tcp-to-wan \
  --rnode-port /dev/cu.usbmodem114401 \
  --frequency 914875000 \
  --bandwidth 125000 \
  --spreadingfactor 10 \
  --codingrate 5 \
  --txpower 14 \
  --debug
```

## Start the client side

```bash
/Users/james/Offline/Reticulum/RTNode-HeltecV4/.venv/bin/python \
  /Users/james/Offline/Reticulum/RTNode-HeltecV4/tests/proof_probe_harness.py \
  client wan-to-local-tcp \
  --rnode-port /dev/cu.usbmodem114401 \
  --frequency 914875000 \
  --bandwidth 125000 \
  --spreadingfactor 10 \
  --codingrate 5 \
  --txpower 14 \
  --payload 'wan to local tcp harness probe' \
  --debug
```

## Run one full scenario end to end

```bash
/Users/james/Offline/Reticulum/RTNode-HeltecV4/.venv/bin/python \
  /Users/james/Offline/Reticulum/RTNode-HeltecV4/tests/proof_probe_harness.py \
  run lora-to-local-tcp \
  --tcp-host 192.168.2.122 \
  --rnode-port /dev/cu.usbmodem114401 \
  --frequency 914875000 \
  --bandwidth 125000 \
  --spreadingfactor 10 \
  --codingrate 5 \
  --txpower 14 \
  --debug
```

## Run all four scenarios

```bash
/Users/james/Offline/Reticulum/RTNode-HeltecV4/.venv/bin/python \
  /Users/james/Offline/Reticulum/RTNode-HeltecV4/tests/proof_probe_harness.py \
  run-all \
  --tcp-host 192.168.2.122 \
  --rnode-port /dev/cu.usbmodem114401 \
  --frequency 914875000 \
  --bandwidth 125000 \
  --spreadingfactor 10 \
  --codingrate 5 \
  --txpower 14 \
  --debug
```

## Dry-run mode

Pass `--dry-run` to the `server`, `client`, `run`, or `run-all` command to print the exact `proof_probe.py` invocations without launching them.

## Defaults

- The harness uses the same Python interpreter that launched it unless `--python` is provided.
- `PYTHONPATH` is prepended with the sibling `Reticulum-master` tree unless `--reticulum-root` is overridden.
- Default work directories are created under `RTNode-HeltecV4/tests/harness_*`.
- TCP defaults to `mynode.local:4242`.
- LoRa defaults to `/dev/cu.usbmodem114401` on `914875000/125000/SF10/CR5/TXP14`.
- Orchestrated runs write scenario-specific server and client logs into the scenario work directory.
- Orchestrated runs clear old `config_*` directories and `server_hash.txt` by default so the path lookup is not satisfied by stale state.