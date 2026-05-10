#!/usr/bin/env python3
"""Noninteractive RNS proof probe for RTNode TCP <-> RNode LoRa tests."""

import argparse
import os
import sys
import time
from pathlib import Path

import RNS


APP_NAME = "rtnode_probe"
ASPECT = "proof"


def write_config(config_dir, side, args):
    config_dir.mkdir(parents=True, exist_ok=True)
    lines = [
        "[reticulum]",
        "  enable_transport = no",
        "  share_instance = no",
        f"  instance_name = proof_probe_{side}",
        "",
        "[interfaces]",
        "",
    ]

    if side == "tcp":
        lines += [
            "  [[RTNode TCP]]",
            "    type = TCPClientInterface",
            "    enabled = yes",
            f"    target_host = {args.tcp_host}",
            f"    target_port = {args.tcp_port}",
            "",
        ]
    elif side == "lora":
        lines += [
            "  [[RNode LoRa]]",
            "    type = RNodeInterface",
            "    enabled = yes",
            f"    port = {args.rnode_port}",
            f"    frequency = {args.frequency}",
            f"    bandwidth = {args.bandwidth}",
            f"    txpower = {args.txpower}",
            f"    spreadingfactor = {args.spreadingfactor}",
            f"    codingrate = {args.codingrate}",
            "",
        ]
    else:
        raise ValueError(f"Unsupported side: {side}")

    (config_dir / "config").write_text("\n".join(lines), encoding="utf-8")


def load_or_create_identity(identity_path):
    if identity_path.exists():
        identity = RNS.Identity.from_file(str(identity_path))
        RNS.log(f"Loaded identity from {identity_path}", RNS.LOG_NOTICE)
        return identity

    identity = RNS.Identity()
    identity.to_file(str(identity_path))
    RNS.log(f"Created identity at {identity_path}", RNS.LOG_NOTICE)
    return identity


def run_server(args):
    config_dir = args.work_dir / f"config_{args.side}_server"
    write_config(config_dir, args.side, args)
    RNS.loglevel = RNS.LOG_DEBUG if args.debug else RNS.LOG_NOTICE
    RNS.Reticulum(configdir=str(config_dir))

    identity = load_or_create_identity(args.work_dir / f"{args.side}_server.identity")
    destination = RNS.Destination(identity, RNS.Destination.IN, RNS.Destination.SINGLE, APP_NAME, ASPECT)
    destination.set_proof_strategy(RNS.Destination.PROVE_ALL)

    def on_packet(data, packet):
        RNS.log(
            "SERVER_RX "
            f"side={args.side} len={len(data)} hash={RNS.hexrep(packet.packet_hash, delimit=False)} "
            f"from={packet.receiving_interface}",
            RNS.LOG_NOTICE,
        )

    destination.set_packet_callback(on_packet)
    args.hash_file.parent.mkdir(parents=True, exist_ok=True)
    args.hash_file.write_text(destination.hash.hex(), encoding="utf-8")

    RNS.log(f"SERVER_READY side={args.side} hash={destination.hash.hex()}", RNS.LOG_NOTICE)
    destination.announce()
    RNS.log("SERVER_ANNOUNCE initial", RNS.LOG_NOTICE)

    start = time.time()
    last_announce = start
    while time.time() - start < args.duration:
        now = time.time()
        if args.announce_interval > 0 and now - last_announce >= args.announce_interval:
            destination.announce()
            last_announce = now
            RNS.log("SERVER_ANNOUNCE periodic", RNS.LOG_NOTICE)
        time.sleep(0.25)

    RNS.log("SERVER_DONE", RNS.LOG_NOTICE)


def run_client(args):
    config_dir = args.work_dir / f"config_{args.side}_client"
    write_config(config_dir, args.side, args)
    RNS.loglevel = RNS.LOG_DEBUG if args.debug else RNS.LOG_NOTICE
    RNS.Reticulum(configdir=str(config_dir))

    if args.target:
        target_hash = bytes.fromhex(args.target)
    else:
        target_hash = bytes.fromhex(args.hash_file.read_text(encoding="utf-8").strip())

    RNS.log(f"CLIENT_TARGET side={args.side} hash={target_hash.hex()}", RNS.LOG_NOTICE)
    start = time.time()
    RNS.Transport.request_path(target_hash)
    while not RNS.Transport.has_path(target_hash):
        if time.time() - start > args.path_timeout:
            RNS.log(f"CLIENT_FAIL no path after {args.path_timeout}s", RNS.LOG_ERROR)
            return 2
        time.sleep(0.25)

    RNS.log(f"CLIENT_PATH_READY elapsed={time.time() - start:.1f}s", RNS.LOG_NOTICE)
    identity = RNS.Identity.recall(target_hash)
    if identity is None:
        RNS.log("CLIENT_FAIL identity recall returned none", RNS.LOG_ERROR)
        return 3

    destination = RNS.Destination(identity, RNS.Destination.OUT, RNS.Destination.SINGLE, APP_NAME, ASPECT)
    payload = args.payload.encode("utf-8")
    packet = RNS.Packet(destination, payload)
    receipt = packet.send()
    receipt.set_timeout(args.timeout)

    def delivered(receipt):
        RNS.log(f"CLIENT_DELIVERED rtt={receipt.get_rtt():.3f}s", RNS.LOG_NOTICE)

    def timed_out(receipt):
        RNS.log("CLIENT_TIMEOUT proof not received", RNS.LOG_WARNING)

    receipt.set_delivery_callback(delivered)
    receipt.set_timeout_callback(timed_out)
    RNS.log(f"CLIENT_SENT packet_hash={RNS.hexrep(receipt.hash, delimit=False)} len={len(payload)}", RNS.LOG_NOTICE)

    wait_start = time.time()
    while time.time() - wait_start < args.timeout + 2:
        if receipt.status == RNS.PacketReceipt.DELIVERED:
            return 0
        if receipt.status == RNS.PacketReceipt.FAILED:
            return 4
        time.sleep(0.25)

    RNS.log(f"CLIENT_FAIL final_status={receipt.status}", RNS.LOG_ERROR)
    return 5


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["server", "client"])
    parser.add_argument("--side", choices=["tcp", "lora"], required=True)
    parser.add_argument("--work-dir", type=Path, default=Path(__file__).with_name("proof_probe_state"))
    parser.add_argument("--hash-file", type=Path, default=Path(__file__).with_name("proof_probe_state") / "server_hash.txt")
    parser.add_argument("--duration", type=float, default=90)
    parser.add_argument("--announce-interval", type=float, default=20)
    parser.add_argument("--target", default=None)
    parser.add_argument("--path-timeout", type=float, default=45)
    parser.add_argument("--timeout", type=float, default=45)
    parser.add_argument("--payload", default="proof probe")
    parser.add_argument("--tcp-host", default="mynode.local")
    parser.add_argument("--tcp-port", type=int, default=4242)
    parser.add_argument("--rnode-port", default="/dev/cu.usbmodem11201")
    parser.add_argument("--frequency", type=int, default=914875000)
    parser.add_argument("--bandwidth", type=int, default=125000)
    parser.add_argument("--spreadingfactor", type=int, default=10)
    parser.add_argument("--codingrate", type=int, default=5)
    parser.add_argument("--txpower", type=int, default=22)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    args.work_dir.mkdir(parents=True, exist_ok=True)

    if args.mode == "server":
        run_server(args)
        return 0
    return run_client(args)


if __name__ == "__main__":
    sys.exit(main())