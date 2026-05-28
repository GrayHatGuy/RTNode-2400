# Core Principles

## FIREWALL_MODE Boundary Transport

The key to allowing transport to work reliably on the ESP32 is that all WAN traffic, meaning backbone-side traffic, must be filtered immediately at the point of reception. This is the purpose of `FIREWALL_MODE`.

The ESP32 cannot safely accept arbitrary backbone traffic and then decide later whether it should be retained, cached, routed, rebroadcast, or processed. Unfiltered WAN traffic can exhaust internal heap, flood transport state, trigger LoRa rebroadcast storms, and cause watchdog resets. The boundary firewall must therefore make a fast accept-or-drop decision before normal transport processing.

## Required Whitelists

`FIREWALL_MODE` must maintain two boundary whitelists:

1. LAN device addresses

   Every address belonging to a device observed on the LAN side must be remembered. These are local-side identities, destinations, link identifiers, and other packet addresses that identify LAN-originating devices or flows.

2. LAN-mentioned addresses

   Every address referred to by a LAN-side device must be remembered. If a LAN device sends, requests, proves, links to, or otherwise references an address, that address is considered relevant to LAN traffic and may be allowed back through the boundary.

Together, these whitelists define the only WAN-side traffic that is allowed to enter normal transport handling.

## WAN-Side Rule

Every packet received from the WAN or backbone side must be inspected immediately.

If the packet does not contain an address from either boundary whitelist, it must be deleted at reception and must not continue into transport processing.

This applies before the packet can populate routing tables, path tables, announce tables, packet hash lists, caches, link state, receipt state, or outbound queues.

Backbone announces require special care. They must not be accepted merely because they are announces. In boundary mode, unsolicited WAN announces are hostile to stability unless they are a directly solicited response to LAN-originating discovery and are addressed to this transport instance.

## LAN-Side Rule

All packets that originate from the LAN side are trusted to pass freely through all configured interfaces.

LAN-originating traffic is what teaches the boundary firewall what is relevant. As LAN packets pass through the system, the firewall must update both boundary whitelists so that valid return traffic from the WAN side can be admitted later.

LAN-side packets should not be blocked just because they reference unknown addresses. Instead, those references are the signal used to permit corresponding WAN-side return traffic.

LAN-side trust applies to endpoint traffic. A LAN-connected Reticulum node that is itself running transport mode can relay WAN-scale announces and routed packets into the boundary node through the LAN interface. That traffic must be treated as a topology hazard, even if it arrives on a local TCP connection, because it can recreate the same unbounded routing workload that `MODE_BOUNDARY` is designed to prevent on the WAN side.

Local TCP clients should therefore be configured as clients/endpoints, not transport nodes, unless the firmware has an explicit policy for bounded LAN-side transport peers.

## Implementation Contract

In `FIREWALL_MODE`, the transport layer must preserve these invariants:

- WAN packets are filtered before normal transport state is mutated.
- WAN packets without whitelisted addresses are dropped immediately.
- LAN packets are allowed to traverse all interfaces.
- LAN packets update the LAN-device and LAN-mentioned whitelists.
- LAN-connected transport routers are not ordinary endpoint traffic and must not be allowed to reintroduce unbounded WAN routing state through the LAN side.
- WAN announces are heavily filtered and are never allowed to become an unbounded announce, cache, or rebroadcast workload.
- Boundary filtering must protect internal heap and watchdog health on ESP32-class devices.

Any change that weakens these rules risks turning the boundary node into an unbounded backbone router, which is not the purpose of this firmware mode.