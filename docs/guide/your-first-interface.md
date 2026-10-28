# Your first interface

[日本語](your-first-interface.ja.md)

How to add a function of your own (an interface) to an OEP probe, starting from a small one with no pins and ending with one
that drives a pin. Read it with `examples/02.Interfaces/HelloInterface` and `CustomInterface` open. The full rules are in
[writing a probe](writing-a-probe.md).

## 1. What an interface is

A probe is a small machine that answers requests from a host (the PC). A request says three things: which interface, which
operation (op), and what payload.

```text
host --request: fn 1, op 0x01, payload--> Endpoint --> your interface (handle)
host <--result: completed, payload------- Endpoint <--
```

- The **Endpoint** is the library's part: reading and writing frames, the session and the lock, and passing each request
  to its interface.
- An **interface** is a class you write: it has a name, takes ops and returns results.
- The endpoint numbers the interfaces in the order added (fn 1, 2, ...). A host finds them by name, not by number, so you
  need not care about the numbers.

## 2. Pick a name

Use something you own, reversed (reverse DNS): for a GitHub account, `io.github.<you>.<name>`. Nobody has to approve it.
`oep.*` is for the standard interfaces the specification defines; do not use it.

```cpp
const char *name() const override { return "io.github.example.hello"; }
uint16_t instance() const override { return 0; }   // which one of that name: 0 when there is only one
uint8_t revision() const override { return 1; }     // the version of the payloads' shapes
```

Raise the revision when the fixed part of a payload changes shape; adding a new op does not need it.

## 3. Pick the ops

HelloInterface only keeps a total.

| op | name | request payload | result payload | lock |
|---|---|---|---|---|
| 0x01 | add | value (u32) | total (u32) | needed (changes the total) |
| 0x02 | total | none | total (u32) | not needed (changes nothing) |

```cpp
bool offers(uint8_t op) const override { return op == kOpAdd || op == kOpTotal; }   // the ops it has
bool lockFree(uint8_t op) const override { return op == kOpTotal; }                 // the ops that change nothing
```

- `offers` says which ops it has. The endpoint answers any other op `unknown_operation`, so you need not refuse them yourself.
- An op that changes something only arrives from the session holding the lock; the endpoint checks that. Make a read that
  changes nothing `lockFree`, so it can be read without the lock.
- Numbers are little-endian (`oep::getU32` / `oep::putU32`).

## 4. Answer a request

```cpp
oep::Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override {
  oep::Tail tail;
  const oep::Result parsed = oep::plainTail(tail, payload, length, op == kOpAdd ? 4 : 0, out, capacity);
  if (oep::refused(parsed)) return parsed;   // too short, or a TLV it must not ignore
  if (capacity < 4) return oep::failed();
  if (op == kOpAdd) total_ += oep::getU32(payload);
  oep::putU32(out, total_);
  return oep::completed(4);                  // done: 4 bytes of result payload
}
```

- There are three kinds of result:
  - `completed(n)`: it worked, and n bytes were written to `out`.
  - `failed()`: it ran but did not work.
  - `rejected(reason)`: it did not run (`kRejectMalformed`, ...).
- Every request may carry TLVs (extra information) after its fixed part. Give `plainTail` the fixed part's length and it
  checks the length and handles the TLVs. To begin with, these two lines as they are are enough.
- Do not wait in `handle`. Do long work a little at a time from `loop()` (§7).

## 5. Add it and try it

```cpp
static Hello hello;
void setup() {
  // ...the endpoint set up as in MinimalProbe
  endpoint.add(hello);
}
void loop() { endpoint.poll(); }
```

Flash it, then call it from the Python client on the PC:

```python
from oep_client import core, link
hst = link.open_host("<port>"); hst.open(3000)          # open a session, holding the lock
fn = core.find(hst, "io.github.example.hello")          # find it by name
hst.call(fn, 0x01, (5).to_bytes(4, "little")).payload    # b'\x05\x00\x00\x00'
hst.call(fn, 0x02, locked=False).payload                 # read without the lock
```

`oep dump --port <port>` shows what the probe has: your interface and its ops.

## 6. Clean up when the session ends

When the host's session ends (end, a lapsed lease, another host's force: all alike), `sessionOver()` is called. Put back
what that session made; leave nothing for the next session.

```cpp
void sessionOver() override { total_ = 0; }
```

## 7. Using pins

An interface that drives pins does not pick them itself: the host assigns them at run time with the **plan**. That way one
firmware serves every jig, and two interfaces never drive the same pin. `CustomInterface` is the example (an LED blinker,
`io.github.example.blink`).

1. **Number the roles**: e.g. "1 = the output line".
2. **Declare the candidates in describe**: `w.roleChannels(roles, n, pins.allowedMask())`. The host reads it to assign them.
3. **Write the plan's three steps**:
   - `planRoles()` returns true.
   - `planCheck()` checks without changing anything: `kRejectUnsupported` for a role or pin it does not declare,
     `kRejectUnavailable` for a pin something else holds.
   - `planApply()` takes the pin with `pins.claim(channel, owner)`, and `planRelease()` gives it back with `pins.release(owner)`.
4. **The owner id**: use `oep::owners::kSketch` (0x80) and up. The library's parts use the ids in `oep::owners`, so they never
   clash.
5. A pin is driven only once the first op comes: taking the plan changes nothing on it.

For something that takes time (blinking), answer at once in `handle` and move it along in a `poll()` called from `loop()`.

## 8. Next

- Add your interface to the ready-made firmware (`Firmware/OepProbe`): [firmware features](firmware-features.md)
- Notifications, settings, debug wires and the other rules: [writing a probe](writing-a-probe.md)
- The specification itself: oep-spec `docs/oep-core.md` (§2 requests and results, §6 the lock, §7 describe, §8 the plan)
