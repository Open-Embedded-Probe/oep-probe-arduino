# Your first interface

[日本語](your-first-interface.ja.md)

The steps to add your own feature (an interface) to an OEP probe. It starts with a small one that uses no pins and ends with one
that uses pins. Read it with `examples/02.Interfaces/HelloInterface` and `CustomInterface` open. The more detailed rules are in
[Writing a probe](writing-a-probe.md).

For commands that only need registered functions, [Adding your own commands](simple-interface.md) (`oep::SimpleInterface`) is
the easiest. Here the steps to write `oep::Interface` directly are explained from the inside.

## 1. What an interface is

A probe is a small machine that answers requests from the host (the PC). A request has 3 parts: "which interface, which
operation (op), with what contents (payload)".

```text
host --request: fn 1, op 0x10, payload--> Endpoint --> your interface (handle)
host <--result: completed, payload-------- Endpoint <--
```

- The **Endpoint** is the part the library has. It handles reading and writing frames, the session and lock, and which
  interface to hand a request to.
- An **interface** is a class you write. It has a name, takes ops and returns results.
- The endpoint numbers interfaces in the order added (fn 1, 2, ...). The host finds them by name, not by number, so you need not
  care about the numbers.

## 2. Choose a name

The name is something you own, written in reverse (reverse DNS). For a GitHub account it is `io.github.<you>.<name>`. Nobody's
approval is needed. `oep.*` is for the standard interfaces the specification defines; do not use it.

```cpp
const char *name() const override { return "io.github.example.hello"; }
uint16_t instance() const override { return 0; }   // which one of the same name. 0 if there is only one
uint8_t revision() const override { return 1; }     // the version of the payload form
```

When the form of the payload's fixed part changes, raise revision. Just adding a new op does not raise it.

## 3. Choose the ops

HelloInterface is an example that only counts a total.

| op | Name | Request payload | Result payload | Lock |
|---|---|---|---|---|
| 0x10 | add | value (u32) | total (u32) | needed (changes the total) |
| 0x11 | total | none | total (u32) | not needed (changes nothing) |

Choose your ops from 0x10-0xFF. 0x01-0x0F are the numbers of operations common to every interface (0x01 subscribe, 0x02
unsubscribe; core §2.5), and if you put your own op there the endpoint does not accept it (unknown operation).

```cpp
bool offers(uint8_t op) const override { return op == kOpAdd || op == kOpTotal; }   // the ops it has
bool lockFree(uint8_t op) const override { return op == kOpTotal; }                 // ops that change nothing
```

- `offers` states the ops it has. The endpoint answers other ops with `unknown_operation`, so you need not refuse them yourself.
- Ops that change something arrive only from the session that holds the lock. The endpoint checks this. Make reads that change
  nothing `lockFree`, so they can be read without the lock.
- Numbers are little-endian (`oep::getU32` / `oep::putU32`).

## 4. Answer requests

```cpp
oep::Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override {
  oep::Tail tail;
  const oep::Result parsed = oep::plainTail(tail, payload, length, op == kOpAdd ? 4 : 0, out, capacity);
  if (oep::refused(parsed)) return parsed;   // too short, or a TLV that cannot be ignored was attached
  if (capacity < 4) return oep::failed();
  if (op == kOpAdd) total_ += oep::getU32(payload);
  oep::putU32(out, total_);
  return oep::completed(4);                  // success. The result payload is 4 bytes
}
```

- There are 3 kinds of result.
  - `completed(n)`: success. n bytes were written to `out`.
  - `failed()`: executed but did not succeed.
  - `rejected(reason)`: not executed (`kRejectMalformed` and so on).
- Every request can carry TLVs (extra information) after the fixed part. Passing the fixed part's length to `plainTail` checks
  the length and handles the TLVs in one go. At first, writing these 2 lines as they are is enough.
- Do not wait inside `handle`. Advance long work little by little in `loop()` (§7).

## 5. Add it and try it

```cpp
static Hello hello;
void setup() {
  // ...prepare the endpoint (the same as MinimalProbe)
  endpoint.add(hello);
}
void loop() { endpoint.poll(); }
```

After flashing, call it on the PC from the Python client.

```python
from oep_client import core, link
hst = link.open_host("<port>"); hst.open(3000)          # open a session and take the lock
fn = core.find(hst, "io.github.example.hello")          # find by name
hst.call(fn, 0x10, (5).to_bytes(4, "little")).payload    # b'\x05\x00\x00\x00'
hst.call(fn, 0x11, locked=False).payload                 # read without the lock
```

`oep dump --port <port>` shows what the probe has (your interface and its ops).

## 6. Clean up when the session ends

When the host's session ends (end, lease expiry, forcing by another host, all the same), `sessionOver()` is called. Undo what
that session made here. Leave nothing for the next session.

```cpp
void sessionOver() override { total_ = 0; }
```

## 7. Use pins

An interface that drives pins does not decide which pins to use by itself. The host assigns them at run time with the **plan**.
This way the same firmware works on any fixture, and two interfaces never drive the same pin. `CustomInterface` is the example
(`io.github.example.blink`, which blinks an LED).

1. **Decide the roles**: number them, like "1 = the output line".
2. **Declare the candidates in describe**: `w.roleChannels(roles, n, pins.allowedMask())`. The host reads this to assign.
3. **Write the four plan functions**:
   - `planRoles()` returns true.
   - `planCheck()` checks without changing anything. It returns `kRejectUnsupported` for undeclared roles or pins, and
     `kRejectUnavailable` for pins someone else uses.
   - `planApply()` takes the pins with `pins.claim(channel, owner)`, and `planRelease()` gives them back with
     `pins.release(owner)`.
4. **Owner numbers** use `oep::owners::kSketch` (0x80) and up. The library's parts use the numbers in `oep::owners`, so they do
   not overlap.
5. A pin is driven only when the first op comes. Receiving the plan alone changes nothing.

For long-running actions (such as blinking), answer at once in `handle` and advance little by little in `poll()`, called from
`loop()`.

## 8. Next

- Adding your own interface to the ready-made firmware (`Firmware/OepProbe`): [Firmware features](firmware-features.md)
- Detailed rules for sending notifications, holding settings, debug wires and so on: [Writing a probe](writing-a-probe.md)
- The specification itself: oep-spec's `docs/oep-core.md` (§4 requests and results, §6 lock, §7 describe, §8 channels) and
  `interfaces/oep-if-plan.md` (plan)
