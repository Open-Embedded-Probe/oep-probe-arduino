# /// script
# dependencies = ["pyserial>=3.5", "pyusb>=1.3", "libusb1>=3"]
# ///
import sys
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[4] / "oep-client-python" / "src"))
from oep_client.v1 import core, link
A = link.open_usb_host(transports=("vendor",))
try:
    A.open(3000)
    print("build:", A.call(core.find(A, "oep.test.bridge"), 0x04).payload.decode())
    A.end()
finally:
    A.link.close()
