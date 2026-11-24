"""bam_eval (C, Q16) vs the analytic BAM oracle (references.py) over every tick."""
import ctypes, math, subprocess, sys, tempfile, os
from pathlib import Path
R = Path(__file__).resolve().parent.parent
# The analytic oracle (references.py) is not part of this tree. Point
# XL330_BAM_ORACLE_DIR at a directory containing it; without it the check is skipped.
_oracle = os.environ.get('XL330_BAM_ORACLE_DIR')
if not _oracle or not (Path(_oracle) / 'references.py').is_file():
    print('BAM native vs oracle SKIPPED (set XL330_BAM_ORACLE_DIR to a directory with references.py)')
    sys.exit(0)
ORACLE = Path(_oracle)
sys.path.insert(0, str(ORACLE)); import references as REF
lib = Path(tempfile.mkdtemp()) / 'bam.so'
subprocess.check_call(['clang', '-std=c11', '-O2', '-shared', '-fPIC', '-I', str(R / 'firmware'), str(R / 'firmware/bam_native.c'), '-o', str(lib)])
L = ctypes.CDLL(str(lib))
class Out(ctypes.Structure):
    _fields_ = [('q', ctypes.c_int32), ('v', ctypes.c_int32), ('a', ctypes.c_int32), ('active', ctypes.c_uint8), ('enable', ctypes.c_uint8), ('terminal', ctypes.c_uint8), ('junction', ctypes.c_uint8)]
L.bam_eval.argtypes = [ctypes.c_uint8, ctypes.c_uint32, ctypes.c_int32, ctypes.POINTER(Out)]; L.bam_eval.restype = ctypes.c_bool
worst = {}
for kind in range(4):
    eq = ev = 0.0
    for tick in range(12001):
        o = Out(); assert L.bam_eval(kind, tick, 0, ctypes.byref(o))
        q, v, a, _ = REF.reference(kind, tick * 0.0005)
        eq = max(eq, abs(o.q / 65536 - q)); ev = max(ev, abs(o.v / 65536 - v))
    worst[REF.NAMES[kind]] = (eq, ev)
    print(f'{REF.NAMES[kind]:16s} max|q err| {math.degrees(eq):.4f} deg  max|v err| {ev:.4f} rad/s')
    assert eq < math.radians(0.05) and ev < 0.05, kind
print('BAM native vs oracle PASS')
