import ctypes as C
from fractions import Fraction
from pathlib import Path
import hashlib,sys
class Cal(C.Structure):_fields_=[('knots',C.c_uint16*65),('origin',C.c_uint16),('trim',C.c_int16),('aux',C.c_int16),('segments',C.c_uint8),('ready',C.c_bool)]
class Tracker(C.Structure):_fields_=[('uncorrected',C.c_int64),('homing',C.c_int32),('phase',C.c_uint16),('maxstep',C.c_uint16),('reverse',C.c_bool),('ready',C.c_bool)]
new=C.CDLL(sys.argv[1]);old=C.CDLL(sys.argv[2]);fixture=Path(sys.argv[3]).read_bytes();buf=(C.c_uint8*len(fixture)).from_buffer_copy(fixture)
for lib in (new,old):
 lib.xl330_calibration_load.argtypes=[C.POINTER(Cal),C.POINTER(C.c_uint8),C.c_size_t];lib.xl330_calibration_load.restype=C.c_bool
 lib.xl330_calibration_map.argtypes=[C.POINTER(Cal),C.c_int64,C.c_bool,C.POINTER(C.c_int64)];lib.xl330_calibration_map.restype=C.c_bool
 lib.xl330_tracker_begin.argtypes=[C.POINTER(Tracker),C.POINTER(Cal),C.c_uint16,C.c_bool,C.c_int32,C.c_uint16,C.POINTER(C.c_int64)];lib.xl330_tracker_begin.restype=C.c_bool
 lib.xl330_tracker_update.argtypes=[C.POINTER(Tracker),C.POINTER(Cal),C.c_uint16,C.POINTER(C.c_int64)];lib.xl330_tracker_update.restype=C.c_bool
new.xl330_calibration_map_fraction.argtypes=[C.POINTER(Cal),C.c_int64,C.c_bool,C.POINTER(C.c_int64),C.POINTER(C.c_int32)];new.xl330_calibration_map_fraction.restype=C.c_bool
new.xl330_tracker_begin_fraction.argtypes=[C.POINTER(Tracker),C.POINTER(Cal),C.c_uint16,C.c_bool,C.c_int32,C.c_uint16,C.POINTER(C.c_int64),C.POINTER(C.c_int32)];new.xl330_tracker_begin_fraction.restype=C.c_bool
new.xl330_tracker_update_fraction.argtypes=[C.POINTER(Tracker),C.POINTER(Cal),C.c_uint16,C.POINTER(C.c_int64),C.POINTER(C.c_int32)];new.xl330_tracker_update_fraction.restype=C.c_bool
actual=Cal();assert new.xl330_calibration_load(C.byref(actual),buf,len(fixture));assert actual.origin==1234 and actual.segments==64 and actual.trim==42 and actual.aux==-321  # synthetic fixture: tests/fixtures/make_synthetic_calibration.py
cases=[actual]
for table in ([0,1,2,3,4096],[0,1000,2050,3070,4096]):
 c=Cal();c.segments=4;c.origin=37;c.ready=True
 for i,v in enumerate(table):c.knots[i]=v
 cases.append(c)
def rational(c,n,reverse):
 turn,phase=divmod(n,4096);x=phase if reverse else 4095-phase;lo=0;hi=c.segments
 while hi-lo>1:
  m=(hi+lo)//2
  if x>c.knots[m]:lo=m
  else:hi=m
 value=Fraction(lo*(4096//c.segments))+Fraction((x-c.knots[lo])*(4096//c.segments),c.knots[lo+1]-c.knots[lo]);return turn*4096+(value if reverse else 4095-value)
checks=0
for cal in cases:
 for reverse in (False,True):
  for phase in range(4096):
   for turn in (-3,-1,0,1,3):
    n=phase+turn*4096;a=C.c_int64(111);b=C.c_int64(222);f=C.c_int32(333);assert old.xl330_calibration_map(C.byref(cal),n,reverse,C.byref(a));assert new.xl330_calibration_map_fraction(C.byref(cal),n,reverse,C.byref(b),C.byref(f));assert a.value==b.value
    value=Fraction(b.value*65536+f.value,65536);error=value-rational(cal,n,reverse);assert abs(error)<Fraction(1,65536);assert (-65535<=f.value<=0) if not reverse else (0<=f.value<=65535);checks+=1
  for home in (-2147483648,0,2147483647):
   for raw in range(4096):
    ta,tb=Tracker(),Tracker();a,b=C.c_int64(),C.c_int64();f=C.c_int32();assert old.xl330_tracker_begin(C.byref(ta),C.byref(cal),raw,reverse,home,128,C.byref(a));assert new.xl330_tracker_begin_fraction(C.byref(tb),C.byref(cal),raw,reverse,home,128,C.byref(b),C.byref(f));assert a.value==b.value;assert all(getattr(ta,n)==getattr(tb,n) for n,_ in Tracker._fields_);assert abs(Fraction((b.value-home)*65536+f.value,65536)-rational(cal,tb.uncorrected,reverse))<Fraction(1,65536);checks+=1
   ta,tb=Tracker(),Tracker();a,b=C.c_int64(),C.c_int64();f=C.c_int32();raw=4080;assert old.xl330_tracker_begin(C.byref(ta),C.byref(cal),raw,reverse,home,128,C.byref(a));assert new.xl330_tracker_begin_fraction(C.byref(tb),C.byref(cal),raw,reverse,home,128,C.byref(b),C.byref(f))
   for step in [31]*300+[-47]*300+[128,-128]*10:
    raw=(raw+step)&4095;assert old.xl330_tracker_update(C.byref(ta),C.byref(cal),raw,C.byref(a));assert new.xl330_tracker_update_fraction(C.byref(tb),C.byref(cal),raw,C.byref(b),C.byref(f));assert a.value==b.value;assert all(getattr(ta,n)==getattr(tb,n) for n,_ in Tracker._fields_);assert abs(Fraction((b.value-home)*65536+f.value,65536)-rational(cal,tb.uncorrected,reverse))<Fraction(1,65536);checks+=1
   a.value=b.value=987654;f.value=12345;assert not old.xl330_tracker_update(C.byref(ta),C.byref(cal),5000,C.byref(a));assert not new.xl330_tracker_update_fraction(C.byref(tb),C.byref(cal),5000,C.byref(b),C.byref(f));assert a.value==b.value==987654 and f.value==12345 and not ta.ready and not tb.ready
# Invalid maps preserve output; complete original wrappers remain callable.
invalid=Cal();a=C.c_int64(9);f=C.c_int32(10);assert not new.xl330_calibration_map_fraction(C.byref(invalid),0,False,C.byref(a),C.byref(f));assert a.value==9 and f.value==10
print(f'fractional calibration: {checks} rational/stock parity checks; all4096 phases,2 drives,4/64 segments,negative/multipleturns,homing extremes,wrap/tracker/invalid invariants PASS; fixtureSHA256={hashlib.sha256(fixture).hexdigest()}')
