"""PD-comp v2 defaults (app.c): demo5/6 baked gear-train values (main.c demo_service) except
ja = 6375 (reference-accel feed-forward, 0.38e-3 kg m^2) and high-pass damping OFF (kd 0, shift 0).
v1.1 Stage 3: the XL330-M077 BETA tables (APP_GEAR == 77) are the M288 ones scaled by r = 77.5/288.4 with the rules of
docs/M077_BETA.md (c0 x r, c1 kept, b and b_drop x r^2, J_fb x r^2, veps and the inertia gate x 1/r,
ja = horn 2e-6 + 40 % of the M077 rotor, flags and magics kept), each within rounding."""
import re
from pathlib import Path
F = Path(__file__).resolve().parent.parent / 'firmware'
app = (F / 'app.c').read_text()
def arr(src, pat):
    m = re.search(pat, src); assert m, pat
    return [x.strip() for x in m.group(1).split(',')]
# the two gears' tables: '#if APP_GEAR == 77 ... #else ... #endif' around them
m = re.search(r'#if APP_GEAR == 77\n(.*?)#else\n(.*?)#endif', app, re.S)
assert m and 'app_comp_default' in m.group(1) and 'app_comp_default' in m.group(2), 'gear tables not found'
m077, m288 = m.group(1), m.group(2)
# demo5/6 baked values (main.c demo_service until fast17 retired the demo images)
DEMO_COMP = ['68451u', '20054u', '1401u', '0u', '3277u', '0u', '0u', '9040u', '0x102u', 'JC_COMP_MAGIC']
DEMO_COMP2 = ['405u', '19661u', '1u', 'JC_COMP2_MAGIC']
c, d = arr(m288, r'app_comp_default\[10\]=\{([^}]*)\}'), DEMO_COMP
assert c[3] == '6375u' and d[3] == '0u' and c[:3] + c[4:] == d[:3] + d[4:], (c, d)
assert arr(m288, r'app_comp2_default\[4\]=\{([^}]*)\}') == DEMO_COMP2
assert arr(app, r'app_hpd_default\[3\]=\{([^}]*)\}') == ['0u', '0u', 'JC_HPD_MAGIC']
print('PD-comp v2 defaults == demo5/6 gear-train values except ja 6375 and HP damping off PASS')

num = lambda v: int(v.rstrip('u'), 0)
r = 77.5 / 288.4
a, b = [num(v) for v in arr(m288, r'app_comp_default\[10\]=\{([^}]*)\}')[:9]], [num(v) for v in arr(m077, r'app_comp_default\[10\]=\{([^}]*)\}')[:9]]
want = [a[0] * r, a[1], a[2] * r * r, (2e-6 + 0.4 * 0.898e-3 * r * r) * 2 ** 24, a[4] / r, a[5], a[6], a[7] * r * r, a[8]]
assert all(abs(x - y) <= 1.0 for x, y in zip(b, want)), (b, [round(w, 2) for w in want])   # 1 LSB: veps is 0.05/r exact, not 3277/r
a2, b2 = [num(v) for v in arr(m288, r'app_comp2_default\[4\]=\{([^}]*)\}')[:3]], [num(v) for v in arr(m077, r'app_comp2_default\[4\]=\{([^}]*)\}')[:3]]
want2 = [a2[0] * r * r, a2[1] / r, a2[2]]
assert all(abs(x - y) <= 1.0 for x, y in zip(b2, want2)), (b2, want2)
assert arr(m077, r'app_comp_default\[10\]=\{([^}]*)\}')[9] == 'JC_COMP_MAGIC' and arr(m077, r'app_comp2_default\[4\]=\{([^}]*)\}')[3] == 'JC_COMP2_MAGIC'
print('XL330-M077 beta tables == the M288 tables scaled by r = 77.5/288.4 (M077 beta scaling rules) PASS')
