"""fast16 torque command: mutation check (owner 2026-11-23: "just have some good debug and review checks for fast16").

Each mutant changes one rule of the torque path (firmware/joint_control.c, app.c, bus.h, main.c); the torque tests in
tests/test_pd.c (plain and APP_DEFAULT_COMP builds), tests/test_bus.c and, for bus/main mutants, the main-level cases of
tests/run_fw231_main_native.sh must fail for every one of them. Run from the tree root; needs clang.
"""
import os, shutil, subprocess, sys, tempfile

W = os.getcwd()
FL = ('-std=c11 -O1 -g -Wall -Wextra -Werror -Wno-misleading-indentation -fsanitize=address,undefined '
      '-fno-sanitize-recover=undefined -fno-omit-frame-pointer -DAPP_ASYNC_TX=0 -DSENSING_FAST_I2C=1').split()
PD = ('tests/test_pd.c tests/hw_stub.c firmware/app.c firmware/joint_control.c firmware/est32.c firmware/bam_native.c '
      'firmware/deadline.c firmware/general_reference.c firmware/torque.c firmware/observability.c '
      'research/calibration/xl330_calibration.c research/calibration/nv_read.c').split()
MUTANTS = [
 ('firmware/joint_control.c', 'if(s->tau_mode)u=s->tau_q24;', ';'),
 ('firmware/joint_control.c', 'if(s->tau_mode!=JC_TAU_RAW){', 'if(1){'),
 ('firmware/app.c', 's->control.tau_mode=leave?JC_TAU_PD:(uint8_t)m;', 's->control.tau_mode=(uint8_t)m;'),
 ('firmware/app.c', 'if(s->target_counts!=p->target||s->control.tau_mode)return 7;', 'if(s->target_counts!=p->target)return 7;'),
 ('firmware/app.c', 's->openloop_pwm=0;s->control.tau_mode=JC_TAU_PD;}', 's->openloop_pwm=0;}'),
 ('firmware/app.c', '&&!s->openloop&&!s->control.tau_mode&&!s->bam.running', '&&!s->openloop&&!s->bam.running'),
 ('firmware/app.c', '(!m&&(v< -480||v>480))', '((v< -480||v>480))'),
 ('firmware/app.c', '((uint32_t)s->control.tau_mode<<2)', '((uint32_t)s->control.tau_mode<<3)'),
 ('firmware/app.c', 's->openloop=!leave&&!m;', 's->openloop=!leave;'),
 ('firmware/app.c', "s->control.tau_mode=leave?JC_TAU_PD:(uint8_t)m;s->control.tau_q24=v;", "s->control.tau_mode=leave?JC_TAU_PD:(uint8_t)m;"),
 ('firmware/bus.h', 'return (address & ~12u) == 256u;', 'return (address & ~4u) == 256u;'),
 ('firmware/bus.h', 'return (address & ~12u) == 256u;', 'return (address & ~28u) == 256u;'),
 ('firmware/joint_control.c', 'if(s->tau_mode)u=s->tau_q24;', 'if(s->tau_mode)u=-(int64_t)s->tau_q24;'),            # sign
 ('firmware/joint_control.c', 'if(s->tau_mode)u=s->tau_q24;', 'if(s->tau_mode)u=(int64_t)s->tau_q24*2;'),           # scaling
 ('firmware/joint_control.c', 'if(s->tau_mode)u=s->tau_q24;', 'if(s->tau_mode)u=s->tau_q24>>1;'),                   # Q-format off by one bit
 ('firmware/joint_control.c', 'if(s->tau_mode!=JC_TAU_RAW){', 'if(s->tau_mode!=JC_TAU_COMP){'),                     # comp/raw swapped
 ('firmware/app.c', 'm>>=2;   /* m: 0 duty', 'm=(m>>2)?3u-(m>>2):0u;   /* m: 0 duty'),                                  # 264/268 mapping swapped
 ('firmware/app.c', ' if(p->kind==64u){if(!APP_ALLOW_MOTOR||', ' if(p->kind==64u){s->control.tau_mode=0;if(!APP_ALLOW_MOTOR||'),          # ARM clears torque
 ('firmware/app.c', 's->control.tau_mode=leave?JC_TAU_PD:(uint8_t)m;', 's->control.tau_mode=leave?(m?JC_TAU_PD:s->control.tau_mode):(uint8_t)m;'),   # lease HOLD (INT32_MIN to 260) keeps torque
 ('firmware/app.c', ' s->faults|=f;stop(s);', ' s->faults|=f;motor_off(s);'),                                         # fault keeps torque
 ('firmware/main.c', 'else if(bus_lease_motion(a)&&!cmd_broadcast)', 'else if((a==256u||a==260u)&&!cmd_broadcast)'),  # torque does not renew
 ('firmware/main.c', ' uint32_t irq=platform_irq_save();uint8_t result=app_write(ctx,a,n,in);', ' uint8_t result=app_write(ctx,a,n,in);uint32_t irq=platform_irq_save();'),   # review P1-T1: the mask taken after the write
]


def ok(tree, cmd):
    return subprocess.run(cmd, cwd=tree, capture_output=True).returncode == 0


def survives(tree, f):
    for extra in ([], ['-Wno-unused-function', '-DAPP_DEFAULT_COMP=1']):
        if not (ok(tree, ['clang', *FL, *extra, '-DAPP_ALLOW_MOTOR=1', '-Ifirmware', '-Iresearch/calibration', *PD, '-o', 'pd'])
                and ok(tree, ['./pd'])):
            return False
    if not (ok(tree, ['clang', *FL, '-Ifirmware', 'tests/test_bus.c', 'firmware/bus.c', 'firmware/protocol.c', '-o', 'bus'])
            and ok(tree, ['./bus'])):
        return False
    if f in ('firmware/main.c', 'firmware/bus.h', 'firmware/bus.c'):
        return ok(tree, ['sh', 'tests/run_fw231_main_native.sh'])
    return True


def main():
    killed = 0
    with tempfile.TemporaryDirectory() as tmp:
        for i, (f, a, b) in enumerate(MUTANTS, 1):
            tree = os.path.join(tmp, f'm{i}')
            for d in ('firmware', 'tests', 'research'):
                shutil.copytree(os.path.join(W, d), os.path.join(tree, d))
            path = os.path.join(tree, f)
            text = open(path).read()
            if text.count(a) != 1:
                print(f'torque mutant {i}: anchor not found once in {f}: {a!r}')
                return 1
            open(path, 'w').write(text.replace(a, b))
            if survives(tree, f):
                print(f'torque mutant {i} SURVIVED ({f}: {b!r})')
            else:
                killed += 1
    print(f'torque path mutations: {killed}/{len(MUTANTS)} killed')
    return 0 if killed == len(MUTANTS) else 1


if __name__ == '__main__':
    sys.exit(main())
