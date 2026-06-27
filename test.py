import math
import numpy as np
import pygame
import sounddevice as sd
from scipy import signal

pygame.init()

WIDTH, HEIGHT = 700, 420
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Audi RS7 4.0 TFSI V8 Biturbo")
font = pygame.font.SysFont(None, 48)
clock = pygame.time.Clock()

# -------------------- Arac durumu --------------------
rpm = 900.0
speed = 0.0
gear = 1
throttle = 0.0
pop_burst = 0.0      # ani egzoz patlamasi tetigi (lift/vites/limiter)
bov_burst = 0.0      # blow-off valf tetigi (boost altinda gaz kesince)
auto = True          # otomatik vites
exhaust_mode = True  # True=EGZOZ (disardan), False=MOTOR (kabin)
brake = 0.0
engine_on = False    # motor calisiyor mu
cranking = False     # mars donuyor mu
limiter = False      # devir limitine vuruyor mu (sert kesme)

gear_ratios = {1: 4.71, 2: 3.14, 3: 2.10, 4: 1.67,
               5: 1.29, 6: 1.00, 7: 0.84, 8: 0.67}

IDLE_RPM = 820.0
MAX_RPM = 7000.0
REDLINE = 6800.0

# -------------------- Ses --------------------
SR = 44100
BLOCK = 512

# Cross-plane V8: 2 turda (1 cevrim) 8 atesleme.
# Banka genlik farki + hafif duzensizlik = RS7'nin "lope/burble" karakteri.
_base = np.arange(8) / 8.0
_jit = np.array([0, .010, 0, .010, 0, .010, 0, .010])
FIRE_ANGLES = (_base + _jit) % 1.0
FIRE_AMPS = np.array([1.0, .72, 1.0, .72, 1.0, .72, 1.0, .72])


def rbj_bandpass(f, Q):
    """RBJ bandpass biquad (peak gain = Q) -> rezonator."""
    w0 = 2 * math.pi * f / SR
    a = math.sin(w0) / (2 * Q)
    cw = math.cos(w0)
    b = np.array([a, 0.0, -a])
    aa = np.array([1 + a, -2 * cw, 1 - a])
    return b / aa[0], aa / aa[0]


# Egzoz "govdesi": sabit rezonanslar. Atesleme hizi devirle artar ama
# rezonans frekanslari (boru geometrisi) sabit -> gercek egzoz boyle davranir.
RESONATORS = [
    dict(f=70,   Q=5,  base=1.00, load=0.00),   # alt homurtu / thump
    dict(f=155,  Q=7,  base=0.90, load=0.10),
    dict(f=330,  Q=8,  base=0.65, load=0.35),   # govde
    dict(f=620,  Q=10, base=0.35, load=0.55),
    dict(f=1250, Q=12, base=0.10, load=0.75),   # rasp (gazla acilir)
]
for r in RESONATORS:
    r["b"], r["a"] = rbj_bandpass(r["f"], r["Q"])
    r["zi"] = np.zeros(2)

# Sub-bass (gogus thump)
SUB_B, SUB_A = signal.butter(2, 120 / (SR / 2), btype="low")
# Intake / induction gurultusu
NZ_B, NZ_A = signal.butter(2, [500 / (SR / 2), 2200 / (SR / 2)], btype="band")
# Genel cikis lowpass (sertligi alir)
OUT_B, OUT_A = signal.butter(2, 7500 / (SR / 2), btype="low")
# "Motor" modu icin ek bogma lowpass (egzoz modunda atlanir)
MUF_B, MUF_A = signal.butter(2, 1900 / (SR / 2), btype="low")
# Blow-off / dump valf "psshh" (genis bantli hava gurultusu)
BOV_B, BOV_A = signal.butter(2, [1200 / (SR / 2), 6500 / (SR / 2)], btype="band")

# -------------------- Egzoz patlama cekirdekleri --------------------
# Her "pat" = keskin atak + alt thump + parlak catirti, ustel sonum.
# Tek-sample impuls yerine bunlar gercek patlama gibi duyulur.
POP_LEN = int(0.060 * SR)


def make_pop_kernel(c):
    t = np.arange(POP_LEN) / SR
    env = np.exp(-t * (40 + 25 * c))
    thump = np.sin(2 * np.pi * (85 + 50 * c) * t) * 0.8        # alt patlama
    mid = np.sin(2 * np.pi * (480 + 380 * c) * t) * np.exp(-t * 80) * 0.5
    crack = np.random.randn(POP_LEN) * np.exp(-t * 130) * 1.3  # parlak catirti
    k = (thump + mid + crack) * env
    return (k / (np.max(np.abs(k)) + 1e-9)).astype(np.float64)


POP_KERNELS = [make_pop_kernel(i / 5.0) for i in range(6)]


class Engine:
    def __init__(self):
        self.cyc = 0.0          # monoton cevrim sayaci
        self.s_rpm = IDLE_RPM
        self.s_thr = 0.0
        self.crackle = 0.0
        self.turbo_ph = 0.0
        # zi uzunluklari filtre derecesinden (len(a)-1) gelir
        self.zi_sub = np.zeros(len(SUB_A) - 1)
        self.zi_nz = np.zeros(len(NZ_A) - 1)
        self.zi_out = np.zeros(len(OUT_A) - 1)
        self.zi_muf = np.zeros(len(MUF_A) - 1)
        self.zi_bov = np.zeros(len(BOV_A) - 1)
        self.pop_tail = np.zeros(POP_LEN)   # bloklar arasi patlama kuyrugu
        self.boost = 0.0                    # turbo basinci (0..1), gecikmeli spool
        self.bov = 0.0                      # blow-off zarf
        self.lim_ph = 0.0                   # rev-limiter kesme fazi
        self.idx = np.arange(BLOCK)

    def callback(self, out, frames, t, status):
        # --- motor kapali: sessizlik ---
        if not engine_on and not cranking:
            out[:] = 0.0
            return

        # --- parametre yumusatma ---
        self.s_rpm += (rpm - self.s_rpm) * 0.30
        self.s_thr += (throttle - self.s_thr) * 0.25
        r = self.s_rpm
        thr = self.s_thr
        rn = np.clip((r - IDLE_RPM) / (MAX_RPM - IDLE_RPM), 0, 1)

        cyc_freq = r / 120.0          # cevrim/saniye (2 tur)
        inc = cyc_freq / SR
        span = frames * inc

        # --- atesleme darbe treni ---
        imp = np.zeros(frames)
        for a, amp in zip(FIRE_ANGLES, FIRE_AMPS):
            n0 = math.ceil(self.cyc - a)
            n1 = math.floor(self.cyc + span - a - 1e-9)
            for n in range(n0, n1 + 1):
                si = int(round((n + a - self.cyc) / inc))
                if 0 <= si < frames:
                    # her yanma biraz farkli -> canlilik
                    imp[si] += amp * (0.8 + 0.4 * np.random.rand())
        self.cyc += span
        if self.cyc > 1e6:
            self.cyc -= 1e6

        # --- govde: darbeler -> rezonans bankasi ---
        body = np.zeros(frames)
        for res in RESONATORS:
            y, res["zi"] = signal.lfilter(res["b"], res["a"], imp, zi=res["zi"])
            w = res["base"] + res["load"] * (0.3 + 0.7 * thr)
            body += w * y

        # --- sub-bass thump ---
        sub, self.zi_sub = signal.lfilter(SUB_B, SUB_A, imp, zi=self.zi_sub)
        sub *= 2.2

        sig = 4.4 * body + 1.9 * sub

        # --- egzoz patlamalari: gercek "pat" cekirdekleri, overlap-add ---
        global pop_burst
        if thr < 0.30 and r > 2200:
            self.crackle = min(1.0, self.crackle + 0.12)
        else:
            self.crackle *= 0.84
        ov = self.crackle * np.clip((r - 2000) / 3000, 0, 1)
        burst = pop_burst           # ani patlama (lift / vites / limiter)
        pop_burst *= 0.80
        intensity = max(ov, burst)

        work = np.zeros(frames + POP_LEN)
        work[:POP_LEN] += self.pop_tail
        if intensity > 0.01:
            # patlama yogunlugu: overrun seyrek "pat..pat", burst sik "patpatpat"
            rate = 18.0 * ov + 75.0 * burst        # saniyedeki patlama sayisi
            n_pop = np.random.poisson(rate * frames / SR)
            for _ in range(int(n_pop)):
                p = np.random.randint(0, frames)
                amp = (0.55 + 0.6 * np.random.rand()) * (0.5 + 0.8 * intensity)
                k = POP_KERNELS[np.random.randint(len(POP_KERNELS))]
                work[p:p + POP_LEN] += k * amp
        pop_out = work[:frames]
        self.pop_tail = work[frames:frames + POP_LEN]

        # ses modu: EGZOZ = patlamalar one cikar, MOTOR = boguk/mekanik
        pop_gain = 0.9 if exhaust_mode else 0.30
        sig += pop_out * pop_gain

        # --- intake / induction noise (motor modunda daha belirgin) ---
        noise = np.random.randn(frames)
        nz, self.zi_nz = signal.lfilter(NZ_B, NZ_A, noise, zi=self.zi_nz)
        nz_gain = (0.04 + 0.20 * thr) * (0.7 if exhaust_mode else 1.6)
        sig += nz * nz_gain

        # --- turbo spool (gecikmeli basinc) + whine ---
        global bov_burst
        btar = thr * float(np.clip((r - 1500) / 2500, 0, 1))
        # yukselis yavas (spool), dususte daha hizli
        self.boost += (btar - self.boost) * (0.030 if btar > self.boost else 0.10)
        tf = 1200 + self.boost * 2600 + rn * 3200      # boost & devirle tizleşir
        ti = tf / SR
        tph = self.turbo_ph + self.idx * ti
        self.turbo_ph = (self.turbo_ph + frames * ti) % 1.0
        tgain = (0.05 if exhaust_mode else 0.11) * (0.25 + 0.9 * self.boost)
        sig += np.sin(2 * np.pi * tph) * tgain

        # --- blow-off / dump valf "psshh" (boost varken gaz kesilince) ---
        self.bov = max(self.bov * 0.86, bov_burst)
        bov_burst *= 0.4
        if self.bov > 0.02:
            bn = np.random.randn(frames)
            bn, self.zi_bov = signal.lfilter(BOV_B, BOV_A, bn, zi=self.zi_bov)
            sig += bn * self.bov * (0.7 if exhaust_mode else 0.9)

        # --- mod bogma: MOTOR modunda ekstra lowpass (kabin/mekanik his) ---
        if not exhaust_mode:
            sig, self.zi_muf = signal.lfilter(MUF_B, MUF_A, sig, zi=self.zi_muf)

        # --- rev-limiter: motoru ritmik kes ("dut-dut-dut") ---
        if limiter:
            lf = 17.0 / SR                       # ~17 Hz kesme
            lph = self.lim_ph + self.idx * lf
            self.lim_ph = (self.lim_ph + frames * lf) % 1.0
            gate = 0.18 + 0.82 * (np.sin(2 * np.pi * lph) > -0.1)
            sig *= gate

        # --- mars motoru cizirtisi (calistirma aninda) ---
        if cranking:
            stf = 95.0 / SR                      # marş diŝlisi vinlamasi
            sph = self.turbo_ph * 3 + self.idx * stf
            whir = signal.sawtooth(2 * np.pi * sph) * 0.10
            whir *= (0.7 + 0.3 * np.random.rand(frames))
            sig = sig * 0.45 + whir              # chug + marş

        # --- cikis lowpass + yumusak limit (tanh = egzoz griftligi) ---
        sig, self.zi_out = signal.lfilter(OUT_B, OUT_A, sig, zi=self.zi_out)
        master = 0.62 if exhaust_mode else 0.55
        drive = (1.8 + 1.0 * thr) if exhaust_mode else (1.3 + 0.6 * thr)
        out[:, 0] = (np.tanh(sig * drive) * master).astype(np.float32)


eng = Engine()
stream = sd.OutputStream(samplerate=SR, blocksize=BLOCK, channels=1,
                         dtype="float32", callback=eng.callback)
stream.start()

# -------------------- Fizik (gercek RS7 C8 verileri) --------------------
# 4.0 TFSI V8: 800 Nm (2050-4500 rpm duz), ~6800 devir limiti,
# 0-100 ~3.6s, Vmax ~305 km/h (dynamic plus). 8 ileri otomatik.
TIRE_CIRC = 2.16                 # m (275/35 R21 cevre)
WHEEL_R = TIRE_CIRC / (2 * math.pi)
FINAL = 3.7                      # diferansiyel orani
MASS = 2075.0                    # kg
EFF = 0.85                       # aktarma verimi
CD, ROLL_V, ROLL0 = 0.26, 12.0, 120.0   # drag: CD*v^2 + ROLL_V*v + ROLL0
EB_TQ = 150.0                    # motor freni torku (Nm), gaz kapali iken
REV_K = 20.0                     # serbest devir ivme katsayisi (bos viteste)
REV_FR = 0.040                   # motor ic surtunmesi (devir dususu)
v = 0.0                          # m/s
rev_rpm = IDLE_RPM               # bos viteste serbest motor devri
crank_t = 0.0                    # mars suresi sayaci
idle_phase = 0.0                 # rolanti dalgalanma fazi


def pick_gear(v_ms):
    # hiza uygun en yuksek (rpm < ~5500) vitesi sec
    for g in range(1, 9):
        if rpm_from_speed(v_ms, g) < 5500:
            return g
    return 8


def torque(rp):
    # Nm: dusukte yukselir, 2050-4500 duz 800, sonra guc sinirli duser
    if rp < 2050:
        return 800.0 * (0.55 + 0.45 * rp / 2050.0)
    if rp <= 4500:
        return 800.0
    return max(150.0, 800.0 * (4500.0 / rp) * 0.92)


def rpm_from_speed(v_ms, g):
    return IDLE_RPM + (v_ms / TIRE_CIRC) * gear_ratios[g] * FINAL * 60.0


# -------------------- Ana dongu --------------------
running = True
prev_thr = 0.0
shift_timer = 0.0     # vites degisiminde kisa tork kesme
while running:
    dt = min(clock.tick(60) / 1000.0, 0.05)
    shift = pygame.key.get_mods() & pygame.KMOD_SHIFT
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False
        if e.type == pygame.KEYDOWN:
            if e.key == pygame.K_SPACE:
                if engine_on:                         # SPACE = calistir/durdur
                    engine_on = False
                    cranking = False
                elif not cranking:
                    cranking = True
                    crank_t = 0.0
            elif e.key == pygame.K_a:
                auto = not auto                       # A = oto/manuel
            elif e.key == pygame.K_m:
                exhaust_mode = not exhaust_mode       # M = motor/egzoz sesi
            elif e.key == pygame.K_n:
                gear = 0 if gear != 0 else pick_gear(v)   # N = bos vites
            elif pygame.K_1 <= e.key <= pygame.K_8:
                gear = e.key - pygame.K_0             # vitese tak (N'den de cikar)
                auto = False
            elif not auto and e.key == pygame.K_RIGHT and gear < 8:
                gear = max(1, gear + 1)
            elif not auto and e.key == pygame.K_LEFT and gear > 1:
                gear -= 1

    keys = pygame.key.get_pressed()
    # YUKARI = kismi gaz (feather edebilirsin), SHIFT+YUKARI = TAM GAZ
    if keys[pygame.K_UP] and shift:
        throttle = 1.0
    elif keys[pygame.K_UP]:
        throttle = min(throttle + 0.05, 0.80)
    else:
        throttle = max(throttle - 0.06, 0.0)
    # ASAGI = fren
    brake = 1.0 if keys[pygame.K_DOWN] else 0.0

    in_gear = gear != 0
    running = engine_on and not cranking

    # --- ates / devir durumu ---
    if cranking:
        crank_t += dt
        rpm = 260 + 70 * math.sin(crank_t * 38)        # marş chug
        rev_rpm = IDLE_RPM
        if crank_t > 0.75:                             # motor tutuştu
            cranking = False
            engine_on = True
            running = True
            pop_burst = 0.5                            # calisirken hafif blip
    elif not engine_on:
        rpm = 0.0
        throttle = 0.0
    elif in_gear:
        rpm = max(IDLE_RPM, min(rpm_from_speed(v, gear), MAX_RPM))
        rev_rpm = rpm                                  # N'e gecince yumusak
    else:
        rpm = rev_rpm                                  # bos vites: serbest devir

    # --- otomatik vites (sadece calisirken & viteste) ---
    if running and auto and in_gear and shift_timer <= 0:
        if gear < 8 and rpm > 6550 and throttle > 0.05:
            gear += 1
            shift_timer = 0.12
            pop_burst = 1.0                            # yukari vites -> BRAP bang
            bov_burst = max(bov_burst, 0.6)            # vites arasi turbo flutter
        elif gear > 1 and rpm < 2300 and throttle < 0.85:
            gear -= 1
        elif gear > 1 and throttle > 0.92 and rpm < 3200:
            gear -= 1                                  # kickdown
    shift_timer = max(0.0, shift_timer - dt)

    # --- egzoz patlamasi & blow-off tetikleyicileri ---
    if running:
        if prev_thr > 0.45 and throttle < 0.2 and rpm > 2600:
            pop_burst = 1.0                            # gaz birakma -> pat pat
        if prev_thr > 0.55 and throttle < 0.3 and rpm > 2400:
            bov_burst = 1.0                            # boost altinda lift -> psshh
    # rev-limiter sert kesme (gaz tam + devir tavanda)
    limiter = running and rpm >= REDLINE - 30 and throttle > 0.5
    prev_thr = throttle

    # --- tahrik & fizik ---
    if running and in_gear:
        eng_tq = throttle * torque(rpm)
        if rpm > IDLE_RPM + 120:
            ebrake = (1.0 - throttle) * EB_TQ * (0.35 + 0.65 * rpm / REDLINE)
            eng_tq -= ebrake
        drive = eng_tq * gear_ratios[gear] * FINAL * EFF / WHEEL_R
        if rpm >= REDLINE or shift_timer > 0:
            drive = min(drive, 0.0)   # devir limiti / vites kesintisi
    else:
        drive = 0.0                   # bos vites / motor kapali

    drag = CD * v * v + ROLL_V * v + ROLL0
    accel = (drive - drag) / MASS
    if brake > 0:
        accel -= 9.5 * brake          # fren ~0.97g
    if throttle == 0 and v < 0.4 and brake == 0 and in_gear:
        accel = -v / dt
    v = max(0.0, v + accel * dt)
    if brake > 0 and v < 0.3:
        v = 0.0

    # --- devir guncelle ---
    if running and in_gear:
        rpm = max(IDLE_RPM, min(rpm_from_speed(v, gear), MAX_RPM))
    elif running:
        # bos viteste serbest devir dinamigi (free-rev)
        net = throttle * torque(rev_rpm) - REV_FR * (rev_rpm - IDLE_RPM)
        if rev_rpm >= REDLINE and throttle > 0.05:
            net = -REV_FR * (rev_rpm - IDLE_RPM)      # devir limiti: yakit kes
            pop_burst = max(pop_burst, 0.8)
        rev_rpm = min(MAX_RPM, max(IDLE_RPM, rev_rpm + net * REV_K * dt))
        rpm = rev_rpm

    # --- rolanti dalgalanmasi (lope): motoru canli gosterir ---
    if running and throttle < 0.05 and rpm < IDLE_RPM + 220:
        idle_phase += dt
        rpm += 13 * math.sin(idle_phase * 6.3) + (np.random.rand() - 0.5) * 8

    speed = v * 3.6

    # --- HUD ---
    screen.fill((10, 10, 12))
    over = rpm >= REDLINE
    col = (255, 60, 60) if over else (255, 255, 255)
    screen.blit(font.render(f"RPM: {int(rpm)}", True, col), (50, 40))
    screen.blit(font.render(f"KM/H: {int(speed)}", True, (255, 255, 255)), (50, 105))
    mode = "OTO" if auto else "MANUEL"
    snd = "EGZOZ" if exhaust_mode else "MOTOR"
    gtxt = "N" if gear == 0 else str(gear)
    screen.blit(font.render(f"VITES: {gtxt}  [{mode}]", True, (255, 255, 255)), (50, 170))
    gcol = (120, 220, 120) if throttle > 0 else ((255, 120, 120) if brake else (255, 255, 255))
    screen.blit(font.render(f"GAZ: %{int(throttle*100)}   SES: {snd}", True, gcol), (50, 235))
    # motor durumu rozeti
    if cranking:
        est, ecol = "MARŞ...", (255, 200, 80)
    elif engine_on:
        est, ecol = "● CALISIYOR", (90, 220, 90)
    else:
        est, ecol = "○ KAPALI - SPACE ile calistir", (200, 90, 90)
    screen.blit(pygame.font.SysFont(None, 30).render(est, True, ecol), (360, 48))
    small = pygame.font.SysFont(None, 24)
    screen.blit(small.render("SPACE=calistir/durdur  YUKARI=gaz  SHIFT+YUKARI=TAM GAZ  ASAGI=fren", True, (150, 150, 160)), (50, 290))
    screen.blit(small.render("A=oto/manuel  M=motor/egzoz  N=bos vites  SOL/SAG=vites", True, (150, 150, 160)), (50, 312))
    bw = int((rpm - IDLE_RPM) / (MAX_RPM - IDLE_RPM) * (WIDTH - 100))
    pygame.draw.rect(screen, (40, 40, 40), (50, 340, WIDTH - 100, 22))
    pygame.draw.rect(screen, (255, 80, 80) if over else (80, 200, 80),
                     (50, 340, max(0, bw), 22))
    pygame.display.flip()

stream.stop()
stream.close()
pygame.quit()
